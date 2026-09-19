#!/usr/bin/env python3
"""桌面剔除（补丁③）离线回归测试 —— 2026-09-18，**不用起仿真**。

复现的现场故障（2026-09-18 抓取日志）：
    掩码法轴心 base(+0.385, +0.051)  vs  地面测距 base(+0.458, +0.056)   差 74 mm
    已知尺寸测距: 掩码宽 106 px → 隐含宽度 0.066 m（目录 0.038~0.097）轴距 0.332 m
⇒ 掩码法把糖盒报**近**了 ~100 mm，用户现场看到"抓取点不够靠后、夹爪停在物体前面"✗

机理（本测试用合成场景复现）：
    相机只高出桌面 39 mm ⇒ 视线几乎与桌面平行 ⇒ **同一条视线**上，
    物体前方那条桌面比物体近得多（0.45 m 处的盒子，它前方桌面只有 0.35 m）
    ⇒ 掩码底边吃进那几行桌面（SAM2 常把接触区/阴影带进来）时，
      "最近 5% 分位"取到的就是桌面，而不是物体 ✗

测试做法：合成 640×480 的深度图 + 掩码（盒子正面 + 其下方几行桌面），
跑**真身** `_mask_points_cam`，比较"开/关桌面剔除"两种设置下掩码的最近面深度。

用法：
    cd ~/Robocup@home_ws && source install/setup.bash
    python3 src/HANDOFF_harness/test_table_pixel_filter.py
退出码 0 = 通过
"""

import math
import sys
import types

import numpy as np

sys.path.insert(0, "/home/xing/Robocup@home_ws/src/turtlebot3_manipulation_grasp/scripts")

FAIL = []


def check(name, ok, detail=""):
    print("  {} {}{}".format("✓" if ok else "✗", name, ("  ← " + detail) if detail else ""))
    if not ok:
        FAIL.append(name)


# ── 合成相机模型（与现场一致：fx=fy=530.47、cx=320、cy=240、相机高出桌面 39 mm、
#    俯仰 0°、朝 base +x）────────────────────────────────────────────────────
FX = FY = 530.47
CX, CY = 320.0, 240.0
H, W = 480, 640
SUPPORT_Z = 0.780                      # 桌面顶
CAM_Z = SUPPORT_Z + 0.039              # 相机高（日志："相机高 39mm"）
# 相机光学系 → base：x_cam=base −y、y_cam=base −z、z_cam=base +x
R = np.array([[0.0, 0.0, 1.0],
              [-1.0, 0.0, 0.0],
              [0.0, -1.0, 0.0]])
# R 对应的四元数（下面用 R 反解校验，避免手写符号错）
QW = 0.5 * math.sqrt(1.0 + R[0, 0] + R[1, 1] + R[2, 2])
QX = (R[2, 1] - R[1, 2]) / (4.0 * QW)
QY = (R[0, 2] - R[2, 0]) / (4.0 * QW)
QZ = (R[1, 0] - R[0, 1]) / (4.0 * QW)

BOX_FRONT = 0.45            # 盒子正面到机器人的距离（日志里"已知尺寸测距"给 0.45 m）
BOX_HALF_W = 0.0445         # 糖盒宽 89 mm
BOX_TOP = SUPPORT_Z + 0.175  # 糖盒高 175 mm
TOL = 0.015


def quat_to_R(qx, qy, qz, qw):
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
    ])


def base_to_pix(p_base):
    """base 点 → (u, v, 深度 z_cam)"""
    p_cam = R.T.dot(np.asarray(p_base) - np.array([0.0, 0.0, CAM_Z]))
    z_c = p_cam[2]
    return (CX + FX * p_cam[0] / z_c, CY + FY * p_cam[1] / z_c, z_c)


def synth_scene():
    """深度图 + 掩码：盒子正面（垂直面 @0.45 m） + 其下方 12 行桌面。

    返回 (mask, depth, 桌面带里的最小深度=应被剔除的"假最近面")
    """
    depth = np.full((H, W), np.inf, dtype=np.float32)
    mask = np.zeros((H, W), dtype=bool)

    # ① 盒子正面：base x = 0.45，y ∈ ±44.5 mm，z ∈ [0.78, 0.955]
    u0, v0, _ = base_to_pix([BOX_FRONT, -BOX_HALF_W, SUPPORT_Z])
    u1, v1, _ = base_to_pix([BOX_FRONT, +BOX_HALF_W, BOX_TOP])
    cu0, cu1 = int(round(min(u0, u1))), int(round(max(u0, u1)))
    cv0, cv1 = int(round(min(v0, v1))), int(round(max(v0, v1)))
    mask[cv0:cv1 + 1, cu0:cu1 + 1] = True
    depth[cv0:cv1 + 1, cu0:cu1 + 1] = BOX_FRONT
    box_bottom_v = max(cv0, cv1)

    # ② 掩码底边吃进的"物体前方桌面"（12 行，深度按桌面平面算 ⇒ 比盒子近得多）
    fake_near = None
    for dv in range(1, 31):
        v = box_bottom_v + dv
        if v >= H:
            break
        z_tab = (CAM_Z - SUPPORT_Z) * FY / (v - CY)     # 该行若在桌面上应有的深度
        mask[v, cu0:cu1 + 1] = True
        depth[v, cu0:cu1 + 1] = z_tab
        fake_near = z_tab if fake_near is None else min(fake_near, z_tab)
    return mask, depth, fake_near


class Stub:
    """只借真身 `_mask_points_cam` 需要的成员。"""
    _mask_points_cam = None            # 在 main 里绑定

    def __init__(self, table_pixel_tol):
        self.mask_erode_px = 0         # 关掉腐蚀，让本测试只考桌面剔除 ✓
        self.max_range = 2.0
        self.support_z = SUPPORT_Z
        self.table_pixel_tol = table_pixel_tol
        self.object_sizes = {"sugar_box": (0.038, 0.089, 0.175)}
        self.plane_min_den = 0.10


def main():
    sys.path.insert(0, "/home/xing/Robocup@home_ws/src/turtlebot3_manipulation_grasp/scripts")
    import detect_grasp_target_node as D
    Stub._mask_points_cam = D.DetectGraspTargetNode._mask_points_cam

    # 先自检合成场景的四元数与矩阵一致（否则后面结论不可信）
    q = (QX, QY, QZ, QW)
    R2 = quat_to_R(*q)
    check("合成相机四元数与旋转矩阵一致（|ΔR|<1e-9）",
          float(np.abs(R2 - R).max()) < 1e-9, "max|ΔR|={:.2e}".format(float(np.abs(R2 - R).max())))

    mask, depth, fake_near = synth_scene()
    tf = types.SimpleNamespace(transform=types.SimpleNamespace(
        rotation=types.SimpleNamespace(x=QX, y=QY, z=QZ, w=QW),
        translation=types.SimpleNamespace(x=0.0, y=0.0, z=CAM_Z)))
    k = (FX, 0.0, CX, 0.0, FY, CY)    # 内部只用 k[0],k[2],k[4],k[5]

    print("\n合成场景：盒子正面 {:.2f} m（高 {:.0f} mm），掩码底边多吃 30 行桌面".format(
        BOX_FRONT, 1000 * (BOX_TOP - SUPPORT_Z)))
    print("  那几行桌面里最近的深度 = {:.3f} m（比盒子近 {:.0f} mm）".format(
        fake_near, 1000 * (BOX_FRONT - fake_near)))
    check("合成场景确实复现了「桌面比物体近 ~100 mm」",
          (BOX_FRONT - fake_near) > 0.06, "差 {:.0f} mm".format(1000 * (BOX_FRONT - fake_near)))

    # ── A：关掉桌面剔除（=改动前行为）→ 最近面被桌面拉近
    gp_off = Stub(0.0)
    pts_off, ys_off, st_off = gp_off._mask_points_cam(
        mask, depth, k, "sugar_box", patch=True, tf=tf)
    z5_off = float(np.percentile(pts_off[:, 2], 5))       # _contract_point 用的"可见面"
    print("\nA. 不做桌面剔除（table_pixel_tol=0）：可见面(5% 分位) = {:.3f} m".format(z5_off))
    check("不做剔除时可见面被拉近（偏浅 ⇒ 夹爪停在物体前面 ✗）",
          (BOX_FRONT - z5_off) > 0.010, "偏近 {:.0f} mm".format(1000 * (BOX_FRONT - z5_off)))

    # ── B：打开桌面剔除（默认）→ 可见面回到盒子正面
    gp_on = Stub(TOL)
    pts_on, ys_on, st_on = gp_on._mask_points_cam(
        mask, depth, k, "sugar_box", patch=True, tf=tf)
    z5_on = float(np.percentile(pts_on[:, 2], 5))
    print("\nB. 打开桌面剔除（table_pixel_tol={:.0f} mm）：可见面 = {:.3f} m"
          "（剔除 {:.0f}% 像素）".format(1000 * TOL, z5_on, 100.0 * st_on.get("drop_table", 0.0)))
    check("剔除后可见面 ≈ 盒子正面（误差 ≤15 mm）", abs(z5_on - BOX_FRONT) <= 0.015,
          "差 {:+.0f} mm".format(1000 * (z5_on - BOX_FRONT)))
    check("剔除确实改善了偏浅（前后对比）",
          abs(z5_on - BOX_FRONT) < abs(z5_off - BOX_FRONT) - 0.010,
          "剔除前 {:+.0f} mm → 剔除后 {:+.0f} mm".format(
              1000 * (z5_off - BOX_FRONT), 1000 * (z5_on - BOX_FRONT)))
    check("stats 报出剔除比例（供现场日志判读 ✓）", st_on.get("drop_table", 0.0) > 0.05,
          "drop_table={:.3f}".format(st_on.get("drop_table", 0.0)))

    # ── C：掩码整个落在桌面上（误检）⇒ 不剔除 + 回告警，交给尺寸/高度核对
    mask_t = np.zeros((H, W), dtype=bool)
    depth_t = np.zeros((H, W), dtype=np.float32)
    mask_t[300:340, 300:360] = True
    for v in range(300, 340):
        depth_t[v, 300:360] = (CAM_Z - SUPPORT_Z) * FY / (v - CY)   # 真实桌面深度
    gp_c = Stub(TOL)
    got = gp_c._mask_points_cam(mask_t, depth_t, k, "sugar_box", patch=True, tf=tf)
    check("整块掩码都是桌面时不剔除（回 table_only 告警，交给尺寸/高度核对 ✗）",
          got is not None and got[2].get("table_only", 0.0) > 0.0,
          "stats={}".format(None if got is None else got[2]))

    # ── D：远处物体（1.2 m）不受影响（剔除只对"落在桌面上"的像素生效）
    far = np.zeros((H, W), dtype=bool)
    depth_f = np.full((H, W), np.inf, dtype=np.float32)
    u0, v0, _ = base_to_pix([1.2, -0.03, SUPPORT_Z])
    u1, v1, _ = base_to_pix([1.2, 0.03, SUPPORT_Z + 0.12])
    far[int(round(min(v0, v1))):int(round(max(v0, v1))) + 1,
        int(round(min(u0, u1))):int(round(max(u0, u1))) + 1] = True
    depth_f[far] = 1.2
    gp_d = Stub(TOL)
    pts_d, _, st_d = gp_d._mask_points_cam(far, depth_f, k, "sugar_box", patch=True, tf=tf)
    check("远处（1.2 m）物体的像素一个都没被误剔 ✓",
          st_d.get("drop_table", 0.0) < 0.05 and abs(
              float(np.percentile(pts_d[:, 2], 5)) - 1.2) < 0.02,
          "drop={:.3f} z5={:.3f}".format(st_d.get("drop_table", 0.0),
                                         float(np.percentile(pts_d[:, 2], 5))))

    print("\n" + ("全部通过 ✓" if not FAIL else "有 {} 条不过 ✗：{}".format(len(FAIL), FAIL)))
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
