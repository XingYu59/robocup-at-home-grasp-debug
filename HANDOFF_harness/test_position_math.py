#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线自测：地面约束测距（_ground_point）与桌平面校验（_table_plane_check）的数学。

不跑仿真：自己造一个合成场景（相机外参 + 桌面 + 圆柱/长方体），
用 z-buffer 渲染出"掩码"和"深度图"，再喂给检测节点里的真身函数，
看它能不能把物体轴心恢复回来 ✓

用法：  python3 HANDOFF_harness/test_position_math.py
（需要先 source install/setup.bash，因为要 import 节点模块里的 msg/srv）
"""
import math
import os
import sys

import numpy as np

SCRIPTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                       "turtlebot3_manipulation_grasp", "scripts")
sys.path.insert(0, os.path.abspath(SCRIPTS))

import detect_grasp_target_node as D  # noqa: E402

FX = FY = 530.47
CX, CY = 320.0, 240.0
W, H = 640, 480
K = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
SUPPORT_Z = 0.780


class _V:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def rot_to_quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    if i == 0:
        s = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s)
    if i == 1:
        s = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s)
    s = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s)


def make_tf(cam_xyz, look_at):
    """相机放在 cam_xyz，光轴指向 look_at（base 系）→ TransformStamped 替身。"""
    f = np.array(look_at, float) - np.array(cam_xyz, float)
    f /= np.linalg.norm(f)
    up = np.array([0.0, 0.0, 1.0])
    r = np.cross(f, up)
    r /= np.linalg.norm(r)
    d = np.cross(f, r)                       # 光学系 y 轴（图像下方）= forward × right
    R = np.stack([r, d, f], axis=1)          # 列 = 相机轴在 base 系里的表示
    qx, qy, qz, qw = rot_to_quat(R)
    return _V(transform=_V(rotation=_V(x=qx, y=qy, z=qz, w=qw),
                           translation=_V(x=cam_xyz[0], y=cam_xyz[1], z=cam_xyz[2]))), R


def render(points, tf):
    """base 系点云 → (每像素深度图, 每像素标签)。标签: 0 空 / 1 桌面 / 2 物体。"""
    R = D.DetectGraspTargetNode._rot_of(tf)
    t = np.array([tf.transform.translation.x, tf.transform.translation.y,
                  tf.transform.translation.z])
    pc = (points - t).dot(R)                 # R^T·(p-t) == (p-t)·R
    z = pc[:, 2]
    ok = np.isfinite(z) & (z > 0.05) & (z < 3.0)
    u = np.round(FX * pc[ok, 0] / z[ok] + CX).astype(int)
    v = np.round(FY * pc[ok, 1] / z[ok] + CY).astype(int)
    zz = z[ok]
    keep = (u >= 0) & (u < W) & (v >= 0) & (v < H)
    u, v, zz = u[keep], v[keep], zz[keep]
    depth = np.full((H, W), np.inf)
    order = np.argsort(-zz)                  # 远的先写，近的覆盖 ⇒ z-buffer
    depth[v[order], u[order]] = zz[order]
    return depth


def table_points(x0, x1, y0, y1, step=0.004):
    xs = np.arange(x0, x1, step)
    ys = np.arange(y0, y1, step)
    gx, gy = np.meshgrid(xs, ys)
    return np.stack([gx.ravel(), gy.ravel(), np.full(gx.size, SUPPORT_Z)], axis=1)


def cylinder_points(cx, cy, r, h, n_theta=400, n_h=60):
    th = np.linspace(0, 2 * math.pi, n_theta)
    hh = np.linspace(0, h, n_h)
    gth, ghh = np.meshgrid(th, hh)
    return np.stack([cx + r * np.cos(gth).ravel(), cy + r * np.sin(gth).ravel(),
                     SUPPORT_Z + ghh.ravel()], axis=1)


def fake_node():
    n = _V(support_z=SUPPORT_Z, plane_min_den=0.15, max_range=2.0, mask_erode_px=0,
           yaw_backoff=True, object_yaw_map=0.0,
           target_frame="base_footprint", map_frame="map",
           object_sizes={"tomato_soup_can": (0.066, 0.066, 0.115),
                         "sugar_box": (0.038, 0.089, 0.175)})
    n._tf_buffer = _V(lookup_transform=lambda *a, **k: _V(transform=_V(
        rotation=_V(x=0.0, y=0.0, z=0.0, w=1.0),
        translation=_V(x=0.0, y=0.0, z=0.0))))
    # 把真身的方法挂到替身上（不启 rclpy 也能测数学）
    N = D.DetectGraspTargetNode
    n._rot_of = N._rot_of
    n._plane_z_at = lambda u, v, k, R, t: N._plane_z_at(n, u, v, k, R, t)
    n._backoff = lambda cls, vd, tf: N._backoff(n, cls, vd, tf)
    return n


def run_case(name, cls, cam_xyz, obj_xy, obj_r=None, obj_hw=None, obj_h=0.115,
             yaw=0.0):
    look = (obj_xy[0], obj_xy[1], SUPPORT_Z + obj_h / 2.0)
    tf, R = make_tf(cam_xyz, look)
    # 相机外参自检：_rot_of 应还原 R
    assert np.allclose(D.DetectGraspTargetNode._rot_of(tf), R, atol=1e-9), "quat 转换不一致"
    if obj_r is not None:
        pts = cylinder_points(obj_xy[0], obj_xy[1], obj_r, obj_h)
    else:
        hx, hy = obj_hw
        cs, sn = math.cos(yaw), math.sin(yaw)
        loc = np.stack(np.meshgrid(np.linspace(-hx, hx, 60), np.linspace(-hy, hy, 60),
                                   np.linspace(0, obj_h, 60)), axis=-1).reshape(-1, 3)
        loc[:, :2] = loc[:, :2].dot(np.array([[cs, -sn], [sn, cs]]).T)
        pts = loc + np.array([obj_xy[0], obj_xy[1], SUPPORT_Z])
    depth_obj = render(pts, tf)
    depth_tab = render(table_points(obj_xy[0] - 0.75, obj_xy[0] + 0.75,
                                    obj_xy[1] - 0.40, obj_xy[1] + 0.40, step=0.003), tf)
    depth = depth_obj.copy()
    tab_near = np.isfinite(depth_tab) & (depth_tab < depth)
    depth[tab_near] = depth_tab[tab_near]
    mask = np.isfinite(depth_obj)
    depth[~np.isfinite(depth)] = 0.0

    node = fake_node()
    axis, info = D.DetectGraspTargetNode._ground_point(node, cls, mask, K, tf)
    chk = D.DetectGraspTargetNode._table_plane_check(node, depth, K, tf)
    err = None if axis is None else (axis[0] - obj_xy[0], axis[1] - obj_xy[1])
    print("\n── {} ──".format(name))
    print("  相机 base=({:.2f},{:.2f},{:.2f}) 高桌面 {:.3f} m  掩码 {} px"
          .format(cam_xyz[0], cam_xyz[1], cam_xyz[2], cam_xyz[2] - SUPPORT_Z,
                  int(mask.sum())))
    if chk:
        print("  桌平面校验: {} px → z={:.4f}±{:.4f} 差 {:+.1f} mm 斜率 {:.3f} mm/px"
              .format(chk["n"], chk["mean"], chk["std"],
                      (chk["mean"] - SUPPORT_Z) * 1000, chk["slope"] * 1000))
    else:
        print("  桌平面校验: 无（相机没对着桌面）")
    if axis is None:
        print("  ✗ 地面测距失败: {}".format(info.get("why", "?")))
        return None
    print("  地面测距: 轴心 base=({:+.4f},{:+.4f}) 真值 ({:+.4f},{:+.4f}) "
          "→ 误差 ({:+.1f},{:+.1f}) mm".format(axis[0], axis[1], obj_xy[0], obj_xy[1],
                                               err[0] * 1000, err[1] * 1000))
    print("    前缘距 {:.3f} m 后退 {:.0f} mm（{}）底边抖动 {:.1f} mm".format(
        info["range_front"], info["back"] * 1000, info["back_why"], info["jitter"] * 1000))
    return err


def main():
    print("=" * 74)
    print("地面约束测距 / 桌平面校验 —— 离线合成场景自测")
    print("=" * 74)
    ok = True
    # ① 番茄罐：相机前伸 0.35 m、高于桌面 0.27 m、俯视 ~38°（正常观测几何）
    e = run_case("番茄罐 · 俯视 38°", "tomato_soup_can", (0.35, 0.00, 1.05), (0.70, 0.05), obj_r=0.033)
    ok &= e is not None and max(abs(e[0]), abs(e[1])) < 0.006
    # ② 同一物体，相机拉远到 2 m、只高 0.12 m（浅视角 → 考验条件数保护）
    e = run_case("番茄罐 · 浅视角（相机高出桌面仅 0.12 m）→ 应被条件数保护拦下",
                 "tomato_soup_can", (0.00, 0.00, 0.90), (1.60, 0.05), obj_r=0.033)
    ok &= e is None      # 视线几乎与桌面平行 ⇒ 拒绝采用（回退掩码法），而不是给个错值 ✓
    # ③ 糖盒：正对薄边（后退 19 mm）——支撑函数应给出 19 mm
    e = run_case("糖盒 · 斜看 49°（顺便验支撑函数后退量）", "sugar_box", (0.35, -0.30, 1.05), (0.70, 0.10),
                 obj_hw=(0.019, 0.0445), obj_h=0.175, yaw=0.0)
    ok &= e is not None and max(abs(e[0]), abs(e[1])) < 0.006
    # ④ 糖盒：正对宽边（后退 44.5 mm）
    tf, _ = make_tf((0.70, -0.70, 1.05), (0.70, 0.10, 0.8675))
    node = fake_node()
    back, why = D.DetectGraspTargetNode._backoff(
        node, "sugar_box", np.array([0.0, 1.0, 0.0]), tf)
    print("\n── 长方体后退量（可见面→轴心）──")
    print("  正对宽边 期望 44.5 mm，公式给 {:.1f} mm（{}）".format(back * 1000, why))
    ok &= abs(back - 0.0445) < 0.001
    print("\n" + "=" * 74)
    print("结论: {}".format("全部通过 ✓（数学正确，可以被仿真里的日志复核）" if ok
                            else "有未通过项 ✗ —— 见上面各条"))
    print("=" * 74)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
