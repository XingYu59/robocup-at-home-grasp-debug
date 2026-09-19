#!/usr/bin/env python3
"""handle_detect 的离线集成自测 —— 不起 ROS 服务、不跑视觉模型。

为什么需要它（2026-09-18 的教训）：多帧投票+融合改动静态检查（py_compile/pyflakes/
AST）全过，仿真里却当场 TypeError —— frames 元素是 (out, ctx)，三处解包把顺序
写反，fr 拿到 ctx、ctx["idx"] 是 int ✗。这类流程 bug 只有【真跑一遍 handle_detect】
才能抓住。本脚本把它的依赖全部换成可控的假件（相机帧/检测框/TF），端到端验证：

    ① 多帧投票：3 帧里同类 3 票 → 采纳，置信度取中位数
    ② 位置融合：逐帧各算一个轴心点 → 按轴取中位数；代表帧 = 离中位数最近的那帧
    ③ 修复回归：全程无 TypeError/NameError（就是 09-18 现场崩掉的那条路径）

用法（要先 source ROS + 工作区，和 test_position_math.py 一样）：
    source /opt/ros/humble/setup.bash && source ~/Robocup@home_ws/install/setup.bash
    python3 src/HANDOFF_harness/test_handle_detect.py
"""

import os
import sys
import threading
import types

import numpy as np

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "turtlebot3_manipulation_grasp", "scripts"))

import detect_grasp_target_node as D                     # noqa: E402
from builtin_interfaces.msg import Time as TimeMsg       # noqa: E402
from sensor_msgs.msg import CameraInfo, Image            # noqa: E402
from turtlebot3_manipulation_grasp.srv import DetectGraspTarget  # noqa: E402

# ── 合成场景参数（和真实机器人同一量级）───────────────────────────────
FX = FY = 530.0
CX, CY = 320.0, 240.0
CAM_T = (0.08, 0.0, 0.82)          # 相机在 base_footprint 的位置（高出桌面 0.04 m）
SUPPORT_Z = 0.78
CAN = (0.066, 0.066, 0.101)        # tomato_soup_can: (depth, width, height)
CAN_DEPTH = 0.45                   # 掩码内深度（可见表面）
FAR_DEPTH = 2.0                    # 掩码外（桌面/背景）
RECT_V = (170, 310)                # 掩码的行范围（高度核对要过）
RECT_U = (270, 370)                # 掩码的列范围（中心 u=320）
# 相机光学系 → base_footprint 的旋转：optical z→base x、x→−y、y→−z
# 对应四元数 (x,y,z,w)=(0.5,0.5,0.5,0.5)（绕 (1,1,1)/√3 转 120°）


def _tf():
    tr = types.SimpleNamespace(
        translation=types.SimpleNamespace(x=CAM_T[0], y=CAM_T[1], z=CAM_T[2]),
        rotation=types.SimpleNamespace(x=0.5, y=0.5, z=0.5, w=0.5))
    return types.SimpleNamespace(transform=tr)


def _map_tf():
    tr = types.SimpleNamespace(
        translation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
        rotation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
    return types.SimpleNamespace(transform=tr)


def _img(stamp_i, h=8, w=8):
    m = Image()
    m.height, m.width, m.encoding = h, w, "rgb8"
    m.step = w * 3
    m.data = bytes(h * w * 3)
    m.header.stamp.sec, m.header.stamp.nanosec = 0, stamp_i
    return m


def _depth_msg(arr, stamp_i):
    m = Image()
    m.height, m.width = arr.shape
    m.encoding = "32fc1"
    m.step = arr.shape[1] * 4
    m.data = arr.astype(np.float32).tobytes()
    m.header.stamp.sec, m.header.stamp.nanosec = 0, stamp_i
    return m


def _info():
    c = CameraInfo()
    c.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
    return c


def _scene(du):
    """一张合成帧：掩码 = 罐子侧面矩形（横向偏移 du 像素），深度 = 0.45/2.0。"""
    mask = np.zeros((480, 640), np.uint8)
    depth = np.full((480, 640), FAR_DEPTH, np.float32)
    v0, v1 = RECT_V
    u0, u1 = RECT_U[0] + du, RECT_U[1] + du
    mask[v0:v1 + 1, u0:u1 + 1] = 1
    depth[v0:v1 + 1, u0:u1 + 1] = CAN_DEPTH
    return mask, depth


def _det(du, score):
    mask, _ = _scene(du)
    return {"phrase": "tomato soup can", "score": score,
            "box_xyxy": [float(RECT_U[0] + du), float(RECT_V[0]),
                         float(RECT_U[1] + du), float(RECT_V[1])],
            "mask": mask}


class _Log:
    """假 logger：get_logger() 返回它自己，info/warn/error 全部收进 lines。"""

    def __init__(self):
        self.lines = []
        for lvl in ("info", "warn", "error", "debug"):
            setattr(self, lvl, self._make(lvl))

    def _make(self, lvl):
        def f(msg, *a):
            self.lines.append("[{}] {}".format(lvl, msg.format(*a) if a else msg))
        return f


class _Grabber:
    """假的 _grab_frame：按序吐出给定帧（stamp 互不相同 → fresh=True）。"""

    def __init__(self, depth_arrs):
        self.i = 0
        self.arrs = list(depth_arrs)

    def __call__(self, last_key, timeout=None):
        self.i += 1
        arr = self.arrs.pop(0) if self.arrs else np.full((480, 640), FAR_DEPTH, np.float32)
        return _img(self.i), _depth_msg(arr, self.i), _info(), (0, self.i), True


class _Detect:
    """假的 _detect_pass：prompt=None（第 1 帧默认词表）→ 空；否则按序吐 dets。"""

    def __init__(self, dets):
        self.calls = 0
        self.dets = list(dets)

    def __call__(self, bgr, prompt=None, use_reviewer=True):
        self.calls += 1
        if prompt is None:
            return []
        return [self.dets.pop(0)] if self.dets else []


def make_node(dets, depth_arrs, vote_frames):
    """拼一个假节点：属性齐全 + 真身方法绑定（不 rclpy.init，不起模型）。"""
    n = types.SimpleNamespace()
    N = D.DetectGraspTargetNode
    # —— 依赖全部换成假件 ——
    n._wait_streams = lambda timeout: True
    n._ensure_pipeline = lambda: True
    n._lock = threading.Lock()
    n._img, n._depth, n._info = _img(1), _depth_msg(np.zeros((8, 8), np.float32), 1), _info()
    n._lookup_cam_to_target = lambda: _tf()
    n._tf_buffer = types.SimpleNamespace(lookup_transform=lambda *a, **k: _map_tf())
    n._grab_frame = _Grabber(depth_arrs)
    n._detect_pass = _Detect(dets)
    n._publish_annotated = lambda *a, **k: None
    n._vp_mod = types.SimpleNamespace(classify_phrase=lambda p: None)
    lg = _Log()
    n.get_logger = lambda: lg
    n.get_clock = lambda: types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(to_msg=lambda: TimeMsg()))
    n.get_parameter = lambda name: types.SimpleNamespace(
        value={"contract_range_offset": -0.022}[name])
    # —— 参数/目录（照 __init__ 的语义手工填）——
    n.target_frame = "base_footprint"
    n.camera_frame = "camera_rgb_optical_frame"
    n.map_frame = "map"
    n.support_z = SUPPORT_Z
    n.object_yaw_map = 0.0
    n.stream_timeout = 5.0
    n.max_range = 3.0
    n.vote_frames = vote_frames
    n.grasp_mode = "auto"
    n.grasp_prompt = "tomato soup can ."
    n.position_source = "auto"
    n.mask_erode_px = 0
    n.plane_min_den = 0.15                 # 相机高桌面 0.04 m → 地面测距会被条件数保护拦下
    n.yaw_backoff = True
    n.table_check = False
    n.size_check, n.size_lo, n.size_hi = True, 0.7, 1.4
    n.height_check, n.height_lo, n.height_hi = True, 0.55, 1.80
    n.contract_axis_pct, n.backoff_scale = 5.0, 1.0
    n.catalog = {"tomato_soup_can": {"dims": CAN, "graspable": True}}
    n.object_sizes = {"tomato_soup_can": CAN}
    # —— 真身方法绑定到替身上 ——
    for m in ("handle_detect", "_classify_catalog", "_mask_points_cam", "_size_ok",
              "_height_ok", "_contract_point", "_ground_point", "_backoff",
              "_plane_z_at", "_size_range_crosscheck", "_fresh_enough"):
        setattr(n, m, getattr(N, m).__get__(n))
    n._to_bgr = N._to_bgr
    n._to_depth = N._to_depth
    n._to_target_frame = N._to_target_frame
    n._rot_of = N._rot_of
    return n, lg


def run_once(dets, vote_frames, label):
    # ★ 第 1 次 _frame 是 f0（默认词表，det 为空）也会吃掉一张帧
    #   → 先放一张哑深度，之后的场景深度才与 dets 逐一对齐
    depths = [np.full((480, 640), FAR_DEPTH, np.float32)]
    depths += [arr for _m, arr in (_scene(d["du"]) for d in dets)]
    node, lg = make_node([d["det"] for d in dets], depths, vote_frames)
    req = DetectGraspTarget.Request()
    req.class_ids = ["tomato_soup_can"]
    resp = node.handle_detect(req, DetectGraspTarget.Response())
    if not resp.success or not resp.targets:
        print("── {} ── ✗ 失败：success={} msg={}".format(label, resp.success, resp.message))
        for line in lg.lines[-8:]:
            print("    " + line)
        return None, lg
    t = resp.targets[0]
    print("── {} ── ✓ {} conf={:.3f} base=({:+.4f},{:+.4f},{:.3f})  [{} 次检测调用]".format(
        label, t.class_id, t.confidence, t.point.x, t.point.y, t.point.z,
        node._detect_pass.calls))
    return t, lg


def main():
    print("=" * 74)
    print("handle_detect 多帧投票 + 中位数融合 —— 离线集成自测")
    print("=" * 74)
    ok = True

    # ① 三帧各偏 0 / +8 / +16 px（模拟单帧掩码的横向抖动），分数 0.70/0.80/0.75
    t3, lg3 = run_once([{"du": 0, "det": _det(0, 0.70)},
                        {"du": 8, "det": _det(8, 0.80)},
                        {"du": 16, "det": _det(16, 0.75)}], 3, "三帧融合")
    if t3 is None:
        return 1
    # ② 中位数的 oracle：单独跑"中位那一帧"（du=+8）应是同样结果
    t_mid, _ = run_once([{"du": 8, "det": _det(8, 0.7)}], 1, "单帧(中位帧 du=+8)")
    t_lo, _ = run_once([{"du": 0, "det": _det(0, 0.7)}], 1, "单帧(du=0)")
    if t_mid is None or t_lo is None:
        return 1

    # 判据 1：置信度 = 三帧分数的中位数 0.75
    good = abs(t3.confidence - 0.75) < 1e-9
    print("  {} 置信度中位数: {:.3f}（期望 0.75）".format("✓" if good else "✗", t3.confidence))
    ok &= good
    # 判据 2：融合点 == 中位帧单独跑出的点（x/y 逐轴中位数都应落在中位帧上）
    d_mid = max(abs(t3.point.x - t_mid.point.x), abs(t3.point.y - t_mid.point.y))
    good = d_mid < 1e-6
    print("  {} 融合点 == 中位帧点（差 {:.2e} m）".format("✓" if good else "✗", d_mid))
    ok &= good
    # 判据 3：融合点 ≠ 第一帧的点（证明不是"拿一帧就跑"）
    d_lo = abs(t3.point.y - t_lo.point.y)
    good = d_lo > 1e-4
    print("  {} 融合点 ≠ 第一帧点（y 差 {:.1f} mm，证明真的在融合）".format(
        "✓" if good else "✗", d_lo * 1000))
    ok &= good
    # 判据 4：日志里有融合行 + 代表帧 = 第 2 帧（du=+8 那帧）
    fused = [ln for ln in lg3.lines if "融合[" in ln and "中位数" in ln]
    good = bool(fused) and "3 帧中位数" in fused[0]
    print("  {} 融合日志: {}".format("✓" if good else "✗",
                                    (fused[0][:76] + "…") if fused and len(fused[0]) > 76
                                    else (fused[0] if fused else "（没有）")))
    ok &= good
    rep = any("代表帧 3" in ln for ln in lg3.lines)
    print("  {} 代表帧 = 第 3 帧（du=+8 中位帧；编号含 f0：f0=1,重跑=2,投票帧=3,4）".format(
        "✓" if rep else "✗"))
    ok &= rep
    # 判据 5：三帧样本都进了融合（哪一帧被核对剔掉会变 2 帧）
    good = fused and "3 帧" in fused[0]
    print("  {} 三帧样本全部参与（尺寸/高度核对没有误剔）".format("✓" if good else "✗"))
    ok &= good
    # 判据 6：没有异常（回归检查：09-18 现场的 TypeError 就死在这条路径上）
    errs = [ln for ln in lg3.lines if ln.startswith("[error]")]
    good = not errs
    print("  {} 全程无 error 日志{}".format("✓" if good else "✗",
                                          "" if good else "：" + errs[0][:60]))
    ok &= good

    print()
    print("结论: {}".format("全部通过 ✓" if ok else "存在失败项 ✗"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
