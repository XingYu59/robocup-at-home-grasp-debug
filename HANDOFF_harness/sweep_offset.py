#!/usr/bin/env python3
"""径向偏置扫描 —— 直接量出"契约点沿接近方向到底偏了多少"。

为什么需要它（现场事实，不用再论证）：
    ★ 落点核对 = 0 mm  ⇒ 机械臂【精确】停在契约点上（不是没到位 ✗）
    ★ 姿态核对        ⇒ 顶抓姿态正常（接近轴竖直、合拢轴水平）
    但两只物体**都空合**：糖盒窄边 38 mm、手指却合到 29.0 mm（= 指令值）；
                            罐子窄边 66 mm、手指却合到 57.0 mm（= 指令值）。

    几何推论：只要契约点【沿接近方向偏深 ≥ ~28 mm】，指腹就整体落到物体【后面】
    ⇒ 空合；从相机看只是"差一点点"。所以瓶颈不是姿态、也不是落点精度，
    而是**契约点在接近方向上的真实偏差** —— 这个量调参数调不出来，只能测 ✗

    本脚本就做这件事：把契约点沿接近方向平移一组偏置，逐个真抓一次，
    看哪一档手指【提前被挡住】（= 那里真有物体）⇒ 那个偏置就是真实偏差 ✓
    全程只用我们自己的服务 + TF/关节反馈，不读任何 gz 真值 ✓

判据（两个都写进日志，便于复核）：
    · 接触 ✓ ：最小两指间距 ≈ 物体窄边 ± 4 mm（手指被物体挡住，停不到指令值）
    · 空合 ✗ ：最小两指间距 ≈ 指令值（= 窄边 − 2×干涉量；两指之间没有实体）

★ 安全顺序（必须遵守）：
    偏置从【欠伸一侧】开始扫（负偏置 = 契约点朝机器人方向回退 = 停在物体【前面】），
    一旦某档判为接触就【立即停止】并输出结论 ——
    绝不在物体后面反复空合（那是把罐子顶翻的标准姿势 ✗）。
    因此脚本默认把 --offsets 升序排序后再扫，且只有"明确空合"才继续往深里走。

关于正负号（说清楚，免得读反）：
    偏置是加在**契约点**上的修正量，沿"机器人 → 目标点"的水平方向为正。
    ⇒ 偏置为负 = 契约点比现在更靠近机器人（更浅、更靠外）
    ⇒ 偏置为正 = 契约点更深（更靠里、越过物体）
    结论里的"真实径向偏差"就是**第一次接触的那个偏置**：负 = 契约点偏深，要回退。

用法（仿真 + 导航到站位 + 抓取服务都已起来）：
    source /opt/ros/humble/setup.bash && source ~/Robocup@home_ws/install/setup.bash
    python3 src/HANDOFF_harness/sweep_offset.py --class tomato_soup_can
    python3 src/HANDOFF_harness/sweep_offset.py --class tomato_soup_can --point 0.38 0.0
    python3 src/HANDOFF_harness/sweep_offset.py --class sugar_box --axis y
    python3 src/HANDOFF_harness/sweep_offset.py --self-test      # 不连 ROS，验判据
    python3 src/HANDOFF_harness/sweep_offset.py --point 0.38 0 --dry-run   # 只看计划
输出：终端表格 + HANDOFF_harness/.run/sweep_offset_<时间戳>.log（整段贴回即可）
"""

import argparse
import datetime
import math
import os
import sys
import threading
import time

# ══════════════════ 常量 / 路径（纯 Python 部分） ══════════════════

HERE = os.path.dirname(os.path.realpath(__file__))
SRC_ROOT = os.path.abspath(os.path.join(HERE, os.pardir))
OBJECTS_YAML = os.path.join(SRC_ROOT, "turtlebot3_manipulation_grasp", "config", "objects.yaml")
GRASP_PARAMS_YAML = os.path.join(SRC_ROOT, "turtlebot3_manipulation_grasp", "config",
                                 "grasp_params.yaml")
LOG_DIR = os.path.join(HERE, ".run")        # ★ 写工作区内：沙箱不让写 ~/（capture_grasp.py 踩过）

GRASP_SERVICE = "/grasp_fixed_object"
VISION_SERVICE = "/detect_grasp_target"
BASE_FRAME = "base_footprint"

TABLE_TOP_Z_DEFAULT = 0.780     # 与 sweep_target.py 同源：支撑面高度（--point 只给两维时用它）
CONTACT_TOL_MM = 4.0            # 判据容差：≈ 窄边 ± 4 mm ⇒ 接触
SQUEEZE_DEFAULT_M = 0.0045      # grasp_params.yaml: close_hand_squeeze（读不到时的兜底）
CLOSED_MARGIN_MM = 5.0          # 最小间距比"起始张开"小这么多才算"手指真的合过"
SAMPLE_DT = 0.02                # 采样周期：要 50 Hz 才抓得到合爪的最低点
DEFAULT_OFFSETS = [-0.04, -0.03, -0.02, -0.01, 0.0, 0.01, 0.02, 0.03, 0.04]


# ══════════════════ 纯 Python：表读取 + 判据（可单测） ══════════════════

def read_object_sizes(path):
    """读 objects.yaml 的三维尺寸 → {class_id: (depth, width, height)}；读不到返回 {}。"""
    try:
        import yaml
        with open(path, encoding="utf-8") as fh:
            doc = yaml.safe_load(fh)
        out = {}
        for name, o in (doc.get("objects") or {}).items():
            out[name] = (float(o["depth"]), float(o["width"]), float(o["height"]))
        return out
    except Exception:                                     # noqa: BLE001
        return {}


def narrow_edge_mm(size):
    """物体【窄边】(mm) = min(depth, width) —— 合爪方向能跨过去的那一对边。"""
    return min(size[0], size[1]) * 1000.0


def read_close_squeeze(path, default=SQUEEZE_DEFAULT_M):
    """读 grasp_params.yaml: close_hand_squeeze（每根手指多压进去多少 m）。

    ★ 用它才能算出"指令值"：两指间距指令 = 窄边 − 2×squeeze。
      实测核对：罐子 66 − 2×4.5 = 57.0 mm ✓、糖盒 38 − 9 = 29.0 mm ✓ 与现场一致。
    """
    try:
        import yaml
        with open(path, encoding="utf-8") as fh:
            doc = yaml.safe_load(fh)
        stack = [doc]
        while stack:
            cur = stack.pop()
            if isinstance(cur, dict):
                if "close_hand_squeeze" in cur:
                    return float(cur["close_hand_squeeze"])
                stack.extend(cur.values())
    except Exception:                                     # noqa: BLE001
        pass
    return default


def criterion_windows(span_mm, cmd_mm, tol_mm=CONTACT_TOL_MM):
    """返回 (接触窗, 空合窗)，用来判断两个判据有没有重叠。

    重叠 = 物体窄边与指令值只差不到 2×tol ⇒ 这套判据分不开（必须报出来，不能瞎猜 ✗）。
    """
    return ((span_mm - tol_mm, span_mm + tol_mm), (cmd_mm - tol_mm, cmd_mm + tol_mm))


def classify_gap(gap_mm, span_mm, cmd_mm, tol_mm=CONTACT_TOL_MM):
    """按现场判据给一次合爪定性 → ("contact" | "air" | "unknown", 一句话说明)。

    contact ✓ ：最小间距 ≈ 物体窄边 ± tol  → 手指被实体挡住，停不到指令值
    air     ✗ ：最小间距 ≈ 指令值   ± tol  → 两指之间什么都没有
    unknown   ：两个都不沾（或两个都沾 = 判据重叠），不猜，交给调用方安全处理
    """
    if gap_mm is None:
        return "unknown", "没测到两指间距（TF 未就绪？）"
    d_contact = abs(gap_mm - span_mm)
    d_air = abs(gap_mm - cmd_mm)
    if d_contact <= tol_mm and d_air <= tol_mm:
        return "unknown", ("判据重叠：窄边 {:.1f} 与指令值 {:.1f} 只差 {:.1f} mm，"
                           "容差 ±{:.0f} mm 分不开 ⇒ 换更薄的判据或换物体"
                           .format(span_mm, cmd_mm, abs(span_mm - cmd_mm), tol_mm))
    if d_contact <= tol_mm:
        return "contact", ("最小间距 {:.1f} mm ≈ 窄边 {:.1f} mm（±{:.0f}）→ 手指被实体挡住 ✓"
                           .format(gap_mm, span_mm, tol_mm))
    if d_air <= tol_mm:
        return "air", ("最小间距 {:.1f} mm ≈ 指令值 {:.1f} mm（±{:.0f}）→ 两指之间没有实体 ✗"
                       .format(gap_mm, cmd_mm, tol_mm))
    return "unknown", ("最小间距 {:.1f} mm 既不在窄边 {:.1f}±{:.0f} 也不在指令值 {:.1f}±{:.0f}"
                       .format(gap_mm, span_mm, tol_mm, cmd_mm, tol_mm))


# ══════════════════ ROS 部分（没 source 也能 --help / --self-test / --dry-run） ══════════════════

_ROS_ERR = None
try:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import JointState
    import tf2_ros

    from turtlebot3_manipulation_grasp.msg import GraspTargetStamped
    from turtlebot3_manipulation_grasp.srv import DetectGraspTarget, GraspFixedObject
except Exception as _exc:                                 # noqa: BLE001
    # ★ 不在这里抛：否则 --help / --self-test 也会被 ImportError 糊一脸 traceback ✗
    #   真正的报错留到 main() 里、参数解析之后，给一句能照做的话 ✓
    _ROS_ERR = _exc

if _ROS_ERR is None:

    def quat_apply(qx, qy, qz, qw, v):
        """单位四元数旋转向量：v' = v + 2·qw·(q×v) + 2·q×(q×v)。"""
        cx = qy * v[2] - qz * v[1]
        cy = qz * v[0] - qx * v[2]
        cz = qx * v[1] - qy * v[0]
        tx = qy * cz - qz * cy
        ty = qz * cx - qx * cz
        tz = qx * cy - qy * cx
        return (v[0] + 2.0 * (qw * cx + tx),
                v[1] + 2.0 * (qw * cy + ty),
                v[2] + 2.0 * (qw * cz + tz))

    class OffsetSweeper(Node):
        """只有它碰 ROS。复刻工程里现成做法：finger_gap_mm / tcp_pose_base / 抓取服务。"""

        def __init__(self, timeout=120.0):
            # ★ use_sim_time 必须与抓取节点一致：否则时间戳差 1.79e9 s 被"新鲜度"校验拒收 ✗
            #   （sweep_target.py 实测：目标时间戳比本节点时钟超前 1789386791 s）
            super().__init__("sweep_offset",
                             parameter_overrides=[Parameter("use_sim_time", value=True)])
            self.timeout = timeout
            self.cli = self.create_client(GraspFixedObject, GRASP_SERVICE)
            self.vis = self.create_client(DetectGraspTarget, VISION_SERVICE)
            self.create_subscription(JointState, "/joint_states", self._on_js, 10)
            self.tf = tf2_ros.Buffer()
            self.listener = tf2_ros.TransformListener(self.tf, self)
            self._finger_q = None
            self._gmin = None
            self._tcp_at_min = None

        def _on_js(self, msg):
            for name, pos in zip(msg.name, msg.position):
                if name == "fr3_finger_joint1":
                    self._finger_q = float(pos)
                    break

        def finger_gap_mm(self):
            """两指【真实间距】(mm)：量 fr3_leftfinger ↔ fr3_rightfinger 两个帧的距离。

            ★ 不能只看 /joint_states：URDF 里 fr3_finger_joint2 是 <mimic>，
              而 Fortress(gz-sim 6) 不支持 mimic ⇒ 仿真里只有一根手指有驱动 ✗
              量两个手指帧的距离才是真实间隙 ✓（TF 是机器人自己的运动学，不是真值）
            """
            try:
                tr = self.tf.lookup_transform("fr3_leftfinger", "fr3_rightfinger",
                                              rclpy.time.Time())
                t = tr.transform.translation
                return math.sqrt(t.x * t.x + t.y * t.y + t.z * t.z) * 1000.0
            except Exception:                             # noqa: BLE001
                return None

        def tcp_pose_base(self):
            """指尖平面 fr3_hand_tcp 在 base_footprint 里的 (x, y, z)；取不到返回 None。"""
            try:
                tr = self.tf.lookup_transform(BASE_FRAME, "fr3_hand_tcp", rclpy.time.Time())
                t = tr.transform.translation
                return (t.x, t.y, t.z)
            except Exception:                             # noqa: BLE001
                return None

        def to_base(self, point, frame_id):
            """把目标点从其所在帧变换到 base_footprint（用最新 TF，避免外推报错）。"""
            fid = frame_id or BASE_FRAME
            p = (float(point.x), float(point.y), float(point.z))
            if fid == BASE_FRAME:
                return p
            tr = self.tf.lookup_transform(BASE_FRAME, fid, rclpy.time.Time())
            t = tr.transform.translation
            q = tr.transform.rotation
            r = quat_apply(q.x, q.y, q.z, q.w, p)
            return (r[0] + t.x, r[1] + t.y, r[2] + t.z)

        def detect_point(self, cls, timeout=30.0):
            """调一次 /detect_grasp_target 拿目标点 → (px, py, pz, 说明) 或 (None,)*4。

            先按 --class 过滤；视觉词表里没有这个类别（如 tomato_soup_can）时退回不过滤，
            再从中挑同名类别（没有就取置信度最高的那个）。
            """
            for ids in ([cls], []):
                req = DetectGraspTarget.Request()
                req.class_ids = ids
                fut = self.vis.call_async(req)
                t0 = time.time()
                while rclpy.ok() and not fut.done() and time.time() - t0 < timeout:
                    time.sleep(0.05)
                res = fut.result() if fut.done() else None
                if res is None or not res.success or not res.targets:
                    continue
                tg = list(res.targets)
                same = [t for t in tg if t.class_id == cls]
                best = (same or sorted(tg, key=lambda t: -t.confidence))[0]
                try:
                    p = self.to_base(best.point, best.header.frame_id)
                except Exception as e:                    # noqa: BLE001
                    return None, None, None, "目标点 {} → base 变换失败: {}: {}".format(
                        best.header.frame_id, type(e).__name__, e)
                return p[0], p[1], p[2], "detect: class={} conf={:.2f} frame={}".format(
                    best.class_id, best.confidence, best.header.frame_id)
            return None, None, None, "视觉没返回可用目标（服务没起来 / 桌上没有物体）"

        def try_target(self, cls, x, y, z):
            """调一次抓取服务，边等边采两指间距 → (res, 最小间距, 最小间距时的指尖, 错误串)。

            ★ 采样要在服务【执行期间】做：抓取服务返回时手指可能已经松开/抬起，
              只看返回值量不到"合爪过程中到底停在哪" ✗
            """
            req = GraspFixedObject.Request()
            t = GraspTargetStamped()
            t.header.frame_id = BASE_FRAME
            t.header.stamp = self.get_clock().now().to_msg()
            t.class_id = cls
            t.confidence = 1.0
            t.point.x, t.point.y, t.point.z = float(x), float(y), float(z)
            req.target = t
            self._gmin = None
            self._tcp_at_min = None
            fut = self.cli.call_async(req)
            t0 = time.time()
            while rclpy.ok() and not fut.done() and time.time() - t0 < self.timeout:
                g = self.finger_gap_mm()
                if g is not None and (self._gmin is None or g < self._gmin):
                    self._gmin = g
                    self._tcp_at_min = self.tcp_pose_base()
                time.sleep(SAMPLE_DT)
            if not fut.done():
                # 不 cancel：机械臂还在动，cancel 后再发一条会撞上 STAGE_BUSY ✗
                return None, self._gmin, self._tcp_at_min, "服务超时(>{:.0f}s)".format(self.timeout)
            res = fut.result()
            if res is None:
                return None, self._gmin, self._tcp_at_min, "服务无响应"
            return res, self._gmin, self._tcp_at_min, ""


# ══════════════════ 纯 Python：几何小工具 ══════════════════

def unit_axis(args_axis, px, py):
    """接近方向的单位向量（水平面内）→ (ux, uy) 或 (None, None) + 错误串。"""
    if args_axis == "x":
        return 1.0, 0.0, ""
    if args_axis == "y":
        return 0.0, 1.0, ""
    n = math.hypot(px, py)
    if n < 1e-6:
        return None, None, "目标点几乎在底盘正上方（{:.3f},{:+.3f}）→ 没有可用的" \
                           "机器人→目标 方向；请用 --axis x 或 --axis y".format(px, py)
    return px / n, py / n, ""


def offset_point(px, py, pz, ux, uy, off):
    """契约点沿接近方向平移 off（正 = 更里 / 远离机器人）。"""
    return px + ux * off, py + uy * off, pz


# ══════════════════ 自测：判据函数（不连 ROS） ══════════════════

def self_test():
    """用现场实测值当场验判据：罐子(66/57)、糖盒(38/29) —— 接触 vs 空合必须分得开。"""
    cases = [
        # (间距mm, 窄边mm, 指令mm, 期望)
        (57.0, 66.0, 57.0, "air"),        # 现场罐子空合实测值 = 指令值
        (66.0, 66.0, 57.0, "contact"),    # 正好夹到窄边
        (64.0, 66.0, 57.0, "contact"),    # 窄边 − 2 mm（稍许挤压）
        (62.0, 66.0, 57.0, "contact"),    # 窄边 − 4 mm（容差边界里侧）
        (29.0, 38.0, 29.0, "air"),        # 现场糖盒空合实测值 = 指令值
        (38.0, 38.0, 29.0, "contact"),    # 糖盒正好夹到窄边
        (34.5, 38.0, 29.0, "contact"),    # 夹住后被挤进去 3.5 mm
        (80.0, 66.0, 57.0, "unknown"),    # 手指根本没合（服务没执行）
        (None, 66.0, 57.0, "unknown"),    # 没测到
        (61.5, 66.0, 57.0, "unknown"),    # 落在两个窗之间 → 不猜
    ]
    bad = 0
    for gap, span, cmd, want in cases:
        got, why = classify_gap(gap, span, cmd)
        ok = (got == want)
        bad += 0 if ok else 1
        print("  {} gap={} span={} cmd={} → {:<8}（期望 {:<8}）{}".format(
            "✓" if ok else "✗", gap, span, cmd, got, want, why))
    # 判据重叠必须被识别出来（窄边 30、指令 26 → 窗 [26,34] 与 [22,30] 重叠）
    got, why = classify_gap(28.0, 30.0, 26.0)
    ok = (got == "unknown")
    bad += 0 if ok else 1
    print("  {} 重叠判据 (span=30 cmd=26 gap=28) → {}（期望 unknown）{}".format(
        "✓" if ok else "✗", got, why))
    # 表读取必须能读到现场这两个数
    sizes = read_object_sizes(OBJECTS_YAML)
    sq = read_close_squeeze(GRASP_PARAMS_YAML)
    for cls, want_span in (("tomato_soup_can", 66.0), ("sugar_box", 38.0)):
        if cls not in sizes:
            bad += 1
            print("  ✗ objects.yaml 里读不到 {}".format(cls))
            continue
        span = narrow_edge_mm(sizes[cls])
        cmd = span - 2.0 * sq * 1000.0
        ok = abs(span - want_span) < 1e-6
        bad += 0 if ok else 1
        print("  {} {} 窄边 = {:.1f} mm，squeeze = {:.4f} → 指令值 = {:.1f} mm{}".format(
            "✓" if ok else "✗", cls, span, sq, cmd, "" if ok else "（期望 {:.1f}）".format(want_span)))
    print("自测：{}".format("全部通过 ✓" if bad == 0 else "有 {} 项不符 ✗".format(bad)))
    return 0 if bad == 0 else 1


# ══════════════════ 主流程 ══════════════════

def build_parser():
    ap = argparse.ArgumentParser(
        description="径向偏置扫描：量出契约点沿接近方向的真实偏差（从欠伸侧扫起，一接触就停）",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--class", dest="cls", default="tomato_soup_can",
                    help="目标类别（默认 tomato_soup_can）；窄边从 objects.yaml 的 min(d,w) 读")
    ap.add_argument("--offsets", type=float, nargs="*", default=list(DEFAULT_OFFSETS),
                    help="沿接近方向的偏置列表（m；负 = 更靠外/更浅，正 = 更靠里/更深）。"
                         "默认 {}。脚本会【升序】扫（= 从欠伸侧开始），一接触就停"
                         .format(" ".join("{:+.2f}".format(o) for o in DEFAULT_OFFSETS)))
    ap.add_argument("--axis", choices=("view", "x", "y"), default="view",
                    help="接近方向：view = 机器人→目标点的水平方向（默认）；x/y = base 的 +x/+y")
    ap.add_argument("--point", type=float, nargs="+", default=None, metavar="X",
                    help="直接用这个 base 坐标（x y [z]），跳过 /detect_grasp_target")
    ap.add_argument("--z", type=float, default=TABLE_TOP_Z_DEFAULT,
                    help="--point 只给两维时的支撑面高度（默认 {:.3f}）".format(TABLE_TOP_Z_DEFAULT))
    ap.add_argument("--tol", type=float, default=CONTACT_TOL_MM,
                    help="判据容差 mm（默认 {:.1f}）：≈ 窄边 ⇒ 接触；≈ 指令值 ⇒ 空合"
                         .format(CONTACT_TOL_MM))
    ap.add_argument("--tries", type=int, default=2,
                    help="同一偏置最多试几次（默认 2；只在「手指根本没合」时重试，"
                         "MTC 规划本身不稳定）")
    ap.add_argument("--timeout", type=float, default=120.0, help="单次抓取服务超时 s（默认 120）")
    ap.add_argument("--dry-run", action="store_true",
                    help="只打印将要用的偏置/目标点/判据，不调任何服务（需配合 --point）")
    ap.add_argument("--self-test", action="store_true", help="只跑判据与读表的自测，不连 ROS")
    return ap


def print_plan(log, offsets, ux, uy, px, py, pz, span, cmd, tol):
    log("偏置扫描计划：")
    log("  接近方向单位向量 = ({:+.4f}, {:+.4f})  ← 机器人(0,0) → 目标点".format(ux, uy))
    log("  基准契约点 = base({:+.4f}, {:+.4f}, {:.4f})".format(px, py, pz))
    w_contact = criterion_windows(span, cmd, tol)
    log("  判据：最小两指间距 ∈ [{:.1f}, {:.1f}] mm ⇒ 接触 ✓ ；∈ [{:.1f}, {:.1f}] mm ⇒ 空合 ✗"
        .format(w_contact[0][0], w_contact[0][1], w_contact[1][0], w_contact[1][1]))
    log("  扫法：升序（负偏置 = 停在物体【前面】）→ 一旦接触立即停 ✗不再往深里走")
    log("  {:>9} | {:>20} | {}".format("偏置mm", "目标 base(x, y)", "含义"))
    log("  " + "-" * 62)
    for off in offsets:
        qx, qy, _ = offset_point(px, py, pz, ux, uy, off)
        tag = "欠伸侧（物体前面）" if off < 0 else ("契约点原样" if off == 0 else "更深（越过物体）")
        log("  {:+9.1f} | ({:+.4f}, {:+.4f}) | {}".format(off * 1000.0, qx, qy, tag))
    log("  " + "-" * 62)


def main():
    args = build_parser().parse_args()

    if args.self_test:
        return self_test()

    # ── 从 objects.yaml / grasp_params.yaml 算"窄边"和"指令值" ──
    sizes = read_object_sizes(OBJECTS_YAML)
    if args.cls not in sizes:
        print("!! objects.yaml 里没有类别 \"{}\"（现有：{}）".format(
            args.cls, "、".join(sorted(sizes)) or "读不到表，检查路径 " + OBJECTS_YAML))
        return 2
    size = sizes[args.cls]
    span = narrow_edge_mm(size)
    squeeze = read_close_squeeze(GRASP_PARAMS_YAML)
    cmd = span - 2.0 * squeeze * 1000.0
    lo_w, hi_w = criterion_windows(span, cmd, args.tol)
    overlapped = not (hi_w[0] < lo_w[1] or lo_w[0] < hi_w[1])

    offsets = sorted(args.offsets)          # ★ 升序 = 从欠伸侧开始（安全顺序，见文件头）

    # ── 纯 Python 干跑：不连 ROS ──
    if args.dry_run:
        if not args.point:
            print("!! --dry-run 需要同时给 --point x y（否则要用视觉服务取点）")
            return 2
        px, py = args.point[0], args.point[1]
        pz = args.point[2] if len(args.point) >= 3 else args.z
        ux, uy, err = unit_axis(args.axis, px, py)
        if err:
            print("!! " + err)
            return 2
        print("[dry-run] {} 窄边 = {:.1f} mm，squeeze = {:.4f} → 指令值 = {:.1f} mm".format(
            args.cls, span, squeeze, cmd))
        print_plan(print, offsets, ux, uy, px, py, pz, span, cmd, args.tol)
        print("[dry-run] 一切正常；去掉 --dry-run 就会真的逐个调用 " + GRASP_SERVICE)
        return 0

    if _ROS_ERR is not None:
        print("!! 起不来 ROS 环境：{}: {}".format(type(_ROS_ERR).__name__, _ROS_ERR))
        print("   先 source 一下（当前 shell 里 rclpy 或本工程的消息包找不到）：")
        print("     source /opt/ros/humble/setup.bash && source ~/Robocup@home_ws/install/setup.bash")
        print("   （只跑 --self-test 不需要 source）")
        return 2

    # ── 日志：从这一刻起，参数/判据/表格/结论全部落盘 ──
    os.makedirs(LOG_DIR, exist_ok=True)
    log_path = os.path.join(LOG_DIR, "sweep_offset_{}.log".format(
        datetime.datetime.now().strftime("%Y%m%d_%H%M%S")))
    fh = open(log_path, "w", encoding="utf-8")

    def log(line=""):
        text = str(line)
        print(text, flush=True)
        fh.write(text + "\n")
        fh.flush()

    log("=" * 88)
    log("径向偏置扫描  {}".format(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")))
    log("参数: class={} axis={} offsets={} tol={:.1f}mm tries={} point={}".format(
        args.cls, args.axis, ["{:+.3f}".format(o) for o in offsets], args.tol, args.tries,
        args.point))
    log("物体: {} depth={:.4f} width={:.4f} height={:.4f}".format(args.cls, *size))
    log("窄边 = min(d,w) = {:.1f} mm；squeeze = {:.4f} m → 合爪指令值 = {:.1f} mm".format(
        span, squeeze, cmd))
    log("判据: 最小间距 ∈ [{:.1f}, {:.1f}] ⇒ 接触 ✓ ；∈ [{:.1f}, {:.1f}] ⇒ 空合 ✗"
        .format(lo_w[0], lo_w[1], hi_w[0], hi_w[1]))
    if overlapped:
        log("!! 两个判据窗重叠（窄边与指令值只差 {:.1f} mm ≤ 2×容差）→ 本物体靠这套判据分不开，"
            "结论不可信 ✗".format(abs(span - cmd)))
    log("=" * 88)

    try:
        rclpy.init()
    except Exception as e:                                # noqa: BLE001
        # ★ 不让 rclpy 自己的 traceback 糊脸（最常见：ROS_HOME/日志目录不可写，
        #   报 "Failed opening file ~/.ros/log/….log for writing"）→ 说清怎么办 ✓
        log("!! rclpy.init() 失败：{}: {}".format(type(e).__name__, e))
        log("   八成是 ROS 日志目录不可写。两个办法：")
        log("     ① 直接跑（普通终端下 ~/.ros 一般可写）；")
        log("     ② 换个可写目录：export ROS_LOG_DIR=$PWD/HANDOFF_harness/.run/roslog")
        return 2
    node = OffsetSweeper(timeout=args.timeout)
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    spin_thread = threading.Thread(target=ex.spin, daemon=True)
    spin_thread.start()

    rc = 0
    try:
        if not node.cli.wait_for_service(timeout_sec=20.0):
            log("!! {} 不可用：抓取服务没起来（先起 grasp_service.launch.py），或本终端没 source "
                "install/setup.bash".format(GRASP_SERVICE))
            return 1
        time.sleep(1.0)
        open_gap = node.finger_gap_mm()
        log("起始两指间距 = {} mm".format("-" if open_gap is None else "{:.1f}".format(open_gap)))
        if open_gap is None:
            log("!! 量不到两指间距（TF fr3_leftfinger→fr3_rightfinger 不可用）→ 无法判接触/空合 ✗")
            return 1

        # ── 取基准契约点 ──
        if args.point:
            px, py = args.point[0], args.point[1]
            pz = args.point[2] if len(args.point) >= 3 else args.z
            log("基准契约点 = 命令行给的 base({:+.4f}, {:+.4f}, {:.4f})".format(px, py, pz))
        else:
            log("调 {} 取目标点…".format(VISION_SERVICE))
            px, py, pz, msg = node.detect_point(args.cls)
            log("  " + msg)
            if px is None:
                log("!! 取不到目标点。两条路：① 起视觉适配层；② 直接给坐标 "
                    "--point x y（base_footprint 系）")
                return 1

        ux, uy, err = unit_axis(args.axis, px, py)
        if err:
            log("!! " + err)
            return 1

        log("")
        print_plan(log, offsets, ux, uy, px, py, pz, span, cmd, args.tol)
        log("")

        head = ("{:>9} | {:>16} | {:>9} | {:<8} | {:>9} | {:>9} | {}").format(
            "偏置mm", "目标base(x,y)", "最小间距", "判据", "停短mm", "落点mm", "服务返回")
        log(head)
        log("-" * 118)

        rows = []
        first_contact = None
        aborted = ""
        for off in offsets:
            qx, qy, qz = offset_point(px, py, pz, ux, uy, off)
            res = gmin = tcp = None
            err = ""
            for attempt in range(1, max(1, args.tries) + 1):
                res, gmin, tcp, err = node.try_target(args.cls, qx, qy, qz)
                if err:
                    break
                closed = gmin is not None and gmin < open_gap - CLOSED_MARGIN_MM
                if closed or attempt >= max(1, args.tries):
                    break
                log("  （偏置 {:+.1f} mm 第 {} 次没合爪 → 重试；MTC 规划本身不稳定）"
                    .format(off * 1000.0, attempt))
            if err:
                log("{:+9.1f} | ({:+.3f},{:+.3f}) | {:>9} | {:<8} | {:>9} | {:>9} | {}".format(
                    off * 1000.0, qx, qy, "-", "中断", "-", "-", err))
                aborted = err
                rc = 1
                break

            closed = gmin is not None and gmin < open_gap - CLOSED_MARGIN_MM
            if not closed:
                verdict, why = "no_close", "手指根本没合（服务没执行/解不出）→ 本档没有信息"
            else:
                verdict, why = classify_gap(gmin, span, cmd, args.tol)
            stop_short = "-" if gmin is None else "{:+.1f}".format(gmin - cmd)
            land = "-"
            if tcp is not None:
                land = "{:.1f}".format(math.hypot(tcp[0] - qx, tcp[1] - qy) * 1000.0)
            svc = "success={} stage={} {}".format(
                getattr(res, "success", "?"), getattr(res, "stage", -1),
                (getattr(res, "message", "") or "")[:26])
            log("{:+9.1f} | ({:+.3f},{:+.3f}) | {:>9} | {:<8} | {:>9} | {:>9} | {}".format(
                off * 1000.0, qx, qy,
                "-" if gmin is None else "{:.1f}".format(gmin),
                verdict, stop_short, land, svc))
            log("           └ {}".format(why))
            rows.append((off, gmin, verdict, tcp))

            if verdict == "contact":
                first_contact = off
                log("           ★ 判为【接触】→ 立即停止扫描（不再往深里试，避免在物体后面空合碰倒它）")
                break
            if verdict == "air":
                continue                       # 明确空合 → 继续往深里走
            if verdict == "unknown":
                log("           !! 判据既不是接触也不是空合 → 按【安全】处理：停止扫描，"
                    "不再往深里走（往深里走的风险不对称）")
                break
            # no_close：没有信息，继续下一档

        log("-" * 118)
        log("整张表（偏置mm, 最小间距mm, 判据）: {}".format(
            "  ".join("({:+.0f}, {}, {})".format(o * 1000.0,
                                                 "-" if g is None else "{:.1f}".format(g), v)
                      for o, g, v, _ in rows)))
        if first_contact is not None:
            log("")
            log("★ 结论：第一次判为接触的偏置 = {:+.0f} mm".format(first_contact * 1000.0))
            log("★ 真实径向偏差 ≈ {:+.0f} mm（偏里为正）".format(first_contact * 1000.0))
            if first_contact < 0:
                log("   读法：负号 = 现在的契约点比物体的真实位置【偏深】{:.0f} mm；"
                    "要把它沿接近方向【朝机器人回退】{:.0f} mm".format(
                        -first_contact * 1000.0, -first_contact * 1000.0))
                log("   ⇒ 修法方向：把契约点沿接近方向前移量的取法再收紧（像 c0df261 那样），"
                    "或把该类别单独加一个径向修正参数")
            elif first_contact == 0:
                log("   读法：0 mm = 契约点本来就对 —— 那空合的原因就不在径向偏差上，"
                    "回到夹持高度/干涉量/物体尺寸去查 ✗")
            else:
                log("   读法：正号 = 契约点比物体真实位置【偏浅】{:.0f} mm，要往深里推"
                    .format(first_contact * 1000.0))
            if rows and rows[0][0] == first_contact:
                log("   ⚠ 注意：这已经是【最靠外】的那一档（欠伸侧第一档）就接触了 → "
                    "真实偏差比它更浅或物体比表里宽，请把 --offsets 往负方向再扩一档重扫")
        elif aborted:
            log("")
            log("★ 结论：扫描中断（{}），没有拿到结论 ✗ —— 先解决服务/超时，再重跑".format(aborted))
        else:
            log("")
            log("★ 结论：整个扫描里【没有任何一档判为接触】→ 偏移区间 [{:+.0f}, {:+.0f}] mm 内"
                "手指都没碰到东西".format(offsets[0] * 1000.0, offsets[-1] * 1000.0))
            log("   ⇒ 空合的原因不是「沿接近方向偏深」，而是别的："
                "夹持高度不对（指腹落在物体上方/下方）、物体实际比表里窄、"
                "或合爪方向根本不在物体上（横向偏了）—— 下一步该扫横向/高度，不是径向 ✗")
    except KeyboardInterrupt:
        log("!! 被 Ctrl-C 打断")
        rc = 130
    finally:
        log("")
        log("日志: {}".format(log_path))
        fh.close()
        # ★ 退出顺序不能乱：先让 exec 停、join 掉 spin 线程，再销毁节点，最后 rclpy.shutdown。
        #   反着来（destroy_node 时 spin 还在跑）会让 rclcpp abort
        #   —— "terminate called without an active exception" + core dump（实测踩过）✗
        try:
            ex.shutdown()
        except Exception:                                 # noqa: BLE001
            pass
        try:
            spin_thread.join(timeout=3.0)
        except Exception:                                 # noqa: BLE001
            pass
        try:
            ex.remove_node(node)
        except Exception:                                 # noqa: BLE001
            pass
        try:
            node.destroy_node()
        except Exception:                                 # noqa: BLE001
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:                                 # noqa: BLE001
            pass
    return rc


if __name__ == "__main__":
    sys.exit(main())
