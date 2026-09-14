#!/usr/bin/env python3
"""抓取点"前后扫描"标定工具 —— 用机械臂自己找出物体的真实距离。

为什么需要它（2026-09-14 卡住的地方）：
    现场观察与日志对不上 —— 你看到"夹爪覆盖住了盒子、但提不起来"，
    而 TF 量到的指尖高度（0.914 m）比盒子顶面（0.955 m）低 41 mm、按几何应该夹得到 ✗
    推理已经绕圈了，所以改成**直接做实验**：
    把抓取点沿"前后方向"（base_footprint 的 +x）扫一遍，看哪个位置手指会被挡住。
      · 手指合到指令值（不被挡）→ 那个位置上没有物体
      · 手指提前停住（被挡）    → 那里就是物体的真实距离 ✓
    全程只用我们自己的服务 + 关节/TF 反馈，不读任何 gz 真值 ✓

前置：
    机器人已经停在抓取站位（用 dining_grasp_task.py 跑到站位后它会停在那儿，
    也可以开 --skip-nav 让它只做服务调用）。抓取服务已在跑。

用法：
    source /opt/ros/humble/setup.bash && source ~/Robocup@home_ws/install/setup.bash
    python3 src/HANDOFF_harness/sweep_target.py                 # 默认扫 0.26~0.46
    python3 src/HANDOFF_harness/sweep_target.py --xs 0.30 0.35 0.40
    python3 src/HANDOFF_harness/sweep_target.py --class sugar_box --y 0
"""

import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
import tf2_ros

from turtlebot3_manipulation_grasp.msg import GraspTargetStamped
from turtlebot3_manipulation_grasp.srv import GraspFixedObject

TABLE_TOP_Z = 0.780


class Sweeper(Node):
    def __init__(self):
        # ★ use_sim_time 必须与抓取节点一致：否则时间戳差 1.79e9 s 被"新鲜度"校验拒收 ✗
        #   （实测：目标时间戳比本节点时钟超前 1789386791s）
        super().__init__("sweep_target",
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.cli = self.create_client(GraspFixedObject, "/grasp_fixed_object")
        self.sub = self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf, self)
        self.q = None
        self.gap = None
        self.gmin = None

    def _on_js(self, msg):
        for n, p in zip(msg.name, msg.position):
            if n == "fr3_finger_joint1":
                self.q = float(p)

    def read_gap_mm(self):
        try:
            tr = self.tf.lookup_transform("fr3_leftfinger", "fr3_rightfinger",
                                          rclpy.time.Time())
            t = tr.transform.translation
            return math.sqrt(t.x * t.x + t.y * t.y + t.z * t.z) * 1000.0
        except Exception:
            return None

    def tcp_x(self):
        try:
            tr = self.tf.lookup_transform("base_footprint", "fr3_hand_tcp", rclpy.time.Time())
            return tr.transform.translation
        except Exception:
            return None

    def try_target(self, cls, x, y, z=TABLE_TOP_Z):
        req = GraspFixedObject.Request()
        t = GraspTargetStamped()
        t.header.frame_id = "base_footprint"
        t.header.stamp = self.get_clock().now().to_msg()
        t.class_id = cls
        t.confidence = 1.0
        t.point.x, t.point.y, t.point.z = float(x), float(y), float(z)
        req.target = t
        self.gmin = None
        fut = self.cli.call_async(req)
        t0 = time.time()
        while rclpy.ok() and not fut.done() and time.time() - t0 < 120.0:
            g = self.read_gap_mm()
            if g is not None:
                self.gmin = g if self.gmin is None else min(self.gmin, g)
            time.sleep(0.1)
        if not fut.done():
            return None, None, "超时"
        res = fut.result()
        tp = self.tcp_x()
        return res, (None if tp is None else (tp.x, tp.y, tp.z)), ""


def main():
    ap = argparse.ArgumentParser(description="抓取点前后扫描标定")
    ap.add_argument("--class", dest="cls", default="sugar_box", help="目标类别（默认 sugar_box）")
    ap.add_argument("--y", type=float, default=0.0, help="横向坐标（默认 0）")
    ap.add_argument("--xs", type=float, nargs="*",
                    default=[0.22, 0.30, 0.38, 0.46],
                    help="要试的前后距离列表（默认 0.22~0.46）")
    # z 扫的是"支撑面高度"：抓取侧会用它算箱心（箱心 = z + 高度/2），
    # 差值会被 z_tolerance(0.03) 限制，所以可扫范围约 ±30 mm ✓ 用来试"高度是不是偏了"
    ap.add_argument("--zs", type=float, nargs="*", default=[0.780],
                    help="要试的支撑面高度列表（默认 0.780；可试 0.750/0.780/0.810）")
    ap.add_argument("--ys", type=float, nargs="*", default=None,
                    help="要试的横向坐标列表（给了就优先横着扫，前后固定 --xs 的第一个）")
    ap.add_argument("--tries", type=int, default=3,
                    help="每个点最多试几次（MTC 规划/执行本身不稳定，同一目标可能第一次失败）")
    args = ap.parse_args()

    rclpy.init()
    node = Sweeper()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    if not node.cli.wait_for_service(timeout_sec=20.0):
        print("!! /grasp_fixed_object 不可用（终端3 起了吗？）", flush=True)
        return 1
    time.sleep(1.0)
    print("起始两指间距 = {} mm".format(node.read_gap_mm()))
    print("{:>7} {:>7} {:>7} | {:>10} | {}".format("目标x", "横向y", "支撑z", "最小间隙mm", "服务返回"))
    print("-" * 78)
    rows = []
    if args.ys is not None:            # 横扫模式：前后固定用 --xs[0]
        combos = [(args.xs[0], y, args.zs[0]) for y in args.ys]
        label = "横向y"
    else:
        combos = [(x, args.y, z) for z in args.zs for x in args.xs]
        label = "目标x"
    for x, y, z in combos:
        res, gap = None, None
        for attempt in range(1, args.tries + 1):
            res, tcp, err = node.try_target(args.cls, x, y, z)
            gap = node.gmin
            if res is not None and getattr(res, "success", False):
                break
            # 没成功也留着最后一次的间隙（可能是"执行失败但已经合过爪"）
            if res is None:
                break
        if True:
            if res is None:
                print("{:>7.3f} {:>7.3f} {:>7.3f} | {:>10} |".format(x, y, z, "-"))
                continue
            print("{:>7.3f} {:>7.3f} {:>7.3f} | {:>10} | success={} stage={} {}".format(
                x, y, z, "-" if gap is None else "{:.1f}".format(gap),
                res.success, getattr(res, "stage", -1), res.message[:30]))
            rows.append((x, y, z, gap))
    print("-" * 78)
    # "被挡住"的判据：手指确实合拢过（< 60 mm），但停在了物体窄边附近（> 34 mm）
    hits = [(x, y, z, g) for x, y, z, g in rows if g is not None and 34.0 < g < 60.0]
    if hits:
        print("★ 手指被挡住的位置（= 那里确实有实体）：")
        for x, y, z, g in hits:
            print("    x={:.3f} y={:+.3f} z={:.3f} → 最小间隙 {:.1f} mm".format(x, y, z, g))
        print("    → 物体真实位置就在这附近 ✓")
    else:
        moved = [(x, y, z, g) for x, y, z, g in rows if g is not None and g < 60.0]
        if moved:
            print("手指都合到了指令值（{}）→ 这些位置两指之间都没有实体 ✗"
                  .format("、".join("{:.1f}".format(g) for *_, g in moved)))
        else:
            print("!! 每次手指都没动过（一直 80 mm）→ 服务没执行，先看上面的报错 ✗")
    node.destroy_node()
    ex.shutdown()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
