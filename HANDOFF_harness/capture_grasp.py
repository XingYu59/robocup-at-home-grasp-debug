#!/usr/bin/env python3
"""抓取过程相机取证：调一次 /grasp_fixed_object，同时把相机画面按时间存盘。

为什么需要它（2026-09-14 卡住的地方）：
    日志里"手指自由合到 29 mm ⇒ 两指之间没有实体"，但你从 Gazebo 看"夹爪明明覆盖着盒子" ✗
    两者只能有一个对 —— 而机械臂伸出去时**手指会进入相机视野** ✓
    所以把抓取全程的原始画面存下来，肉眼就能看到手指与盒子的真实相对位置 ✓
    不依赖任何帧/外参/模型假设，也不用真值 ✓

用法（仿真+抓取服务已起来、车已停在站位）：
    source /opt/ros/humble/setup.bash && source ~/Robocup@home_ws/install/setup.bash
    python3 src/HANDOFF_harness/capture_grasp.py                 # 默认目标 (0.38, 0, 0.78)
    python3 src/HANDOFF_harness/capture_grasp.py --x 0.32 --y 0
输出：~/turtlebot3_detections/graspcap_XX.png（每 0.3 s 一张）+ 终端打印服务返回与手指间隙。
"""

import argparse
import os
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, JointState
import tf2_ros

from turtlebot3_manipulation_grasp.msg import GraspTargetStamped
from turtlebot3_manipulation_grasp.srv import GraspFixedObject

# ★ 写到【工作区内】：沙箱不让写 ~/ 下的目录，而 cv2.imwrite 失败只警告不抛异常 ✗
#   （实测：终端打印了路径，文件其实没落盘）→ 这里写进本目录的 .run/capture/
OUT_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), ".run", "capture")


class Capture(Node):
    def __init__(self):
        super().__init__("capture_grasp",
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.cli = self.create_client(GraspFixedObject, "/grasp_fixed_object")
        self.create_subscription(Image, "/camera/image_raw", self._on_img, 10)
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf, self)
        self.img = None
        self.lat = None
        self.q = None
        self.gap = None
        self.gmin = None
        self.n = 0

    def _on_img(self, msg):
        self.img = msg
        # ★ 量画面延迟：画面自带的 stamp 与"现在"的差
        #   （如果延迟好几秒，那我看到的"抓取瞬间"其实是机械臂还在 home 时的画面 ✗
        #     会直接误导"手指没进视野"这种判断）
        try:
            now = self.get_clock().now().nanoseconds * 1e-9
            st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if st > 0:
                self.lat = now - st
        except Exception:
            pass

    def _on_js(self, msg):
        for n, p in zip(msg.name, msg.position):
            if n == "fr3_finger_joint1":
                self.q = float(p)

    def gap_mm(self):
        try:
            tr = self.tf.lookup_transform("fr3_leftfinger", "fr3_rightfinger",
                                          rclpy.time.Time())
            t = tr.transform.translation
            return float(np.sqrt(t.x * t.x + t.y * t.y + t.z * t.z)) * 1000.0
        except Exception:
            return None

    def tcp(self):
        try:
            tr = self.tf.lookup_transform("base_footprint", "fr3_hand_tcp", rclpy.time.Time())
            t = tr.transform.translation
            return (t.x, t.y, t.z)
        except Exception:
            return None

    def save(self, tag):
        m = self.img
        if m is None:
            return None
        try:
            import cv2
            h, w = m.height, m.width
            enc = (m.encoding or "").lower()
            buf = np.frombuffer(m.data, dtype=np.uint8)
            if enc in ("rgb8", "bgr8"):
                a = buf.reshape(h, w, 3)
                bgr = a[:, :, ::-1] if enc == "rgb8" else a
            elif enc in ("mono8", "8uc1"):
                bgr = cv2.cvtColor(buf.reshape(h, w), cv2.COLOR_GRAY2BGR)
            else:
                return None
            path = os.path.join(OUT_DIR, "graspcap_{:02d}_{}.png".format(self.n, tag))
            if not cv2.imwrite(path, bgr):        # ★ 检查返回值！失败必须能看见 ✗
                print("!! cv2.imwrite 返回 False（目录不可写？）: {}".format(path), flush=True)
                return None
            self.n += 1
            return path
        except Exception as e:                       # noqa: BLE001
            print("!! 存图失败: {}: {}".format(type(e).__name__, e), flush=True)
            return None


def main():
    ap = argparse.ArgumentParser(description="抓取过程相机取证")
    ap.add_argument("--x", type=float, default=0.38)
    ap.add_argument("--y", type=float, default=0.0)
    ap.add_argument("--z", type=float, default=0.780)
    ap.add_argument("--cls", default="sugar_box")
    args = ap.parse_args()

    os.makedirs(OUT_DIR, exist_ok=True)
    rclpy.init()
    node = Capture()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    if not node.cli.wait_for_service(timeout_sec=20.0):
        print("!! /grasp_fixed_object 不可用", flush=True)
        return 1
    for _ in range(50):                     # 等第一帧画面
        if node.img is not None:
            break
        time.sleep(0.1)
    for _ in range(50):
        if node.lat is not None:
            break
        time.sleep(0.1)
    print("画面延迟 = {} s（>1s 说明画面是旧的 ✗）".format(
        "-" if node.lat is None else "{:.2f}".format(node.lat)), flush=True)
    print("两指初始间距 = {} mm；指尖 = {}".format(node.gap_mm(), node.tcp()), flush=True)
    print("存图目录: {}".format(OUT_DIR), flush=True)
    before = node.save("before")
    print("抓取前画面: {}".format(before), flush=True)

    req = GraspFixedObject.Request()
    t = GraspTargetStamped()
    t.header.frame_id = "base_footprint"
    t.header.stamp = node.get_clock().now().to_msg()
    t.class_id = args.cls
    t.confidence = 1.0
    t.point.x, t.point.y, t.point.z = args.x, args.y, args.z
    req.target = t
    fut = node.cli.call_async(req)
    t0 = time.time()
    last_save = 0.0
    print("目标: {} base({:+.3f},{:+.3f},{:.3f})".format(args.cls, args.x, args.y, args.z),
          flush=True)
    while rclpy.ok() and not fut.done() and time.time() - t0 < 120.0:
        g = node.gap_mm()
        if g is not None:
            node.gmin = g if node.gmin is None else min(node.gmin, g)
        if time.time() - t0 - last_save > 0.3:      # 每 0.3 s 存一张
            last_save = time.time() - t0
            p = node.save("t{:04.1f}".format(last_save))
            if p:
                print("  {:.1f}s  延迟 {:>4} s  间隙 {:>5} mm  指尖 {:>22}  → {}".format(
                    last_save, "-" if node.lat is None else "{:.1f}".format(node.lat),
                    "-" if g is None else "{:.1f}".format(g),
                    "-" if node.tcp() is None else "({:+.3f},{:+.3f},{:.3f})".format(*node.tcp()),
                    os.path.basename(p)), flush=True)
        time.sleep(0.05)
    res = fut.result() if fut.done() else None
    if res is None:
        print("!! 服务无响应/超时", flush=True)
    else:
        print("服务返回: success={} stage={} {}".format(res.success, res.stage, res.message),
              flush=True)
    print("最小两指间距 = {} mm".format(node.gmin), flush=True)
    node.save("after")
    node.destroy_node()
    ex.shutdown()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
