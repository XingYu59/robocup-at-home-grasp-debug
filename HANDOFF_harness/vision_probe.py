#!/usr/bin/env python3
"""视觉探针：调一次 /detect_grasp_target，把返回的目标按置信度打出来。

用途：改完视觉侧（词表/复核/投票/尺寸核对）后，**不跑整条抓取链**就能看标签准不准。
只用我们自己的服务，不读任何 gz 真值 ✓

用法：
    source /opt/ros/humble/setup.bash && source ~/Robocup@home_ws/install/setup.bash
    python3 src/HANDOFF_harness/vision_probe.py                # 全类别（抓取模式）
    python3 src/HANDOFF_harness/vision_probe.py "coke can"     # 只找指定类别
"""

import sys
import time

import rclpy
from rclpy.node import Node

from turtlebot3_manipulation_grasp.srv import DetectGraspTarget


def main():
    classes = [c.strip() for c in (sys.argv[1].split(",") if len(sys.argv) > 1 else []) if c.strip()]
    rclpy.init()
    node = Node("vision_probe")
    cli = node.create_client(DetectGraspTarget, "/detect_grasp_target")
    if not cli.wait_for_service(timeout_sec=20.0):
        print("!! 服务 /detect_grasp_target 不可用", flush=True)
        return 1

    req = DetectGraspTarget.Request()
    req.class_ids = classes
    t0 = time.time()
    fut = cli.call_async(req)
    # ★ 必须 spin：rclpy 的 future 要靠执行器回调才会 done，光 sleep 轮询永远等不到 ✗
    rclpy.spin_until_future_complete(node, fut, timeout_sec=150.0)
    if not fut.done():
        print("!! 超时（>150s）", flush=True)
        return 1
    res = fut.result()
    dt = time.time() - t0
    node.destroy_node()
    rclpy.shutdown()
    if res is None:
        print("!! 无响应", flush=True)
        return 1
    print("success={} ({:.1f}s) {}".format(res.success, dt, res.message))
    for t in sorted(res.targets, key=lambda x: -x.confidence):
        print("   {:>18s}  conf={:.2f}  轴心点({:+.3f}, {:+.3f}, {:+.3f})  帧={}".format(
            t.class_id, t.confidence, t.point.x, t.point.y, t.point.z, t.header.frame_id))
    print("共 {} 个目标".format(len(res.targets)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
