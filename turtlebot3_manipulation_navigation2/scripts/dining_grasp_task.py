#!/usr/bin/env python3
"""
独立抓取入口（薄壳）—— 只为"单独调试抓取"存在。

真正的实现全在 grasp_phase.py（比赛主流程 patrol_task.py 也调同一份，避免两份逻辑漂移）。
本文件只做三件事：
    1. 起节点（use_sim_time 必须与 grasp_node / move_group 一致）
    2. 发布 /initialpose（与 turtlebot3_franka.launch.py 的 spawn 参数一致）
    3. 调 GraspPhase（可命令行覆盖站位、跳过导航/微调）

用法（需先起仿真 + 导航 + grasp_service）:
    ros2 launch turtlebot3_manipulation_gazebo turtlebot3_franka.launch.py
    ros2 launch turtlebot3_manipulation_navigation2 navigation.launch.py
    source ~/venvs/vision_env/bin/activate            # 适配层需要 torch
    ros2 launch turtlebot3_manipulation_grasp grasp_service.launch.py
    ros2 run turtlebot3_manipulation_navigation2 dining_grasp_task.py

调试开关:
    --classes "coke can,apple"   只找这几类
    --park-x/--park-y/--park-yaw 固定站位（不给就由选中的物体位置算出来）
    --skip-nav          机器人已手动停在站位，只测服务链路
    --no-creep          跳过相对闭环微调
    --keep-pose         不重发 /initialpose（仿真已在跑、机器人已在餐桌边时用这个重测）
    --observation "x,y,yaw"  手工指定观察位；默认按餐桌桌沿法线自己算
"""

import argparse
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose

from grasp_phase import (GraspPhase, INITIAL_X, INITIAL_Y, INITIAL_YAW,
                         NAV_ACTION)


class DiningGraspTask(Node):
    def __init__(self, args):
        # use_sim_time 必须与 grasp_node / move_group 一致！仿真里它们跟着 Gazebo 的
        # /clock（仿真钟），驱动若用墙钟，目标时间戳会比抓取侧的 now() 大十几亿秒，
        # 被新鲜度校验直接拒掉（实测报错：age=-1789228743s）。
        super().__init__(
            "dining_grasp_task",
            parameter_overrides=[Parameter("use_sim_time", value=bool(args.use_sim_time))])
        self.args = args
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.nav_client = ActionClient(self, NavigateToPose, NAV_ACTION)
        self.get_logger().info("dining_grasp_task 就绪（实现走 grasp_phase.GraspPhase）")

    # ── 等待 Nav2（轮询，不 spin：本节点跑在多线程 executor 下）────────
    def wait_for_nav2(self, timeout=60.0):
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            if self.nav_client.server_is_ready():
                self.get_logger().info("Nav2 action server 就绪")
                return True
            time.sleep(0.5)
        raise RuntimeError("Nav2 action server 超时")

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = INITIAL_X
        msg.pose.pose.position.y = INITIAL_Y
        from grasp_phase import yaw_to_quat
        _, _, qz, qw = yaw_to_quat(INITIAL_YAW)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        for _ in range(5):
            self.init_pub.publish(msg)
            time.sleep(0.2)
        self.get_logger().info("已发布初始位姿")

    def start(self):
        self.wait_for_nav2()
        if self.args.keep_pose:
            # ★ 不动定位：机器人现在停在哪儿就用哪儿的位姿。
            #   为什么要这个开关：本入口默认会重发 /initialpose（那是给"机器人刚 spawn
            #   在起点"用的）。仿真已经在跑、机器人已经开到餐桌边时重发初始位姿会把
            #   AMCL 的估计一把拽回起点 ✗ → 后面算出来的站位全错。
            self.get_logger().info("--keep-pose：跳过 /initialpose，沿用当前 AMCL 位姿")
        else:
            self.publish_initial_pose()
            time.sleep(5.0)      # 等 AMCL 收敛

        classes = [c.strip() for c in (self.args.classes or "").split(",") if c.strip()]
        phase = GraspPhase(
            self,
            target_classes=classes,
            nav_client=self.nav_client,
        )
        override = None
        if self.args.park_x is not None or self.args.park_y is not None or self.args.park_yaw is not None:
            override = (self.args.park_x if self.args.park_x is not None else 3.0,
                        self.args.park_y if self.args.park_y is not None else 2.51,
                        self.args.park_yaw if self.args.park_yaw is not None else -1.5707963)
        observation = None
        if self.args.observation:
            v = [float(x) for x in self.args.observation.split(",")]
            if len(v) == 3:
                observation = tuple(v)
        return phase.run(observation_pose=observation,
                         skip_nav=self.args.skip_nav, no_creep=self.args.no_creep,
                         park_override=override)


def main():
    parser = argparse.ArgumentParser(description="抓取阶段（独立调试入口）")
    # 注：--truth-only / --no-vision / --vision-only 已删除。
    #     规则书禁止读 gz 真值（即使调试也不行：它会让"视觉到底行不行"这个结论失效），
    #     所以本工程没有任何真值入口，只有视觉一条链。
    parser.add_argument("--classes", default=None,
                        help="只找这几类，逗号分隔，如 \"coke can,apple\"")
    parser.add_argument("--park-x", type=float, default=None, help="固定站位 x（不给就自动算）")
    parser.add_argument("--park-y", type=float, default=None, help="固定站位 y")
    parser.add_argument("--park-yaw", type=float, default=None, help="固定站位 yaw")
    parser.add_argument("--skip-nav", action="store_true", help="跳过导航（机器人已手动停好）")
    parser.add_argument("--no-creep", action="store_true", help="跳过相对闭环微调")
    parser.add_argument("--keep-pose", action="store_true",
                        help="不重发 /initialpose，沿用当前 AMCL 位姿（仿真已在跑时重测用）")
    parser.add_argument("--observation", default="",
                        help="手工指定观察位 \"x,y,yaw\"；默认由抓取侧按餐桌桌沿法线算")
    parser.add_argument("--no-sim-time", dest="use_sim_time", action="store_false",
                        help="用墙钟而不是 Gazebo /clock（无仿真调试时用）")
    parser.set_defaults(use_sim_time=True)
    args, _ = parser.parse_known_args()

    print("=== dining_grasp_task starting ===", flush=True)
    rclpy.init()
    node = DiningGraspTask(args)
    # ★ 多线程 executor：抓取阶段在"轮询等 future"，回调必须在别的线程继续跑
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    box = {"ok": False}

    def _run():
        try:
            box["ok"] = node.start()
        except Exception as e:                       # noqa: BLE001
            import traceback
            node.get_logger().error("抓取阶段异常: {}\n{}".format(e, traceback.format_exc()))
        finally:
            ex.shutdown()

    threading.Thread(target=_run, daemon=True).start()
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print("=== dining_grasp_task 结束: {} ===".format("成功" if box["ok"] else "失败/未完成"))
    sys.exit(0 if box["ok"] else 1)


if __name__ == "__main__":
    main()
