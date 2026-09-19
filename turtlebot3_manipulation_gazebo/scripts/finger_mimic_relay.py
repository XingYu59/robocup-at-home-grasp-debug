#!/usr/bin/env python3
"""`fr3_finger_joint2` 的"仿 mimic"中继（**Fortress 必须用这个**）—— 2026-09-18。

## 为什么需要它（真 bug，不是补丁绕路）

URDF 里第二个手指靠 mimic 联动：

    <joint name="fr3_finger_joint2" type="prismatic">
      ...
      <mimic joint="fr3_finger_joint1"/>     ← 这一行在仿真里**不生效** ✗
    </joint>

**Gazebo Fortress 的 sdformat12 = SDF 1.9，mimic 是 SDF 1.10+ 才有的元素**，实测：

    $ ign sdf -p <生成的 robot.urdf> | grep -c mimic
    0                                        ← 整条被丢掉（转出的 SDF 里根本没有）

于是 `fr3_finger_joint2` 在仿真里是**被动关节**：
  · 原来 `ros2_control` 只声明了 joint1（`gripper_controller.joint = fr3_finger_joint1`）
  · 被动关节停在 URDF 默认值 **0 = 行程下限 = 闭合端**
  · 而 joint1 的 `initial_value = 0.04 = 张开`
⇒ **右指内侧面永远停在夹爪中线附近（y≈0）**，而规划层（URDF 带 mimic）以为它在 y = −q
⇒ 计划中的 80 mm 对称开口，物理上只有 ~40 mm、还偏轴 ~20 mm ✗
⇒ 顶抓时右指正好落在物体**轴线**上，下降被物体顶面挡住、机械臂停在半路：
      tomato_soup_can（顶 0.881 / 计划 TCP 0.868）⇒ 停在比计划高 **22 mm**
      sugar_box      （顶 0.955 / 计划 TCP 0.9195）⇒ 高 **45 mm**
   现场现象就是"夹爪指尖顶在物体顶面上、伸不下去"（用户 2026-09-18 观察）✓

## 这个节点做什么

订阅 `/joint_states`，把 `fr3_finger_joint1` 的**位置**原样发给 joint2 的控制器
（`/finger2_controller/commands`，`forward_command_controller` 的 `Float64MultiArray`）⇒
两指**真正对称**地跟着 joint1 走 ✓（等价于把 mimic 补回物理层）。

它同时是一个**诊断器**：两指实际位置差 > 4 mm 持续 1 s 就告警
（说明 joint2 没跟上：控制器没起 / 被卡住 / 有东西挡住）✓

用法（`turtlebot3_franka.launch.py` 已自动起；手动调试时）：

    ros2 run turtlebot3_manipulation_gazebo finger_mimic_relay.py
    ros2 topic echo /joint_states | grep -A2 finger        # 两指应几乎相同
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

J1 = "fr3_finger_joint1"
J2 = "fr3_finger_joint2"


class FingerMimicRelay(Node):
    def __init__(self):
        super().__init__("finger_mimic_relay")
        self.declare_parameter("source_joint", J1)
        self.declare_parameter("target_joint", J2)
        self.declare_parameter("command_topic", "/finger2_controller/commands")
        self.declare_parameter("warn_diff", 0.004)      # m，两指位置差告警阈值
        self.declare_parameter("warn_hold_s", 1.0)
        # ★ 2026-09-19：日志默认"只在夹爪动过之后打一条总结"，不再周期性刷屏 ✗
        #   （用户反馈：Gazebo 那个终端一直在滚夹指的日志 ✗）
        self.declare_parameter("verbose", False)        # true = 每 2 s 打一次状态（调试用）
        self.declare_parameter("selftest_s", 1.5)       # s：joint1 动了之后，joint2 该多久内跟上

        self._src = self.get_parameter("source_joint").value
        self._dst = self.get_parameter("target_joint").value
        topic = self.get_parameter("command_topic").value
        self._warn_diff = float(self.get_parameter("warn_diff").value)
        self._warn_hold = float(self.get_parameter("warn_hold_s").value)
        self._verbose = bool(self.get_parameter("verbose").value)
        self._selftest_s = float(self.get_parameter("selftest_s").value)

        self._pub = self.create_publisher(Float64MultiArray, topic, 10)
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

        self._q1 = None
        self._q2 = None
        self._n = 0
        self._bad_since = None
        self._warned = False
        # 日志节流用状态（见 _on_js）
        self._last_logged = None      # 上次打日志时的 q1
        self._moving = False          # joint1 是否正在动
        self._t_last_move = 0.0
        self._self_tested = False     # 自检只做一次
        self._stuck_reported = False
        self.get_logger().info(
            "finger_mimic_relay 就绪：把 {} 的位置转发到 {} → {}".format(self._src, topic, self._dst))

    def _on_js(self, msg):
        q1 = q2 = None
        for name, pos in zip(msg.name, msg.position):
            if name == self._src:
                q1 = float(pos)
            elif name == self._dst:
                q2 = float(pos)
        if q1 is None:
            return
        self._q1 = q1
        if q2 is not None:
            self._q2 = q2
        # ① 转发指令（joint2 的控制器是 forward_command ⇒ 直接发位置即可）
        out = Float64MultiArray()
        out.data = [q1]
        self._pub.publish(out)
        # ② 诊断（★ 2026-09-19 改：**不刷屏**）
        #    只在"joint1 动过之后停下来"时打一条总结（一次夹爪动作 ≈ 1~2 行 ✓）；
        #    想持续看就 -p verbose:=true
        self._n += 1
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_logged is None or abs(q1 - self._last_logged) > 0.002:
            self._last_logged = q1
            self._moving = True
            self._t_last_move = now
        elif self._moving and (now - self._t_last_move) > 0.4:
            self._moving = False
            if self._q2 is None:
                self.get_logger().warn(
                    "夹爪停到 {:.4f}：{} 不在 /joint_states 里 ⇒ 控制器没起（"
                    "ros2 control list_controllers）✗".format(q1, self._dst))
            else:
                self.get_logger().info("夹爪停到 {}={:.4f} / {}={:.4f}（差 {:+.1f} mm）".format(
                    self._src, q1, self._dst, self._q2, (self._q2 - q1) * 1000.0))
        elif self._verbose and self._n % 200 == 1:
            self.get_logger().info("{}={:.4f} {}={:.4f}".format(
                self._src, q1, self._dst, self._q2 if self._q2 is not None else float("nan")))

        # ③ ★ 自检：joint1 动过了，但 joint2 一直不跟 ⇒ 指令没生效（把排查命令直接打出来）
        q2_now = self._q2 if self._q2 is not None else q1
        if (not self._self_tested and self._t_last_move > 0
                and (now - self._t_last_move) <= 0.4
                and abs(q1 - 0.04) > 0.004 and abs(q2_now - q1) > 0.004):
            if not self._stuck_reported:
                self._stuck_reported = True
                self.get_logger().error(
                    "✗ 自检不过：joint1 已经动到 {:.4f}，但 joint2 还在 {:.4f}（没跟上）⇒ "
                    "我的指令没生效。请在另一个终端跑：\n"
                    "    ros2 topic info -v /finger2_controller/commands   # 看有没有订阅者\n"
                    "    ros2 topic pub --once /finger2_controller/commands "
                    "std_msgs/msg/Float64MultiArray \"{data: [0.02]}\"   # 看右指动不动\n"
                    "  （前者没订阅者 ⇒ 话题名/命名空间不对；后者也不动 ⇒ 控制器或硬件这一环 ✗）"
                    .format(q1, q2_now))
        if self._q2 is None:
            return
        diff = abs(self._q2 - q1)
        now = self.get_clock().now().nanoseconds * 1e-9
        if diff > self._warn_diff:
            if self._bad_since is None:
                self._bad_since = now
            elif (now - self._bad_since) >= self._warn_hold and not self._warned:
                self._warned = True
                self.get_logger().error(
                    "✗ 两指不同步：{}={:.4f} vs {}={:.4f}（差 {:.1f} mm）—— joint2 没跟上，"
                    "夹爪会不对称（抓取会顶到物体/夹不牢）✗".format(
                        self._src, q1, self._dst, self._q2, diff * 1000.0))
        else:
            self._bad_since = None
            self._warned = False


def main():
    rclpy.init()
    node = FingerMimicRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # ★ 收到 SIGINT/SIGTERM（ros2 launch 关闭 / Ctrl-C）时**安静退出、退出码 0** ✗→✓
        #   否则会打一长串 traceback，而且非零退出码会让 `ros2 launch`
        #   认为"有进程失败"（它默认会因此把**整个仿真**一起关掉 ✗，2026-09-18 踩过）
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:                              # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
