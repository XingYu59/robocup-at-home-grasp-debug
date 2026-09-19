#!/usr/bin/env python3
"""
Fortress 仿真启动文件

一键启动 Gazebo Sim + TurtleBot3(带臂) + ros2_control + 传感器桥接
用法:
  ros2 launch turtlebot3_manipulation_gazebo turtlebot3_franka.launch.py
  ros2 launch turtlebot3_manipulation_gazebo turtlebot3_franka.launch.py rviz:=true
"""

import os

import xacro
from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_world(world_arg, install_share):
    """把 world 参数解析成**真实存在**的绝对路径 → (路径|None, 试过的候选列表)。

    ★ 为什么需要它（2026-09-19 现场踩过）：比赛当天往
      `src/wpr_simulation_ros2/worlds/` 里丢一个新的 `.world` 之后，
      `install/.../share/wpr_simulation_ros2/worlds/` 里**不会自动出现**它 ——
      `--symlink-install` 是"**构建时**逐个文件做软链"（实测：install 的 worlds/ 里
      只有 example.world 一个软链 ✗）⇒ `ign gazebo <install>/…/新文件.world`
      找不到文件、退出码 255 ⇒ 整个 launch 连机器人一起没起来 ✗
      （launch.log 原文：`process has died [pid …, exit code 255, cmd 'ign gazebo …']`；
        随后 `create` 取不到 world 列表也死了 ⇒ 表现成"换完世界就完全跑不起来"）

    解析顺序：① 原样给的路径（绝对/相对 cwd）
              ② <install share>/worlds/<文件名>
              ③ **源码树**/wpr_simulation_ros2/worlds/<文件名>
                 （源码目录由 install 里 `example.world` 这个软链反推 ✓ 稳）
    """
    cands = []
    base = os.path.basename(world_arg or "")
    if world_arg:
        cands.append(world_arg)
        if base:
            cands.append(os.path.join(install_share, "worlds", base))
            link = os.path.join(install_share, "worlds", "example.world")
            if os.path.islink(link):
                cands.append(os.path.join(os.path.dirname(os.path.realpath(link)), base))
    for c in cands:
        if c and os.path.isfile(c):
            return c, cands
    return None, cands


def generate_launch_description():
    pkg_share = get_package_share_directory("turtlebot3_manipulation_gazebo")
    wpr_share = get_package_share_directory("wpr_simulation_ros2")

    rviz = LaunchConfiguration("rviz", default="false")
    # ★ 2026-09-19：world 与出生位姿都改成 launch 参数 ⇒ **换世界只改命令行** ✓
    #   比赛当天裁判给另一个 world 文件时：
    #     ros2 launch turtlebot3_manipulation_gazebo turtlebot3_franka.launch.py \
    #         world:=/abs/path/arena.world spawn_x:=-5.3 spawn_y:=-0.5 spawn_yaw:=0.0
    #   （原来 world_file 写死在 Python 里 ✗；出生位姿也是 ✗）
    #   ⚠ 出生位姿同时是 AMCL 初始位姿的来源（grasp_phase.py: INITIAL_X/Y/YAW）⇒
    #     两处必须一致；HANDOFF_harness/check_world_swap.py 会核对并告诉你改哪一行 ✓
    world_file = LaunchConfiguration("world")

    # ── 1. xacro → URDF ─────────────────────────────────────────
    xacro_path = os.path.join(pkg_share, "urdf",
                              "turtlebot3_manipulation.urdf.xacro")
    doc = xacro.process_file(xacro_path, mappings={"use_sim": "true"})
    robot_description_xml = doc.toxml()

    # ── 2. 资源路径 ──────────────────────────────────────────────
    # model://package_name/... URI 解析：Gazebo 在 IGN_GAZEBO_RESOURCE_PATH
    # 的每个目录下查找 package_name/ 子目录。必须包含 PACKAGE 的父目录。
    # 同时把 /opt/ros/humble/lib 加入系统插件路径，让 Gazebo 能找到
    # libgz_ros2_control-system.so。
    # model://franka_description/... 需要 franka_description 的父目录
    franka_share = get_package_share_directory("franka_description")
    resource_path = os.pathsep.join([
        os.path.join(pkg_share, "models"),
        os.path.join(wpr_share, "models"),
        os.path.dirname(pkg_share),       # model://turtlebot3_manipulation_gazebo/meshes/...
        os.path.dirname(franka_share),    # model://franka_description/meshes/...
        "/opt/ros/humble/lib",            # libgz_ros2_control-system.so
        os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
    ])

    # ── 3. 节点 ─────────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_xml,
                     "use_sim_time": True}],
    )

    # Humble = Fortress = ign gazebo（不是 gz sim）
    # ★ 换世界最常见的坑：新 .world 没进 install ⇒ 这里按三处候选找，并打印用的是哪个 ✓
    def _gazebo(context):
        world_arg = LaunchConfiguration("world").perform(context)
        path, cands = _resolve_world(world_arg, wpr_share)
        if path is None:
            print("\n[launch] ✗ world 文件找不到: {!r}".format(world_arg), flush=True)
            for c in cands:
                print("         试过: {}".format(c), flush=True)
            print("         → 用绝对路径: world:=/abs/path/xxx.world",
                  flush=True)
            print("         → 或先把新文件装进 install: "
                  "colcon build --symlink-install --packages-select wpr_simulation_ros2",
                  flush=True)
            raise RuntimeError("world 文件不存在: {}".format(world_arg))
        if os.path.abspath(path) != os.path.abspath(world_arg or ""):
            print("[launch] world 自动解析为: {}（给的是 {!r}）".format(path, world_arg),
                  flush=True)
        else:
            print("[launch] world = {}".format(path), flush=True)
        return [ExecuteProcess(cmd=["ign", "gazebo", path, "-r"], output="screen")]

    gazebo = OpaqueFunction(function=_gazebo)

    # 从 /robot_description 话题 spawn 机器人
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "/robot_description",
                   "-name", "turtlebot3",
                   "-x", LaunchConfiguration("spawn_x"),
                   "-y", LaunchConfiguration("spawn_y"),
                   "-z", LaunchConfiguration("spawn_z"),
                   "-Y", LaunchConfiguration("spawn_yaw")],
        output="screen",
    )

    # controller_manager 由 gz_ros2_control 插件在 Gazebo 内提供
    # --controller-type 必须显式指定：gz_ros2_control 的 controller_manager
    # 解析 params_file 时可能丢失 type 参数（与独立 ros2_control_node 不同）
    param_file = os.path.join(pkg_share, "config",
                              "hardware_controller_manager.yaml")
    controller_args = [
        "--controller-manager", "/controller_manager",
        "--controller-manager-timeout", "60",
        "--param-file", param_file,
    ]

    joint_state = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-type",
                   "joint_state_broadcaster/JointStateBroadcaster",
                   *controller_args],
        output="screen",
    )
    diff_drive = Node(
        package="controller_manager", executable="spawner",
        arguments=["diff_drive_controller",
                   "--controller-type",
                   "diff_drive_controller/DiffDriveController",
                   *controller_args],
        output="screen",
    )
    imu = Node(
        package="controller_manager", executable="spawner",
        arguments=["imu_broadcaster",
                   "--controller-type",
                   "imu_sensor_broadcaster/IMUSensorBroadcaster",
                   *controller_args],
        output="screen",
    )
    arm = Node(
        package="controller_manager", executable="spawner",
        arguments=["arm_controller",
                   "--controller-type",
                   "joint_trajectory_controller/JointTrajectoryController",
                   *controller_args],
        output="screen",
    )
    gripper = Node(
        package="controller_manager", executable="spawner",
        arguments=["gripper_controller",
                   "--controller-type",
                   "position_controllers/GripperActionController",
                   *controller_args],
        output="screen",
    )
    # ★ 2026-09-18：第二个手指的控制器 + 中继（Fortress 不支持 URDF mimic，
    #   joint2 必须自己受控并跟着 joint1 走，否则它当被动关节停在闭合端 ✗）
    finger2 = Node(
        package="controller_manager", executable="spawner",
        arguments=["finger2_controller",
                   "--controller-type",
                   "forward_command_controller/ForwardCommandController",
                   *controller_args],
        output="screen",
    )
    # ★ 中继是**辅助节点**：它的失败绝不能把整个仿真带走 ✗
    #   2026-09-18 实测踩过：脚本没有执行权限（-rw-------）⇒ Permission denied ⇒
    #   退出码 126 ⇒ ros2 launch 默认"任一进程非零退出就全部关闭" ⇒ **Gazebo 一起闪退** ✗
    #   所以用 shell 包一层：失败时打醒目提示并保持存活（sleep）⇒ 仿真继续跑，
    #   用户只需单独重启中继 ✓（顺带：退出码 0 的那次关闭不会触发这条 ✓）
    relay_exe = os.path.join(
        get_package_prefix("turtlebot3_manipulation_gazebo"),
        "lib", "turtlebot3_manipulation_gazebo", "finger_mimic_relay.py")
    finger_relay = ExecuteProcess(
        cmd=["bash", "-c",
             "python3 -u '{}' || {{ echo ''; "
             "echo '✗✗✗ finger_mimic_relay 启动失败 —— 两指不会同步，抓取会顶到物体！"
             "检查 /finger2_controller 是否存在、脚本权限是否 755'; sleep infinity; }}"
             .format(relay_exe)],
        name="finger_mimic_relay_wrapper",
        output="screen",
    )

    # ── 4. 传感器桥接：Ign topic → ROS topic ────────────────────
    #      格式: /topic@ROS_type[gz_type  (GZ→ROS)
    #            /topic@ROS_type]gz_type  (ROS→GZ)
    #            /topic@ROS_type@gz_type  (双向)
    #      重映射通过 Node 的 remappings 参数实现（非逗号语法！）

    # 传感器通过 <topic> 元素设定了短路径，直接用根级话题名。
    scan_topic = "/scan"
    image_topic = "/pi_camera/image"
    depth_topic = "/pi_camera/depth_image"
    camera_info_topic = "/pi_camera/camera_info"

    clock_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )
    lidar_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        name="lidar_bridge",
        arguments=[f"{scan_topic}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
        remappings=[(scan_topic, "/scan")],
        # 关键：Gazebo 传感器帧被加了模型名前缀 "turtlebot3/"，
        # 而 TF 树里的帧是未加前缀的 base_scan。必须强制覆盖，否则
        # AMCL/代价地图的 message filter 永远无法变换 /scan 而丢弃它。
        parameters=[{"override_frame_id": "base_scan"}],
        output="screen",
    )
    camera_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        name="camera_bridge",
        arguments=[f"{image_topic}@sensor_msgs/msg/Image[gz.msgs.Image"],
        remappings=[(image_topic, "/camera/image_raw")],
        # 与 lidar 同理：Gazebo 给相机帧加 "turtlebot3/" 前缀，而 TF 树里是
        # 未加前缀的 camera_rgb_optical_frame，必须强制覆盖，否则 3D 定位的
        # TF 查询会失败。RGB/depth/camera_info 三者共用同一光学帧。
        parameters=[{"override_frame_id": "camera_rgb_optical_frame"}],
        output="screen",
    )
    depth_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        name="depth_bridge",
        arguments=[f"{depth_topic}@sensor_msgs/msg/Image[gz.msgs.Image"],
        remappings=[(depth_topic, "/camera/depth/image_raw")],
        parameters=[{"override_frame_id": "camera_rgb_optical_frame"}],
        output="screen",
    )
    camera_info_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        name="camera_info_bridge",
        arguments=[f"{camera_info_topic}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
        remappings=[(camera_info_topic, "/camera/camera_info")],
        parameters=[{"override_frame_id": "camera_rgb_optical_frame"}],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2", executable="rviz2",
        arguments=["-d", os.path.join(pkg_share, "rviz",
                                       "turtlebot3_manipulation.rviz")],
        parameters=[{"use_sim_time": True}],
        output="log",
        condition=IfCondition(rviz),
    )

    # ── 5. 顺序启动 ─────────────────────────────────────────────
    # ── launch 参数（换世界 / 换出生点只改命令行 ✓）────────────────
    declare = [
        DeclareLaunchArgument(
            # ★ 默认值给**文件名**而不是 install 绝对路径（2026-09-19 现场两次踩坑）：
            #   ① 绝对 install 路径在新文件还没构建进 install 时必然失效 ✗
            #   ② 改了默认值之后又给世界文件改名 ⇒ 默认值指向不存在的文件 ✗
            #   ⇒ 给文件名，_resolve_world() 会去 install/worlds 与**源码树** worlds 两处找 ✓
            "world", default_value="example.world",
            description="世界文件名或路径（默认 example.world）；找不到会打印候选并明确报错 ✓"),
        DeclareLaunchArgument("spawn_x", default_value="-5.30",
                              description="机器人出生 x（同时是 AMCL 初始位姿，见 grasp_phase.py）"),
        DeclareLaunchArgument("spawn_y", default_value="-0.50", description="机器人出生 y"),
        DeclareLaunchArgument("spawn_z", default_value="0.01", description="机器人出生 z"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0", description="机器人出生偏航（rad）"),
    ]

    ld = LaunchDescription(declare + [
        DeclareLaunchArgument("rviz", default_value="false",
                              description="是否启动 RViz"),
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
                               "/opt/ros/humble/lib/"),
        robot_state_publisher,
        gazebo,
        rviz_node,
        spawn_robot,
        clock_bridge,
        lidar_bridge,
        camera_bridge,
        depth_bridge,
        camera_info_bridge,
        # 控制器按顺序启动，避免并发 set_parameters 造成竞态条件
        # （FR3 demo 采用同样策略）
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state])),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state,
            on_exit=[diff_drive])),
        RegisterEventHandler(OnProcessExit(
            target_action=diff_drive,
            on_exit=[imu])),
        RegisterEventHandler(OnProcessExit(
            target_action=imu,
            on_exit=[arm])),
        RegisterEventHandler(OnProcessExit(
            target_action=arm,
            on_exit=[gripper])),
        # gripper_controller 起来之后再起 joint2 的控制器与中继
        # （中继要发 /finger2_controller/commands，控制器没起会丢指令 ✗）
        RegisterEventHandler(OnProcessExit(
            target_action=gripper,
            on_exit=[finger2])),
        RegisterEventHandler(OnProcessExit(
            target_action=finger2,
            on_exit=[finger_relay])),
    ])
    return ld
