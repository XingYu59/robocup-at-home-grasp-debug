#!/usr/bin/env python3
"""
Launch move_group + grasp_node(服务) [+ 视觉适配层] for the fixed-station grasp scenario.

Driver (dining_grasp_task.py) navigates to the fixed station, stops, then calls
the /grasp_fixed_object service on grasp_node. The actual pick-and-place
execution is implemented by the user inside grasp_node.cpp (TODO), using
MoveGroupInterface (official pick_and_place demo style).

Requires Gazebo sim already running:
    ros2 launch turtlebot3_manipulation_gazebo turtlebot3_franka.launch.py

视觉适配层（enable_vision:=true，默认开）：
    启动 detect_grasp_target_node.py，提供 /detect_grasp_target 服务
    （视觉检测 → 3D → 契约点）。它需要 torch/groundingdino/sam2 →
    本 launch **显式用视觉 venv 的解释器**启动它，所以不必先 source 激活 ✓
    （解释器路径可用环境变量 VISION_PYTHON 覆盖，默认 ~/venvs/vision_env/bin/python；
      venv 不存在时退回普通 Node，那时才需要手动 source）
    不想要它 / 没装视觉环境时：ros2 launch ... enable_vision:=false
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_pkg = "turtlebot3_moveit_config"
    gazebo_pkg = "turtlebot3_manipulation_gazebo"
    use_sim_time = LaunchConfiguration("use_sim_time")

    moveit_config = (
        MoveItConfigsBuilder("turtlebot3_manipulation", package_name=moveit_pkg)
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory(gazebo_pkg),
                "urdf", "turtlebot3_manipulation.urdf.xacro",
            ),
            mappings={"use_sim": "true"},
        )
        .trajectory_execution(moveit_manage_controllers=False)
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    # MTC 的 task.execute() 需要 move_group 提供 execute_task_solution 动作，
    # 由 moveit_task_constructor_capabilities 的 ExecuteTaskSolutionCapability 提供。
    # move_group 会把该参数【追加】到默认 capability 列表上（先插 DEFAULT_CAPABILITIES
    # 再插本参数），所以 ApplyPlanningSceneService 等默认能力仍然可用。
    move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
            {
                "use_sim_time": use_sim_time,
                "allow_trajectory_execution": True,
                "publish_robot_description_semantic": True,
                "publish_robot_description": True,
            },
        ],
    )

    grasp_node = Node(
        package="turtlebot3_manipulation_grasp",
        executable="grasp_node",
        name="grasp_node",
        output="screen",
        # robot_description / SRDF / kinematics / ompl：规划实现（pick_and_place）要用；
        # grasp_params.yaml：适配层与规划层的全部外部配置（含 objects.yaml 的位置）。
        parameters=[
            moveit_config.to_dict(),
            os.path.join(
                get_package_share_directory("turtlebot3_manipulation_grasp"),
                "config", "grasp_params.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    # ── 视觉适配层：/detect_grasp_target 服务 ─────────────────────
    # 它 import 队友的 vision_pipeline（从 nav2 包的 lib/ 下解析），需要 torch →
    # 默认**显式用视觉 venv 的解释器**启动它（VISION_PYTHON 可覆盖），
    # 这样就不必先 source ~/venvs/vision_env/bin/activate 了 ✓
    # （venv 里没有 / 脚本还没装 → 退回普通 Node，那时才需要手动激活 venv）
    vision_python = os.environ.get("VISION_PYTHON",
                                   os.path.expanduser("~/venvs/vision_env/bin/python"))
    vision_script = os.path.join(get_package_prefix("turtlebot3_manipulation_grasp"), "lib",
                                 "turtlebot3_manipulation_grasp", "detect_grasp_target_node.py")
    vision_params = {
        "use_sim_time": use_sim_time,
        "target_frame": LaunchConfiguration("target_frame"),
        # 支撑面高度不在这里写死：适配层自己去读 grasp_params.yaml 的
        # support_surface（pose.z + thickness/2），与抓取侧同一个数 ✓
    }
    if os.path.isfile(vision_python) and os.path.isfile(vision_script):
        print("[grasp_service] 视觉适配层用解释器: {}".format(vision_python))
        vision_node = ExecuteProcess(
            cmd=[vision_python, vision_script, "--ros-args",
                 "-p", ["use_sim_time:=", use_sim_time],
                 "-p", ["target_frame:=", LaunchConfiguration("target_frame")]],
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_vision")),
        )
    else:
        print("[grasp_service] 找不到 {} 或 {} → 退回普通 Node，"
              "此时需要先 source 视觉虚拟环境".format(vision_python, vision_script))
        vision_node = Node(
            package="turtlebot3_manipulation_grasp",
            executable="detect_grasp_target_node.py",
            name="detect_grasp_target",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_vision")),
            parameters=[vision_params],
        )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("enable_vision", default_value="true",
                              description="是否启动 /detect_grasp_target 视觉适配层"),
        DeclareLaunchArgument("target_frame", default_value="base_footprint",
                              description="视觉目标的输出帧（契约推荐 base_footprint 或相机光学帧）"),
        move_group_node,
        grasp_node,
        vision_node,
    ])
