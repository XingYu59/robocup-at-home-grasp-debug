#!/usr/bin/env python3
"""
Launch move_group for the TurtleBot3 + FR3 arm.

Uses MoveItConfigsBuilder so all MoveIt parameters (planning_pipelines,
trajectory_execution / controllers, kinematics, joint limits) are assembled
the way move_group expects for the installed MoveIt version.

Requires:
  - moveit_configs_utils  (sudo apt install ros-humble-moveit-configs-utils)
  - the Gazebo sim already running (controllers are spawned by
    turtlebot3_franka.launch.py)

Usage:
    ros2 launch turtlebot3_moveit_config move_group.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_pkg = "turtlebot3_moveit_config"
    gazebo_pkg = "turtlebot3_manipulation_gazebo"

    # ── Build MoveIt configs ────────────────────────────────────────────
    # Auto-discovers (by convention) in config/:
    #   turtlebot3_manipulation.srdf, joint_limits.yaml, kinematics.yaml,
    #   ompl_planning.yaml, moveit_controllers.yaml
    # robot_description is NOT in this package (it lives in the gazebo pkg),
    # so we feed the same xacro used by turtlebot3_franka.launch.py.
    moveit_config = (
        MoveItConfigsBuilder("turtlebot3_manipulation", package_name=moveit_pkg)
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory(gazebo_pkg),
                "urdf", "turtlebot3_manipulation.urdf.xacro",
            ),
            mappings={"use_sim": "true"},
        )
        # Controllers are pre-spawned and stay active (gz_ros2_control), so
        # MoveIt must not try to switch their lifecycle.
        .trajectory_execution(moveit_manage_controllers=False)
        # Load ONLY ompl; skip chomp/stomp/pilz (avoids needing pilz deps).
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    # MTC 的 task.execute() 需要 execute_task_solution 动作，
    # 由 moveit_task_constructor_capabilities 提供（该参数是【追加】，不覆盖默认能力）。
    move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}

    # ── move_group node ─────────────────────────────────────────────────
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
            {
                "use_sim_time": True,
                "allow_trajectory_execution": True,
                "publish_robot_description_semantic": True,
                "publish_robot_description": True,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Use Gazebo /clock"),
        move_group_node,
    ])
