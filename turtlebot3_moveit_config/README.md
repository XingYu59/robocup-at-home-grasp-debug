# turtlebot3_moveit_config

MoveIt 2 配置包：TurtleBot3 Waffle Pi + FR3 机械臂（Gazebo Fortress）。

只规划 **FR3 臂（arm 组）**；差速底盘仍由 Nav2 控制，不进 MoveIt 规划。
规划坐标系 = 根 link = `base_footprint`（无虚拟关节）。

## 规划组

| 组 | 关节 | tip |
|---|---|---|
| `arm` | `fr3_joint1` … `fr3_joint7` | `fr3_link8` |
| `hand` | `fr3_finger_joint1`（`fr3_finger_joint2` 为 URDF mimic，不在组内） | `fr3_hand_tcp` |

## 关键约定

- 关节限位严格照抄 `ros2_control`，特别注意：
  - `fr3_joint4` 上限为负 `-0.1169`
  - `fr3_joint6` 下限为正 `0.4398`
- 控制器用 **MoveItSimpleControllerManager**：`arm_controller`（FollowJointTrajectory）+
  `gripper_controller`（GripperCommand），二者由 `turtlebot3_franka.launch.py` 提前 spawn，
  MoveIt 不再二次 spawn。
- 规划管线只保留 **OMPL**；MTC 的笛卡尔段用 `moveit_task_constructor_core` 的 `CartesianPath`
  求解器，无需 Pilz。

## 运行（在仿真已启动的前提下）

依赖：`moveit_configs_utils`（launch 用它组装参数）：
```bash
sudo apt install ros-humble-moveit-configs-utils   # 若 ros-humble-moveit 未带
```

```bash
colcon build --symlink-install --packages-select turtlebot3_moveit_config
source install/setup.bash

# 终端1：仿真 + 控制器
ros2 launch turtlebot3_manipulation_gazebo turtlebot3_franka.launch.py
# 终端2：move_group
ros2 launch turtlebot3_moveit_config move_group.launch.py
```

验证：`ros2 node list` 应看到 `move_group`；RViz 中可加载
`robot_description` + `robot_description_semantic` 检查模型与碰撞体。
