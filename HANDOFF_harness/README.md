> **交接/接手调试先看仓库根的 [HANDOFF.md](../HANDOFF.md)** ✓（本目录是调试工具，用法见下）

# HANDOFF 工具目录（**不属于任何 ROS 包**，colcon 不会安装它）

本目录只放**离线自检工具**。这里没有 package.xml，所以放在 `src/` 下不会被 colcon 当成包 ✓。

## 现有文件

| 文件 | 用途 |
|---|---|
| `check_grasp_geometry.py` | **顶抓几何自检**：用 URDF 的真实碰撞几何（fr3_hand 网格 + 两根手指的 8 个碰撞盒）把整只手摆到抓取位姿，逐个采样角做精确碰撞检测 → 回答"这个物体的 `grasp_lift` 能不能规划成功" |
| `_map_table3_zoom.png` / `_map_waypoints.png` | 地图取证图（餐桌脚印 + 巡逻点） |
| `patches/` | 迁移期的临时补丁留档 |

## ★ 已删除：`sim_harness.sh` 与 `grasp_client.py`（2026-09-14）

那套"无头取证台"是旧工作区带过来的：它自己 spawn 机器人、用 `ign model -m <物体> -p`
读 **gz 真值** 来判"物体有没有被抬起来"，还用一条写死的 static TF 伪造 `map→base_footprint`。

**用户明确要求：不许用 gz 真值，调试也不行** —— 它会让"视觉/规划到底行不行"这个结论失效
（真值 = 直接读仿真答案 ✗）。所以整条删除，并同步删掉了抓取链里的真值退路
（`grasp_phase.truth_targets` / `--truth-only` / `--no-truth-fallback` / 适配层的 `debug_truth`）。

它原本的用途现在有**更真实**的替代，都在真仿真 + 真导航栈里跑，不读任何真值：

```bash
# 真·端到端（Gazebo + Nav2/AMCL + 视觉 + 抓取服务）
ros2 launch turtlebot3_manipulation_gazebo turtlebot3_franka.launch.py
ros2 launch turtlebot3_manipulation_navigation2 navigation.launch.py
source ~/venvs/vision_env/bin/activate
ros2 launch turtlebot3_manipulation_grasp grasp_service.launch.py
ros2 run turtlebot3_manipulation_navigation2 patrol_task.py          # 比赛主流程（Phase 1 + 2）

# 只验抓取阶段（等价于旧取证台"机器人已停好、只测服务链路"的用法）
ros2 run turtlebot3_manipulation_navigation2 dining_grasp_task.py --skip-nav --no-creep
```

判"到底抓没抓起来"也别再读真值：**看 Gazebo GUI / 相机图像**，或看
`/joint_states` 里手指关节的到位情况（`fr3_finger_joint1` 卡在物体半宽处 = 夹住了、
一路合到目标值 = 空合）。旧 README 里"罐子 z 抬升 ≥ 3 cm"那条判据是读真值得来的，
**已作废**（但它记录的失败现象仍然有效：6 次实跑里只有 2 次真抓起、且都被带倒 →
根因就是本文档下面那条抓取高度问题）。

## 规划失败时先跑这个

`grasp pose IK (0/25): eef in collision: fr3_hand - object` 这类报错**与视觉精度、定位、
真值完全无关**：MTC 的 `ComputeIK` 在算 IK **之前**就把手按目标位姿摆好做碰撞检查
（`isTargetPoseCollidingInEEF`，见 `moveit_task_constructor/core/src/stages/compute_ik.cpp`），
手和物体一相交就 25 个角全拒。典型根因是 `objects.yaml` 的 `grasp_lift` 让物体顶段插进了手掌
（手掌只到指尖平面上方 37.4 mm）。改 `grasp_lift` 前后各跑一次体检：

```bash
source /opt/ros/humble/setup.bash && source ~/Robocup@home_ws/install/setup.bash
python3 src/HANDOFF_harness/check_grasp_geometry.py            # 全表体检
python3 src/HANDOFF_harness/check_grasp_geometry.py --sweep mustard_bottle
```
