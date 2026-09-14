# GRASP_SERVICE_HOWTO.md —— 固定站位 + 抓取服务：帧、场景、实现骨架

重构后的抓取链路（无 MTC，底盘到站即锁死）：

```text
Nav2 → 固定站位 (3.0, 2.51, yaw=-π/2) → 停稳
   → 驱动把固定罐子/桌面位姿用 TF 换算到 base_footprint
   → 调用 /grasp_fixed_object 服务（grasp_node 提供）
   → grasp_node：把物体/桌面加进 planning scene → 执行抓取（TODO：你来写）
```

## 1. 帧 / TF 关系（已核对，写抓取时照此用）

| 帧 | 说明 |
|---|---|
| `map` | = Gazebo world。驱动里所有"世界坐标"常量都在这 |
| `base_footprint` | **机器人模型根帧 = MoveIt 规划帧**。抓取服务传的位姿都在这里 |
| `base_link` | 底盘帧；`base_footprint → base_link` 固定 `(0, 0, 0.010)`（仅 z 1cm） |
| `fr3_link0` | 臂基座；在 `base_link` 下 `(-0.092, 0, 0.421)`（**在底盘后方**，所以向前可及打折） |
| `fr3_hand_tcp` | **EEF / 抓取帧**（`fr3_hand` 下方 `+0.1034`，≈指尖平面） |

TF 链：`map → (AMCL) → odom → base_footprint → base_link → fr3_link0 → … → fr3_hand_tcp`

**驱动已经替你做完的换算**：机器人停稳后查一次 `map→base_footprint`，把罐子中心
`(3.0, 2.18, 0.84)`（map 系）和桌面中心 `(2.7, 2.0, 0.765, yaw=π/2)` 变换到
`base_footprint`，随服务请求发出 → 你收到 `object_frame = "base_footprint"` 的位姿，
**直接用作 planning scene / MoveIt 目标，不用再动 TF**。

> 为什么用 base_footprint 而不是 base_link？因为 MoveIt 规划帧 = base_footprint
> （URDF 根），base_link 只是其下方 1cm 的子帧。两者数值几乎一致，但统一用
> base_footprint 最不容易错。

## 2. 把固定物体写进仿真世界（SDF）

例：`wpr_simulation_ros2/worlds/example.world` 里（我们已加好这段）：

```xml
<!-- 固定抓取目标罐子（dining_grasp 场景）：dinning_table_3 东侧近边 -->
<include>
  <uri>model://coke_can</uri>          <!-- 资源库模型（wpr models 目录） -->
  <name>coke_can_dining</name>         <!-- 仿真实例名，不能与已有重名 -->
  <pose>3.0 2.18 0.78 0 0 0</pose>     <!-- x y z roll pitch yaw -->
</include>
```

要点：
- `z=0.78` = 桌面顶面高度（罐子模型原点在**底部**，放上去即稳，中心 z≈0.84）。
- 这个位姿就是第 3 节里"世界常量"的来源：`CAN_CENTER_XYZ = (3.0, 2.18, 0.84)`。
- 换物体：把 `uri`/`name`/`pose` 换成你的模型；几何尺寸（圆柱高/半径）同步改
  驱动里的 `CAN_DIMS` 和服务请求。

## 3. 把物体显式加进 planning scene（C++，MoveIt 官方做法）

grasp_node.cpp 的服务回调里已经写了演示（收到的请求直接加）：

```cpp
moveit::planning_interface::PlanningSceneInterface psi;

// 罐子 = 圆柱（高 h、半径 r），中心在 object_pose（frame = object_frame）
moveit_msgs::msg::CollisionObject obj;
obj.id = "object";
obj.header.frame_id = req->object_frame;      // "base_footprint"
obj.primitives.resize(1);
obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
obj.primitives[0].dimensions = { req->object_height, req->object_radius };
obj.primitive_poses.push_back(req->object_pose);
obj.operation = moveit_msgs::msg::CollisionObject::ADD;
psi.applyCollisionObject(obj);
```

桌面同理（BOX + table_pose，代码已在 grasp_node.cpp）。发出去后 move_group 的
规划场景就"看得到"罐子和桌子 → 规划自动避让桌面。

## 4. 服务契约

`srv/GraspFixedObject.srv`（`ros2 interface show turtlebot3_manipulation_grasp/srv/GraspFixedObject`）：

```
请求: object_pose (几何位姿, frame=object_frame="base_footprint")
      object_height / object_radius    罐子圆柱 (0.12, 0.033)
      table_pose + table_length/width/thickness   桌面盒
响应: success / message
```

## 5. 平铺抓取实现骨架（填 grasp_node.cpp 的 TODO）

参考官方 pick_and_place demo，最简可读版：

```cpp
#include <moveit/move_group_interface/move_group_interface.h>

// 1) arm 组（EEF 链到 fr3_hand_tcp）；手用 gripper_controller
moveit::planning_interface::MoveGroupInterface arm(node, "arm");
arm.setMaxVelocityScalingFactor(0.4);
arm.setStartStateToCurrentState();

// 2) pre-grasp：罐子上方 ~0.1m，手指朝下（TCP +z 朝下，Rx(π) 即 180° 绕 x）
//    object_pose 已在 base_footprint，直接在其上叠一个 offset
auto pregrasp = req->object_pose;  // 或 table/固定常数
pregrasp.position.z += 0.12;       // 罐心 0.84 → 0.96
pregrasp.orientation = /* Rx(π): (1,0,0,0) 的四元数 ≈ (x=1,y=0,z=0,w=0) */;
// 实际朝向请先在 RViz 里对着 fr3_hand_tcp 试，找到"手指朝下"的那组 rpy

arm.setPoseTarget(pregrasp, "fr3_hand_tcp");
// plan → execute（查 arm.plan()/arm.move()）

// 3) approach：笛卡尔下移 ~0.08m（computeCartesianPath 或直接再设一个目标）
// 4) close：夹爪动作（gripper_controller 的 GripperCommand action）
// 5) lift：笛卡尔上移 0.15m
// 6) place：下移回原处 → open → retreat
```

调试顺序建议（先确认每一步，再连起来）：
1. 只做 `pre-grasp` 定点：在 RViz 里看 EEF 是否到罐子正上方、手指朝下；
2. 加上 approach/close；
3. 再加 lift/place。

## 6. 运行

```bash
# 终端1 仿真（罐子已在 example.world，直接就有）
ros2 launch turtlebot3_manipulation_gazebo turtlebot3_franka.launch.py
# 终端2 导航
ros2 launch turtlebot3_manipulation_navigation2 navigation.launch.py start_rviz:=false
# 终端3 move_group + grasp 服务
ros2 launch turtlebot3_manipulation_grasp grasp_service.launch.py
# 终端4 驱动（导航→停稳→调服务）
ros2 run turtlebot3_manipulation_navigation2 dining_grasp_task.py

# 调试（跳过导航，机器人已手动停站位）：
ros2 run turtlebot3_manipulation_navigation2 dining_grasp_task.py --skip-nav
# 手动触发服务自测（不跑驱动）：
ros2 service call /grasp_fixed_object turtlebot3_manipulation_grasp/srv/GraspFixedObject \
  "{object_frame: 'base_footprint', object_pose: {position: {x: 0.33, y: 0.0, z: 0.84}}, object_height: 0.12, object_radius: 0.033}"
```
