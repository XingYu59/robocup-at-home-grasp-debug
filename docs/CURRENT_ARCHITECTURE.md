# CURRENT_ARCHITECTURE.md —— 当前抓取系统架构分析

> 依据重构规划 Phase 1：先完整分析、**不先大规模改代码**。理清"入口/节点/数据流/
> MoveIt 调用/夹爪/物体位姿来源/抓取位姿生成/TF/Nav2 衔接"，并标注每部分的
> 保留 / 简化 / 禁用 / 重写结论。

## 1. 抓取系统入口

- 驱动节点（Python）：`turtlebot3_manipulation_navigation2/scripts/dining_grasp_task.py`
  - `ros2 run turtlebot3_manipulation_navigation2 dining_grasp_task.py`
  - 流程：等 Nav2 → 发 /initialpose → spawn 罐子 → **导航到固定站位** → 底盘停止
    → 用 gazebo 真值算 object/table 位姿 → 子进程启动 `dining_grasp.launch.py`（含
    move_group + grasp_node）→ 抓取子进程负责规划+执行 → 打印结果。
- 抓取节点（C++）：`turtlebot3_manipulation_grasp/src/grasp_node.cpp`（main），
  内部构建 **MTC（MoveIt Task Constructor）任务** `PickPlaceTask`。

## 2. 主要节点

| 节点 | 语言 | 作用 |
|---|---|---|
| `grasp_node` | C++ | 建 planning scene（桌子+罐子+可选碗），跑 MTC 任务 |
| `move_group` | C++ | 由 launch 启动，带 `ExecuteTaskSolutionCapability`（供 MTC 执行） |
| `dining_grasp_task.py` | Python | 导航 + TF/gz 位姿 + 触发抓取（一次性编排） |
| `arm_controller` / `gripper_controller` | ros2_control | 执行轨迹(arm)、GripperAction(手) |

## 3. 节点间数据流

```
dining_grasp_task.py
   │  (spawn can / nav / 测位姿)
   │
   ├─ object_pose + table_pose (+ grasp_lift) → 子进程 launch 参数
   ▼
dining_grasp.launch.py
   ├─ move_group（含 ExecuteTaskSolutionCapability）
   └─ grasp_node（读参数 → PSI 加碰撞对象 → MTC 规划→执行 → 退出）
```

## 4. MoveIt 的调用位置

- **唯一调用点**在 `grasp_node.cpp` 内的 `PickPlaceTask`（`moveit/task_constructor/...`）。
- 用 `MoveItConfigsBuilder("turtlebot3_manipulation", package_name="turtlebot3_moveit_config")`
  生成配置（robot_description / kinematics / planning_pipelines）。
- 规划：`task_->plan(max_solutions)`；执行：`task_->execute()`（MTC 整体发到
  move_group 的 `execute_task_solution`）。
- **没有**直接用 `MoveGroupInterface`；一切经由 MTC 的 stage 抽象。

## 5. Gripper 调用位置

- 也在 MTC 内：`MoveTo("open hand"/"close hand", group="hand", goal=OPEN/CLOSE)`。
- 夹爪是 SRDF `hand` 组，命名姿态 `open=0.04 / close=0.0`（驱动 `fr3_finger_joint1`，
  `fr3_finger_joint2` 为 URDF mimic）。
- 底层由 `gripper_controller`（GripperActionController）执行（由 sim launch 的
  spawner 加载，move_group 通过 trajectory 控制器下发）。

## 6. Object Pose 来源

- 语义：罐心在 `base_footprint` 系下的位姿。
- 计算（驱动侧）：`gz model -p` 读**罐子真实位姿** 与 **机器人真实位姿**，
  `object_base = 机器人⁻¹ ∘ 罐子`；gz CLI 不可用时回退 **TF(map→base_footprint)**。
- 传给 grasp_node 的参数：`object_pose=[x y z qx qy qz qw]`（7 元）、
  `object_dims=[height, radius]`（罐子 → 圆柱 0.12×0.033）。

## 7. Grasp Pose 生成方式

- **MTC `GenerateGraspPose`**：围绕物体圆柱采样（angle delta=π/12），生成 25 个
  候选抓取位姿，再由 `ComputeIK`（IK frame=`fr3_hand_tcp`，grasp_frame =
  `T(0,0,grasp_lift)*Rx(π)`）解 IK。
- 即：**抓取位姿由 MTC 自动采样 + IK**，不是固定位姿。

## 8. TF 使用方式

| 帧 | 说明 |
|---|---|
| `map` | = world（map.yaml origin 仅对齐，非偏移） |
| `base_footprint` | 模型根 / **MoveIt 规划帧**（SRDF+URDF 根） |
| `base_link` | 底盘帧；`base_footprint→base_link` 固定 `(0,0,0.010)` |
| `fr3_link0` | 臂基座；在 `base_link` 下 `(-0.092, 0, 0.421)` |
| `fr3_hand_tcp` | 抓取/EEF 帧（`fr3_hand` 下方 +0.1034） |

- 驱动用 gz/TF 把罐子世界坐标 → `base_footprint`，再交给 grasp_node。
- grasp_node 以 `base_footprint`（=规划帧）作为碰撞对象 / 目标帧。

## 9. Nav2 与抓取之间的连接

- 驱动发 `NavigateToPose(固定站位 map 系)`；到达后底盘**停止**（不再移动）。
- 然后启动 grasp 子进程（move_group+grasp_node）。**无** Nav2 参数/目标动态修改
  （已删掉转向对准与动态前移，仅保留碰撞豁免/几何修正等抓取侧逻辑）。

## 10. 保留 / 简化 / 禁用 / 重写结论

**必须保留**
- `turtlebot3_moveit_config`（SRDF / kinematics / ompl / controllers / 命名姿态）。
- `robot_description`/URDF（TB3+FR3 链、riser、hand_tcp）。
- ros2_control 控制器（arm / gripper）与 sim launch。
- 正确的 TF 链（上面第 8 节）。
- `PlanningScene` 用法（PSI 加 table+object 碰撞对象）。

**可以简化**
- `PickPlaceTask`/`grasp_node` 的 MTC 流程 → 可替换为 **MoveGroupInterface 平铺式
  pre-grasp→approach→close→lift→place→open→retreat**（见第二阶段设计）。
- 驱动里的"gz 真值 / TF 双通道"位姿计算 → 可用**固定已知物体位姿**代替（物体固定）。

**暂时禁用**
- 动态底盘调整 / 自动重定位 / 动态改 Nav2 goal（已无）。
- 动态 grasp-pose 搜索 → 固定 GRASP_POSE（本次重构重点）。
- 物体/视觉动态检测。

**建议重写（本次重构核心）**
- 用 `pick_and_place_node.cpp`（平铺 MoveIt）替代 `PickPlaceTask` 的 MTC 阶段链：
  - 减少抽象层，贴近官方 demo：`setStartState→setPoseTarget(pre_grasp)→plan→execute
    →cartesian 下移(approach)→close→cartesian 上移(lift)→place→open→retreat`。
  - 消除 MTC 的 attach 顺序 / approach min_fraction / close-hand GOAL_STATE_INVALID
    等 stage 校验是当前调试的"磨人点"，平铺后每步独立可查。
