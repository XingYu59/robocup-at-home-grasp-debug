# ⚠️ 本文已被 GRASP_SERVICE_HOWTO.md 取代（架构重构：MTC 已移除，改服务化抓取）

> 2026-09 重构后请以 **docs/GRASP_SERVICE_HOWTO.md** 为准：驱动导航到固定站位后调用
> `/grasp_fixed_object` 服务；grasp_node 仅提供服务骨架（planning scene 演示 +
> TODO 由你实现 MoveIt 平铺抓取）。本文历史内容（MTC 调参/排查表）仅供参考。

把「Nav2 导航到餐桌 → 抓取固定位姿的 coke_can（抬离再放回）」串成一条
可跑的链路。视觉暂不接入：物体位姿是**已知世界坐标**，停车后由 TF 转到
base_footprint 再喂给 grasp_node。

## 重要前提（2026-09 修复）：代价地图里补上了餐桌实体

**现象**：静态地图 map.pgm 里餐桌区域是空的（只有激光扫到的细桌腿在局部
代价地图上闪断），Nav2 会把机器人往"看起来有缝/桌面下"的地方带——导航吃力、
停不到理想抓取位。
**修复**：按 example.world 里 10 张桌子的真实碰撞足迹，把餐桌矩形涂成
occupied 写回 `map.pgm`（原图备份在 `map_backup.pgm`）。**改地图后必须重启
navigation.launch（map_server 启动时加载）**，RViz 里餐桌应显示为实心块。
四张餐厅桌是紧挨的 2×2 整块 x∈[0.9,3.3] y∈[1.25,2.25]，停车点不能选在块内。

## 本轮改动（代码已完成并编译通过）

| 文件 | 改动 |
|---|---|
| `turtlebot3_manipulation_navigation2/map/map.pgm` | **10 张餐桌真实足迹涂为障碍**（备份 map_backup.pgm） |
| `turtlebot3_manipulation_grasp/src/grasp_node.cpp` | 新增参数 `exit_after_execute`（跑完自动退出）；**整段 run 加 try/catch**——MTC/moveit 异常不再 SIGABRT 静默退出，会打印 `Exception during task run: ...`；启动时打印实际收到的 object/table 位姿 |
| `turtlebot3_manipulation_grasp/launch/dining_grasp.launch.py` | **新增**：move_group + grasp_node，`object_pose/object_dims/table_pose/table_dims` 可命令行覆盖（OpaqueFunction 解析） |
| `turtlebot3_manipulation_navigation2/scripts/dining_grasp_task.py` | **新增**：驱动节点（纯 rclpy，**不 import vision_pipeline**）。导航逻辑与 patrol_task 一致（goal 被拒重试、**不中途超时打断**）；停车后**位置精调**：实测 dx/dy（**Gazebo 真值**：`gz model -p` 读真实罐子/机器人位姿，绕开 AMCL 误差与 spawn 漂移），先原地转向对准罐子再低速前移逼近到 ~0.30m；抓取位姿同样用真值计算 |

## 场景几何（脚本顶部常量，已按 example.world 核实）

```
dinning_table_3 : 世界中心 (2.7, 2.0, 0.765) —— **碗所在的那张桌**（碗在 (2.67,2.015)）
罐子(coke_can)  : spawn 在 (3.0, 2.18, 0.78) —— 桌东侧近边（与碗相距 ~0.37m）；
                  落定后中心 z≈0.84
停车航点        : (3.0, 2.66)，yaw=-π/2（固定最优点位，车头朝 -Y，距桌北缘 ~0.43m）
理想抓取距离    : grasp_dist=0.45m（罐心到车头；0.30 太近→手臂蜷缩易撞罐，
                  0.40 更伸展；可 --grasp-dist 调，上限 ~0.55 受 FR3 臂展限制）
抓取高度        : grasp_lift=0.03（指尖夹罐身上段而非罐顶；可 --grasp-lift 调）
```

## 运行（三个终端）

```bash
# 终端1：仿真
ros2 launch turtlebot3_manipulation_gazebo turtlebot3_franka.launch.py

# 终端2：导航
ros2 launch turtlebot3_manipulation_navigation2 navigation.launch.py start_rviz:=false

# 终端3：任务
ros2 run turtlebot3_manipulation_navigation2 dining_grasp_task.py
```

## 预期日志里程碑（照这个顺序检查）

1. `Nav2 action server 就绪`
2. `spawn 罐子: ...` + ros_gz_sim create 无报错（在 /world/home）
3. `[1/6] 导航至 dining_park (2.67, 2.78...)` → `✓ 导航成功到达（Nav2 status=4）`
   （导航**不设中途超时**，与 patrol_task 一致——Nav2 吃力时先跑完 recovery
   再继续，耐心等，别 Ctrl-C）
4. `实测罐子 base 偏移: dx=... dy=...` → 需要时 `原地转向 xx° 对准罐子` /
   `前移 xx m 逼近罐子`，直到 `✓ 罐子已在理想抓取位置 (dx≈0.40)`
   （Nav2 停车精度差是正常的，这一步负责摆到位，目标距离可 --grasp-dist 调；dy 大说明朝向/横向偏，
   会先原地转向）
5. `罐子 base 位姿` ≈ `[0.30, ~0.0, 0.84, ...]`、`桌子 base 位姿` ≈ `[0.65, ~0.0, 0.765, ...]`
6. `[grasp] Planning succeeded` → `[grasp] Execution complete`
   `[grasp] exit_after_execute=true, shutting down after run (ok=1)`
7. `任务完成: 抓取成功`

> grasp_node 侧现在会先打印 `object_pose=[...] table_pose=[...]` 便于核对；
> 若 MTC/moveit 抛异常，会打印 `Exception during task run: ...` 而不是无征兆
> SIGABRT（exit -6）。

## 真抓验证

抓取执行后**罐子是否真的离开桌面再放回**（区别于之前的虚拟碰撞体）：
- rviz / 相机肉眼看；或
- `gz model -m coke_can_dining -i` / `gz topic -e -t /world/home/pose/info` 查罐子 z：
  被抓时应 > 0.90（抬离 ~0.12–0.18m），放回后 ≈0.78。

## 失败排查表

| 现象 | 原因/处理 |
|---|---|
| `罐子 spawn 失败` / create 报错 | 世界名必须是 `home`（example.world 的 `<world name>`）；模型 sdf 路径取自 `wpr_simulation_ros2` share，确认已 build |
| 导航失败/被拒 | 目标在不可达区域/被拒：看 rviz costmap；与 patrol_task 相同，驱动会重试 6 次（Nav2 的 bt_navigator 要等 costmap 激活才接受目标） |
| 到达后停得不是理想抓取位 | **已自动处理**：驱动打印 `实测罐子 base 偏移 dx/dy`，先 `原地转向` 修朝向/横向，再 `前移` 修纵向，收敛到 dx≈0.30 才触发抓取；若仍失败请把终端 3 完整输出贴出来（能看到 dx/dy 具体数值，据此调 PARK 点） |
| grasp_node `process has died, exit -6` | **已修复（2026-09-03）**：`automatically_declare_parameters_from_overrides` 会预声明 CLI/launch 传入的参数，代码里再显式 `declare_parameter("object_pose"...)` 就抛 `ParameterAlreadyDeclaredException` → 未捕获 SIGABRT（之前手测 demo 不传这些参数所以没事）。现在所有 declare 都加 `has_parameter` 保护、整段纳入 try/catch，异常会打印原因而不是静默崩溃 |
| `Planning failed` | 看 `[grasp]` 日志里 `Failing stage(s)`：罐子被碰倒/滚走？桌子位姿方向错？停车太近/太远？ |
| 规划成功但执行时机械臂不动/很慢 | 确认 move_group `capabilities` 已含 ExecuteTaskSolutionCapability（launch 里已有）；慢是 MoveIt 速度缩放默认 0.1，可在 `pick_place_task.cpp` 给 sampling_planner 加 `setMaxVelocityScalingFactor(1.0)` |
| 执行完罐子没起来/掉了 | 物理夹持问题：夹爪位置控制 + 罐子 0.39kg。若夹不住 → 下一步上 effort 力控或调 close 位姿/摩擦 |
| grasp 子进程长时间不退 | 正常：整条 MTC 轨迹执行耗时可能数分钟（上限 GRASP_TIMEOUT=900s，会打印超时终止） |
| move_group 满屏 `Detected jump back in time` | sim 时钟跳变（一般发生在仿真被暂停/重启时）；抓取前让仿真稳定运行、不要中途 Ctrl-C |

## 可调参数

- 脚本顶部常量或命令行：`--park-*` `--can-*` `--table-*`、`--no-spawn`（罐子已在桌上时跳过 spawn）、
  `--skip-nav`（调试：机器人已手动开到操作点，只测 TF+抓取）
- MTC 参数仍集中在 `turtlebot3_manipulation_grasp/src/pick_place_task.cpp` 顶部
  （LIFT/APPROACH/GRASP_FRAME_LIFT）

## 备注

- 桌面上已有的 **bowl 现在也作为碰撞体加入场景**（bowl_pose/bowl_dims），MTC 会
  明确绕开它——之前"一头扎进碗里"就是碗没进规划场景。罐子已移到 (3.0, 2.18)，
  与碗相距 ~0.37m 进一步避让。
- 罐子用 `ros_gz_sim create` 运行时 spawn，**不改 example.world**；重启仿真后重新跑驱动即可复位。
- 接视觉后的下一步（原交接文档步骤 3）：识别出的 /map 位姿替换脚本里的
  `CAN_X/CAN_Y`（或改成从参数/话题读），其余链路不变。
