# 对话总结（RoboCup@home 仿真抓取 + 视觉接入）

> 本文是截至现在的完整交接记录。项目现状：**仿真里"导航到位 → 相对微调 → 规划 → 执行 → 真正抓起可乐罐"已跑通并验证**（用户在 GUI 里确认）。
> 目标：用队友视觉仓库做基座建新工作区，把本工作区的改动迁过去，并接入视觉。

---

## 1. 项目背景与目标

- **比赛形态：只比仿真**（Gazebo Fortress + ROS 2 Humble），机器人 = TurtleBot3 Waffle Pi 底盘 + Franka FR3 机械臂。
- **任务**：导航到固定站位 → 识别桌上物品 → 抓起其中一件 → **清楚抬起** → **放回原处**；另有巡逻计数任务与评分用的答案 JSON。
- 桌上物品类别（视觉侧 `ITEM_NAMES`）：`["apple", "coke can", "bowl", "banana"]`
  （注意：内部用**带空格**的 `"coke can"`，评分 JSON 用**下划线**的 `coke_can`，映射在 `patrol_task.py` 的 `NAME_TO_JSON` 里）。
- 固定餐桌场景：`dinning_table_3`，桌面顶 z≈0.777；罐子 `coke_can_dining` 在 world `(3.0, 2.18, 0.777)`；抓取站位 `(3.0, 2.51, yaw=-π/2)`。

## 2. 系统组成

| 包 | 作用 | 备注 |
|---|---|---|
| `turtlebot3_manipulation_gazebo` | 仿真启动、URDF/xacro、控制器配置 | **本工作区改过 URDF（加前万向轮）** |
| `turtlebot3_manipulation_navigation2` | 导航 launch、Nav2 param、任务脚本 | **本工作区改过 param + 新增 `dining_grasp_task.py`** |
| `turtlebot3_moveit_config` | MoveIt 配置（SRDF/ompl/controllers） | 队友仓库里**没有**，必须随迁 |
| `turtlebot3_manipulation_grasp` | **本项目的抓取包**：服务契约 + 适配层 + MTC 规划 | 队友仓库里**没有**，整包随迁 |
| `moveit_task_constructor` | MTC 源码（vendored） | 队友仓库里**没有**，抓取包依赖它，需随迁 |
| `wpr_simulation_ros2` | 世界文件 + 物体模型 | **两边的 `example.world` 不同，待定用哪份** |
| `franka_description` | 机械臂网格 | 两边**完全一致** ✓ |

### 抓取架构（解耦后的形态）

```
（视觉）→ DetectGraspTarget 服务 ──┐
                                  ├─→ dining_grasp_task.py（任务层/驱动）
（gz 真值，仿真替身）──────────────┘        · 导航到站位 + 相对闭环微调
                                            · 组 GraspTargetStamped(frame=base_footprint)
                                            ↓ /grasp_fixed_object 服务
                                    grasp_node（适配层）
                                            · 校验：新鲜度/查表/TF/支撑面/可达性
                                            · 建规划场景（桌面/目标/障碍）
                                            ↓
                                    pick_and_place.cpp（规划层, MTC）
                                            · 顶抓阶段图 + 执行
```

**接口契约（已冻结，视觉侧必须照此实现）**
- `msg/GraspTargetStamped.msg`：`header`、`class_id`、`confidence`、`point`
  - `point` 语义 = **物体中心轴 ∩ 支撑面** 的那个点（不是箱心、不是质心）
  - `class_id` 必须与 `objects.yaml` / `ITEM_NAMES` **逐字一致**（含空格）
  - **frame 用相机系或 `base_footprint`，不要用 `map`**（map 段含 AMCL 误差）
- `srv/DetectGraspTarget.srv`：请求 `string[] class_ids`，响应 `success/message/targets[]`（最优在前）
- `srv/GraspFixedObject.srv`：请求 `target` + `obstacles[]`，响应 `success/stage/message`（stage 0~7）

## 3. 已完成并验证的改动（按发现顺序）

### 3.1 `dining_grasp_task.py`（任务层驱动，改动最多）

| 改动 | 原因（实测证据） |
|---|---|
| `_gz_model_pose()` 解析修正 | `ign model -p` 输出的**第一组方括号是 `[home]`**（世界名），原正则把它当坐标 → `float("home")` 抛异常 → **每次都静默退回 map 常量、带定位误差** ✗。改为只收"能解析成 3 个数"的括号组 ✓ |
| 目标改为**相对量** | 抓取只需"物体在 `base_footprint` 下"的位置；用 gz 真值算 `p_obj − p_robot` 后按机器人姿态旋到本体（**当时只用 yaw，见 §5 遗留项**） |
| 停位几何诊断 | 打印机器人 map 位姿 + **停位误差** + 目标相对量 + 走 TF(map 链路) 的对照差（= 定位误差）+ 距臂基座距离 |
| **相对闭环微调 `creep_to_standoff()`** | Nav2 容差 0.25 + AMCL 误差 0.096 → 物理停位误差 **0.29 m** → 罐子距臂基座 **0.815 m**（臂展 0.855）→ MTC 在 `approach object` 无解 ✗。微调把物体开到 `base_footprint(0.33, 0)`±0.02（实测可规划几何 ✓），实测 **0.706 → 0.433 m** ✓；轴距 0.33 是"极坐标：方位→0、距离→0.33"` |
| `_spin_for()` 替代 `time.sleep` | `use_sim_time` 下时钟**只在 spin 时前进** ✗：微调 12 s 期间没 spin → 时间戳旧 **17.7 s** → 被抓取侧新鲜度校验拒收（`age=17.738s`）✗。实测验证：spin 等待 2 s → 时钟前进 1.98 s；`time.sleep` 2 s → 前进 0.00 s |
| 日志 `format` 参数修正 | 少一个占位符 → 把机器人 **z 打成了 "yaw"** ✗（曾误导排查） |
| `--no-creep` | 对比调试用 |
| `rel()` 只算 yaw（**遗留**） | 底盘前倾时相对换算会差 `sin(pitch)×距离`；**当前已被前万向轮从根因上消除**（见 3.5），但精确性上仍建议改完整 3D（§5） |

### 3.2 Nav2 参数（`param/turtlebot3.yaml` + `turtlebot3_use_sim_time.yaml`）

`general_goal_checker.xy_goal_tolerance: 0.25 → 0.4` ✓
- 理由：最后十几厘米的精度交给驱动的**相对微调**（不查 map、不受定位误差影响）；容差太紧时 DWB 会为消除定位误差反复磨蹭甚至超时 abort。
- `yaw_goal_tolerance` 保留 0.25；DWB 自己的 `xy_goal_tolerance` 未动。

### 3.3 `grasp_node.cpp`（适配层）

| 改动 | 原因 |
|---|---|
| 支撑面**顶面对齐目标点 z**（步骤 5b） | 配置桌面顶 0.78 比罐底真值 0.7774 高 2.6 mm ✗ → 物体碰撞箱埋进桌面 → MTC `attach object` 直接报 **"table colliding with object"（0/104）** ✗。契约上目标点本就落在支撑面上，两者必须自洽 ✓（`z_tolerance` 校验仍针对**配置值**做，粗差照样拦） |
| reach 判据按实测定 | `reach_max 0.85 → 0.75`（0.815 实测无解，快速拒收避免白等 6 s 规划）；`reach_comfort_max 0.55 → 0.65`（理想站位真实值 0.589~0.600，原值每轮误报） |

### 3.4 `pick_and_place.cpp`（MTC 规划层）

| 改动 | 原因 |
|---|---|
| `allow collision (object,support)` **提到 `attach` 之前** | 官方例程放在其后，因为官方场景物体不接触桌面；本项目物体坐在桌面上，`attach` 的整体状态校验经不起侵入 ✗ |
| **合爪目标按物体宽度算** | 原来用命名状态 `close`（=0.0 全闭）：真机/仿真里手指被物体挡住，位置误差 ≈ 物体半宽 ≫ `GripperActionController` 默认 `goal_tolerance 0.01` → 动作 abort ✗。改为 `min(width,depth)/2 − close_hand_squeeze`，并按手指关节 `position` 限位裁剪；宽度未知则退回命名状态 |
| `close_hand_squeeze: 0.007` | 见 3.6 的抓取失败根因 |
| `close_hand_speed_scaling: 0.15`（合爪单独用一个慢速插值规划器） | 同上 |
| `place_back_at_target: true`（默认） | 规则书是"抬起再放回"，不是搬到别处；`move to place` 由 8 点/0.666 s 缩到 **3 点/0.17 s**，也省比赛时间 |
| **子轨迹体检 `dumpSolution()`** | 执行层问题定位用：打印每条子轨迹的组/点数/起止时间/是否非严格递增 |
| **时间戳兜底 `fixTrajectoryTimes()`** | 见 3.5 的 MTC 零时间戳 bug |

### 3.5 MTC 采样轨迹"零时间戳"bug（执行层失败的真身）

**症状**：`stage=6 EXEC_FAILED`、错误码 99999；`arm_controller` 日志：
`Time between points 0 and 1 is not strictly increasing, it is 0.000000 and 0.000000`

**定位**（靠 `dumpSolution()`）：

| 子轨迹 | 组 | 点数 | 时间 |
|---|---|---|---|
| **move to pick**（Connect） | arm | 5 | **0.000→0.000** ✗ |
| approach / lift / lower / retreat（笛卡尔） | arm | 8/7/8/9 | 0.616 / 0.538 / 0.608 / 0.783 ✓ |
| close hand / open hand（MoveTo） | hand | 5 | 0.400 ✓ |
| **move to place / move home**（Connect） | arm | 6/11 | **0.000→0.000** ✗ |

**根因**：MTC 的 `PipelinePlanner` 用**字符串重载**构造 MoveIt `PlanningPipeline`（只传 `request_adapters` 参数名），本版本 MoveIt 不会因此挂上 **response adapters** → **采样规划（Connect 段）轨迹全零时间戳** ✗。
- 上游已知问题：moveit_task_constructor **#624 / #330**
- 已验证**无效**的解法：往 `ompl_planning.yaml` 加 `response_adapters`（本版本 MoveIt 库里连这个字面量都不存在，构造函数是 `vector` 重载）—— **该改动已撤回** ✓
- **采用的解法**：在 `execute()` 之前，对"时间戳非严格递增"的子轨迹复制一份、用 `TimeOptimalTrajectoryGeneration` 重算时间再放回（`fixTrajectoryTimes()`），缩放可配（默认与 `sampling_planner` 一致 =1.0）。
- 实测日志：`补时间戳: move to pick 0.710 s / move to place 0.666 s / move home 1.046 s` ✓ 之后臂控制器 5 次 `Goal reached, success!` ✓

### 3.6 底盘前倾（导致末端偏离 ~7 cm 的真正原因）

**用户观察**：机械臂一开始抓取，底盘就往前倒、后端前倾 → 位姿估计完全不准确。

**量化（从 URDF 算）**：

| 量 | 值 |
|---|---|
| 机械臂总重 / 底盘总重 | **19.60 kg / 1.55 kg**（臂是底盘的 12.6 倍 ✗） |
| 机械臂质心高度 | **0.874 m** ✗ |
| 支撑多边形（x 方向） | 驱动轮 x=0、万向轮**只在后方** x=−0.177 → 只有 `[−0.177, 0]` ✗ |
| 机械臂质心允许范围 | `[−0.186, +0.005]` → **前向余量仅 0.2 mm** ✗ |
| 能抓到桌面目标点的 57 个构型的机械臂质心 | `[−0.068, +0.153]` → **全部越界** ✗ |

**实测 pitch 时间线**：静止 −0.003 → 机械臂一动 **+0.222 rad（12.7°）**，且**回不来**（末值 +0.220）✗ —— 翻过去趴在底盘前缘上了。

**修法（不是补偿，是修正模型几何）**：在 `turtlebot3_waffle_pi.urdf.xacro` 新增**前万向轮** `caster_front_joint/link`，位置 `xyz="0.18 0.0 -0.004" rpy="-1.57 0 0"`，结构/接地高度与既有后万向轮完全一致。
- 允许的机械臂质心前移 → `+0.195`，比所需的 `+0.153` 留 42 mm 余量 ✓
- **实测效果**：pitch 序列 `-0.003 … -0.034 → -0.003`（**瞬态 2.0°、稳态 0.2°**）✓✓；底盘不再倾覆 ✓
- 对导航/规划的影响：**基本为零** ✓（被动关节无控制器；costmap 未配 footprint 多边形；SRDF 里本来就没有 caster/wheel 的 `disable_collisions`，而 MoveIt 只检查与被规划组相关的碰撞对 ✓）

### 3.7 抓取"空合/碰倒" → 成功（最后一块拼图）

**现象**：下落时没夹住，抬起/移动时把罐子碰倒。

**取证（30 Hz TF + 手指关节时间线）**：

| 阶段 | 手指 | 罐子位移 | 罐子姿态 |
|---|---|---|---|
| 合爪（干涉 0.002、0.4 s） | 合到命令值 0.0315（**未被挡住** ✗） | **被推走 1.6 cm** ✗ | 之后抬起时**被碰倒**（roll −90°）✗ |
| 合爪（干涉 0.007、约 1.4 s） | 合到命令值 0.0265 | **仅 1 mm** ✓ | **仍直立** ✓ |

**几何根因（从 URDF 逐个碰撞盒算出）**：`fr3_leftfinger` 的碰撞盒沿**合拢方向**最内侧在 `y=0`（指尖），而**掌垫**在 `y≈0.011` → **指尖比掌垫更靠内 11 mm** ✗ → 合爪**永远是"指尖点接触"** ✗（关节 0.0315 时掌垫间距 8.9 cm ≫ 罐宽 6.7 cm），加上 0.4 s 的**冲击式**合爪 → 罐子被挤出去 ✗。

**修法**：`close_hand_squeeze 0.002 → 0.007` + `close_hand_speed_scaling 0.15`（合爪单独用慢速规划器）→ **用户确认可以成功抓起可乐罐** ✓✓

## 4. 当前状态

**已验证可跑通（无头全仿真 + 用户 GUI 确认）**：
导航 → 相对微调（0.706→0.433 m）→ 场景（桌面/目标/障碍）→ MTC 规划（10 个解）→ 执行（臂 5 段 + 夹爪 3 次全部 accepted/reached）→ **物理抓起罐子** ✓

**关键验收指标（以后判断成败都用它）**：**罐子位姿是否真的变化**。
> 注意：`grasp_node` 返回的 `success=true` **只代表规划+执行链路成功** ✗ —— 规划场景里的 `attach/detach` 是**逻辑附着**，不代表真抓住 ✗。

## 5. 未完成 / 遗留项

| # | 项 | 说明 |
|---|---|---|
| 1 | **视觉接入（主要工作）** | 队友的 `vision_pipeline.py` 只是**库**：`VisionPipeline.detect(img_bgr)` → 2D 检测（phrase + mask，`_mask_center` 给的是**像素坐标**）；**没有服务、没有 3D 定位** ✗。缺的一环 = 像素 → 深度图取 3D → `camera_info` 内参 → TF 到 `base_footprint` → 换算成"轴心∩支撑面"点 → 填 `DetectGraspTarget` 响应 |
| 2 | 相对换算只用 yaw | `_measure_targets()` 的 `rel()` 只按 yaw 旋转。前万向轮已从根因上消除底盘倾覆 ✓，但更稳妥的是改完整 3D（`rpy_to_matrix` + `Rᵀ·Δp`，之前写过一次又被要求回档） |
| 3 | 4 类物品的"开口超限"检查缺失 | 指尖开合上限约 **0.08 m** ✗，而规划**不检查"开口能否夹住"** ✗。香蕉长 0.198 m / 宽 0.075 m ✓：若 MTC 选了跨长边的抓取朝向 → 物理上必然夹不住 ✗。建议加：按类指定**优先闭合朝向** + 开口校验 |
| 4 | `objects.yaml` 的 banana 行仍是注释 | 实测尺寸已算好：`height 0.0366 / depth 0.1984 / width 0.0750, grasp_lift 0.010, graspable: true` → 启用即可 |
| 5 | 空抓检测 | 目前**没有**任何环节能发现"手上没东西"（旧配置的 0.0 全闭也检测不了，那是反的 ✗）。可行方案：抬起后视觉复核（推荐）/ 夹爪力矩 / 读合爪后实际关节位置 |
| 6 | 视觉环境 | 队友新版把路径改成**仓库相对**（`third_party/sam2` ✓）、`groundingdino` 走 pip ✓、需 `setup.sh` 建 venv（`~/vision_env`，含 torch/sam2 + rclpy）✓；`insid3_review.py` **必须同目录安装** ✗（否则闭集复核静默降级、香蕉被误判成苹果 ✗） |

## 6. 无头仿真取证台（关键工具）

`.dbg_sim.sh` + `.dbg_call.py`（本工作区已清理，需要时按下面重建）：
1. `ign gazebo -s -r --headless-rendering <world>` + `robot_state_publisher`（用 xacro 展开的 URDF 生成 params.yaml）+ `ros_gz_sim create` 机器人到 `(3.0, 2.51, -1.5708)`
2. 桥 `/clock` ✓；加 `/world/home/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V` 可拿 **30 Hz 全连杆位姿** ✓
3. spawn 三个控制器：`joint_state_broadcaster` / `arm_controller` / `gripper_controller`（`--controller-type` 必须显式给 ✓）
4. 静态 TF `map→base_footprint`（放机器人真值位姿处）
5. `ros2 launch turtlebot3_manipulation_grasp grasp_service.launch.py use_sim_time:=true`
6. 用客户端脚本直接调 `/grasp_fixed_object`，同时**记录手指关节 + 罐子位姿**时间线

**踩过的坑（重建时必看）**：
- 必须 `export IGN_GAZEBO_RESOURCE_PATH=<gazebo_pkg>/models:<wpr>/models:<父目录>:<franka父目录>:/opt/ros/humble/lib` ✗（install 不会自动设 ✓）
- `HOME` / `ROS_LOG_DIR` / `ROS_HOME` 要指到可写目录（沙箱下 `~/.ros` 只读 → rclpy 直接崩）
- `joint_state` 初始姿态要用 **SRDF 的 `home`** ✓（全 0 是 Franka 自碰姿态 → 所有阶段归零 ✗）
- `ign model -p` 每个调用约 0.6 s ✗ → 高频取证要走上面那个 TF 桥 ✓
- **别把 `pkill -f "pattern"` 写在含该 pattern 原文的同一命令里** ✗（bracket 技巧在原文出现在自己命令行时会失效 → 自杀 ✓）

---

## 7. 迁移到新工作区（用队友仓库做基座）

队友仓库 `https://github.com/Jasmine-GYZ/turtlebot3_franka_public` 内容：`turtlebot3_manipulation_gazebo`、`turtlebot3_manipulation_navigation2`、`wpr_simulation_ros2`、`franka_description`、`third_party/{dinov3,INSID3,sam2}`、`docs`、`setup.sh`、`build.sh`、`download_weights.sh`、`requirements.txt`。

**他们新版里明显更新的东西（务必保留）**：
- `vision_pipeline.py`（`ITEM_NAMES` 增加 **banana** ✓；路径改为仓库相对 ✓）
- `insid3_review.py` + `reference_views/`（18 个文件）✓
- `patrol_task.py`：巡逻点位调整 ✓ + **评分答案 JSON 输出**（`GROUP_NUMBER`、`ANSWER_OUTPUT_DIR`、`NAME_TO_JSON`、`TARGET_CLASSES_JSON`）✓✓ 比赛关键
- `CMakeLists.txt`：多装 `insid3_review.py` ✓（但**删掉了 `dining_grasp_task.py`** ✗ → 合并时两份都要留 ✓）

**必须从本工作区搬过去的东西**：
1. `turtlebot3_manipulation_grasp/`（**整包**，他们那没有 ✓）
2. `turtlebot3_moveit_config/`（**整包**，他们那没有 ✓）
3. `moveit_task_constructor/`（**整源码树**，抓取包依赖 ✗ 大件，别漏 ✓）
4. `turtlebot3_manipulation_gazebo/urdf/turtlebot3_waffle_pi.urdf.xacro` 的**前万向轮补丁** ✓✓（漏了 → 底盘又倾 12.7°、抓取立刻打偏 ✗）
5. `turtlebot3_manipulation_navigation2/scripts/dining_grasp_task.py`（**本工作区独有** ✓）
6. 两个 param 文件里的 `xy_goal_tolerance: 0.4`（他们的是 0.25）✓
7. `CMakeLists.txt` 合并（保留 `dining_grasp_task.py` + 加 `insid3_review.py`）✓
8. `objects.yaml` 启用 banana 行 ✓

**待定（需要人拍板）**：
- `wpr_simulation_ros2/worlds/example.world` 两边不同 ✗ → 必须挑**含餐桌场景 + `coke_can_dining` 罐子**的那一份 ✓
- `map/map.pgm` 两边不同 ✗（本工作区还多 `map_backup.pgm` ✓）→ 用哪份地图
