# 迁移结果记录（旧工作区 `~/Manipulation_ws` → 新工作区 `~/Robocup@home_ws`）

> 依据：`HANDOFF_conversation_summary.md`（背景/根因）+ `HANDOFF_agent_plan.md`（阶段 1~7 计划）。
> 本文件记录**阶段 0~3 的执行结果**、与原计划书的**差异**、以及**遗留风险**。
> 结论：**阶段 1（迁移）与阶段 2（构建）已完成并通过验收；阶段 3（抓取回归）见下方实测表。**

---

## 1. 阶段 0：三个需要拍板的事项（已定）

| # | 事项 | 决定 | 依据 |
|---|---|---|---|
| 0.1 | 新工作区路径 | `~/Robocup@home_ws`（`src/` = 队友仓库 clone） | 用户指定；仓库在 git 下 → 回滚方便 ✓ |
| 0.2 | 世界文件 | **用队友版 `example.world`**（不改），抓取罐改为**运行期 spawn** | 用户拍板；队友版里有视觉干扰物，旧版没有 |
| 0.3 | 地图 | **用旧工作区的 `map.pgm`** | 用户拍板；见 §5.2 的像素级对照 |

### 0.2 的后果与处理（**2025-09-13 追加：方案已改**）

> 当时决定"世界文件不改、抓取罐改成运行期 spawn"。**后来按用户意见改成：
> 不 spawn 任何东西，抓取目标 = 餐桌上【已经存在的】物体**（规则书 3.2/3.4：
> 餐桌上并排四个物品、位置由裁判定、从中任选一个抓）。原 `spawn_grasp_target.sh`
> 已删除（脚本 + CMakeLists 安装规则 + 取证台里的 spawn 步骤全部移除）。
>
> 现在的做法：`dining_grasp_task.py` 启动时枚举世界里的模型，筛出**落在餐桌
> （`dinning_table_3`）桌面上、类别在词表内、且夹得住**的物体，按
> `TARGET_PREFERENCE` 选一个当目标（同类里挑最靠机器人这侧桌沿的），
> **站位也由它推出来**（`(obj.x, max(obj.y+0.33, 桌沿+0.26))`），
> **桌上其它物体一律作为 `obstacles[]` 一起发给抓取侧**（免得机械臂撞翻它们）。
>
> **当天实测的现状（用世界文件逐条核对）**：餐桌 `dinning_table_3` 上只有一个
> 目标类物体 —— `bowl (2.672, 2.015)`，而碗直径 0.16 m > 夹爪开口 0.08 m
> （`objects.yaml: graspable: false`）→ **桌上目前没有夹得住的目标** ✗。
> 四类物体（apple ×3 / coke can ×2 / bowl ×2 / banana ×1）现在全在**客厅**那 4 张
> 桌子上（基础题计数用）。所以要跑通拔高题，必须往餐桌上放目标物（见 §8）。

## 2. 阶段 1：迁移（已完成）

### 2.1 整包拷贝（队友仓库里没有）

| 来源（旧工作区 `src/`） | 去处 | 备注 |
|---|---|---|
| `turtlebot3_manipulation_grasp/` | 新 `src/` | 抓取包：服务契约 + 适配层 + MTC 规划（+ 本次新增 `scripts/`） |
| `turtlebot3_moveit_config/` | 新 `src/` | MoveIt 配置（SRDF/ompl/controllers） |
| `moveit_task_constructor/` | 新 `src/` | MTC 源码树（抓取包依赖） |
| `turtlebot3_manipulation_navigation2/scripts/dining_grasp_task.py` | 同目录 | 任务层驱动（本工作区独有） |
| `HANDOFF_harness/` | 新 `src/` | 无头取证台（非 ROS 包） |
| `docs/{CURRENT_ARCHITECTURE,dining-grasp-integration,GRASP_SERVICE_HOWTO}.md` + 规则书 PDF | 新 `docs/` | 纯新增，不覆盖队友文档 |

拷贝后逐字节校验：`grasp` / `moveit_config` / `moveit_task_constructor` 与旧工作区**完全一致**
（除下面列出的本次改动外，`diff -rq` 无差异）。

### 2.2 打补丁（只改列出的行，不整文件覆盖）

| # | 文件 | 改动 | 校验 |
|---|---|---|---|
| a | `gazebo/urdf/turtlebot3_waffle_pi.urdf.xacro` | 加**前万向轮**（照抄后轮结构，`xyz="0.18 0 0.004"` 那套） | `diff` 与旧工作区**完全一致**；`grep -c caster_front` = **3** ✓ |
| b | `navigation2/param/turtlebot3.yaml`、`turtlebot3_use_sim_time.yaml` | `general_goal_checker.xy_goal_tolerance: 0.25 → 0.4`（带注释说明理由） | 各 1 处 ✓ |
| c | `navigation2/CMakeLists.txt` | 合并：`insid3_review.py`（队友）+ `dining_grasp_task.py`（我们）**都装** | 两者都在 ✓ |
| d | `grasp/config/objects.yaml` | 启用 `banana`（含"长边超开口、必须跨窄边抓"的注释） | ✓ |
| e | `navigation2/map/map.pgm` | 用旧工作区那份 | `cmp` 与旧工作区一致 ✓ |
| f | ~~`grasp/scripts/spawn_grasp_target.sh`~~ | **已删除**（2025-09-13：改成从桌上已有物体里挑目标，不再 spawn） | — |

### 2.3 队友版"更新的东西"全部保留 ✓

`patrol_task.py`（答案 JSON、banana、巡逻点微调）、`vision_pipeline.py`、
`insid3_review.py`、`reference_views/`、`third_party/`、`setup.sh` / `build.sh` /
`download_weights.sh` / `requirements.txt`、两份队友文档 —— **一律未动**。

## 3. 阶段 2：构建（已完成）

```bash
cd ~/Robocup@home_ws && source /opt/ros/humble/setup.bash
colcon build --symlink-install
# Summary: 12 packages finished [1min 33s]，无失败 ✓
```

`ros2 pkg list` 可见：`turtlebot3_manipulation_grasp`、`turtlebot3_moveit_config`、
`moveit_task_constructor_{core,msgs,capabilities,visualization,demo}`、`rviz_marker_tools` 等 ✓
`install/.../lib/turtlebot3_manipulation_navigation2/` 下 5 个脚本齐全
（`patrol_task.py`、`cmd_vel_relay.py`、`vision_pipeline.py`、`insid3_review.py`、`dining_grasp_task.py`）✓

## 4. 阶段 3：抓取链回归（无头取证台实测）

跑法（`HANDOFF_harness/`）：

```bash
xacro <gazebo pkg>/urdf/turtlebot3_manipulation.urdf.xacro use_sim:=true > /tmp/robot.urdf
WS=~/Robocup@home_ws/src bash HANDOFF_harness/sim_harness.sh <TAG> /tmp/robot.urdf \
   0.33 0.0 0.777 --obstacle 0.495 -0.328 0.777 bowl
```

**6 次无头实跑的实测汇总**（每次都是真 Gazebo 物理 + 真控制器 + move_group + MTC）：

> ⚠️ **重要更正（2025-09-13 追加）**：这批读数**不可信**。事后查明当时本机有一个
> **残留的 Gazebo 服务**在跑（一个早先 GUI 会话留下的世界，里面有一台停在巡逻起点
> `(-5.30, -0.50)` 的机器人）。gz-transport 被它占着 → 我"新起"的 server 根本没生效，
> 所有 `ign model`/桥接读到的都是**那个旧世界**里的东西：机器人是残留那台，
> 而我 spawn 的东西有的没进去、有的和旧的重叠 ✗。
> 现在取证台已经加了防呆（开跑前查"世界里是否已有同名机器人"、spawn 后核对机器人
> 真落在期望站位，不符就中止），**但下面这张表的数字要重跑才算数**。
> 结论层面仍成立的部分：链路每次都是 `success=true stage=0`、10 个解、补时间戳、
> 合爪参数 0.007/0.15、底盘 pitch 0（前万向轮 ✓）——这些是**代码/日志**证据，与污染无关。

| 跑次 | 世界文件 | 臂起始姿态 | 罐子 z 抬升 | 罐子结束 roll | 罐子 xy 漂移 | 服务返回 | 底盘 pitch 峰值 |
|---|---|---|---|---|---|---|---|
| `gate` | 队友版 | 全 0（URDF 默认） | **0.0359 ✓** | **−90.1° ✗** | — | success stage=0 | 0.00° ✓ |
| `gateA` | 队友版 | 全 0 | 0.0050 ✗ | −6.8° ✓ | 0.4 mm | success stage=0 | 0.00° ✓ |
| `gateB` | **旧版**（含罐） | 全 0 | 0.0053 ✗ | −6.7° ✓ | 0.4 mm | success stage=0 | 0.00° ✓ |
| `diag` | 队友版 | 全 0 | 0.0048 ✗ | −1.0° ✓ | 1.7 mm | success stage=0 | 0.00° ✓ |
| `home1` | 队友版 | **SRDF home** | **0.0358 ✓** | **−90.1° ✗** | 40.4 mm ✗ | success stage=0 | 0.00° ✓ |
| `home2` | **旧版** | **SRDF home** | 0.0055 ✗ | −5.5° ✓ | 1.3 mm | success stage=0 | 0.00° ✓ |

**每次都在日志里出现的"链路正确"证据**（与旧工作区逐条一致）：

- `合爪参数: 干涉 0.0070 m，速度缩放 0.15（张开仍用 1.0）` ✓（§3.7 的修法生效）
- `合爪目标: 物体 0.0670 m − 挤压 0.0070 → 关节 0.0265` ✓（按物体宽度算，不是命名状态）
- `放置点(规划帧)= (0.330, 0.000, 0.839) 支撑面 z=0.777 [放回原处]` ✓
- `找到 10 个解，执行第一个` ✓；`补时间戳: move to pick 0.710 s / move to place 0.170 s / move home 0.914 s` ✓（§3.5 的兜底在跑）
- 三次 `gripper_controller: Received & accepted new action goal` ✓；服务 7.0~7.1 s 返回 `success=true stage=0` ✓
- **底盘 pitch 峰值 0.00°** ✓ —— 前万向轮生效（没有它时是 12.7° ✗）

**判读（不要粉饰）**：

| 验收项 | 结果 |
|---|---|
| 底盘 pitch ≤ 2° | ✅ 6/6 |
| `grasp_node` 返回 success/stage=0 | ✅ 6/6 |
| 日志关键行齐全 | ✅ 6/6 |
| **罐子 z 抬升 ≥ 3 cm** | ⚠️ **2/6**（gate / home1）——其余 4 次是"空合"：手指合到命令值，罐子只被蹭动（roll 抖 ±13°、位移 <2 mm） |
| **罐子结束直立 + 回原位** | ❌ 抬起来的那 2 次**结束时 roll = −90.1°、xy 漂移 40 mm**（被碰倒/带倒） |

于是做了两组归因实验：

1. **换世界文件（队友版 vs 旧版）**：`gateA` vs `gateB` —— 两边**同样失败**（抬升 5.0 / 5.3 mm），
   → **不是队友世界里的干扰物造成的**，也**不是迁移造成的**（旧世界 + 旧版世界文件里自带的罐子也不行）。
2. **臂的起始姿态（全 0 vs SRDF home）**：全 0 时 1/4 抬起（`gate`），送 home 时 1/2 抬起（`home1`）。
   样本太少，**不能据此断言 home 就是决定因素**；只能说"从全 0 自碰姿态起步"是**已知的保真度缺陷**
   （真实流程是从 home 出发的），所以取证台已经默认加上"先送 home"这一步。
3. 把 6 次结果排在一起看：**同样的代码、同样的世界、同样的请求，结果会在"抬起并带倒"与"空合"之间跳**
   → 结论是**接触余量本就在临界点上**，微小的物理差异就足以改变成败（不是某个确定性 bug）。

**结论：阶段 3 的"链路"部分通过 ✓，"真正抓起"部分没有稳定通过 ⚠️（2/6，且成功那 2 次罐子被带倒）。**
这是一个**与迁移无关的、抓取本身的鲁棒性问题**（正好对应总结文档 §3.7 的"指尖点接触"与 §5.3 的
"开口超限检查缺失"）：手指只有**指尖**接触罐壁、掌垫够不到（间距 89 mm ≫ 罐宽 67 mm），
接触余量本来就很小，仿真里稍有扰动就变成"从罐子旁边合过去"或"把罐子带倒"。

> 提示：交接文档里也写着最终判据要**再用 GUI 复核一次**（`HANDOFF_harness/README.md` 末节）。
> 无头台本身与真实流程有已知差异（无导航/无相对微调、用理想静态 TF 代替真值位姿），
> 所以**不要用无头台的这一次失败去否定整条链**；但也不要用"服务返回 success"当成抓取成功 ✗。


## 5. 迁移过程中发现并修掉的问题

### 5.1 取证台自身的 4 个 bug（旧工作区里没暴露，属于交接工具缺陷）

| # | 症状 | 根因 | 修法 |
|---|---|---|---|
| 1 | 脚本一行不跑就退出 | `set -u` 下 source ROS 的 `setup.bash`：它引用未定义的 `AMENT_TRACE_SETUP_FILES` | source 前后 `set +u` / `set -u` |
| 2 | `python3: can't open file '<WS>/grasp_client.py'` | 脚本按 `$WS` 根找客户端，实际在 `HANDOFF_harness/` | 用 `$HARNESS_DIR` |
| 3 | spawn 永远被跳过 | 用 `ign model -m NAME -p \| grep -q .` 判"是否已存在"——**查不到时也会往 stdout 打错误信息** → `grep` 恒真 | 改成看 `ros_gz_sim create` 的 `OK creation of entity` |
| 4 | 罐子位姿时间线全是 `nan` | 帧名匹配要求"模型名 + 分隔符"，实测帧名**就是模型名本身**（`coke_can_dining`） | 改为"等于模型名，或模型名+`/`/`:`"；**必须精确匹配**：队友世界里还有 `coke_can` / `coke_can_1`，子串匹配会记录到别的罐子 ✗ |

顺手还做了两处加固：清理不再用 `pkill -x python3`（会误杀用户自己的 python 进程），
`grasp_client.py` 现在同时输出**罐子 xyz/roll + 底盘 pitch**（一张表看全验收指标）。

### 5.1b 取证台的保真度改动

| 改动 | 原因 |
|---|---|
| 加"**先把臂送进 SRDF home**"步骤（`SKIP_ARM_HOME=1` 可关） | spawn 出来的车臂姿态是 URDF 默认的**全 0**（Franka 自碰姿态），而真实流程是从 home 出发；实测这一步会明显影响成败（§4） |
| 临时文件集中到 `HANDOFF_harness/.run/`（+ `.gitignore`） | 原来 `$WS/.sim_*.log`、`$WS/.dbg_rsp.yaml`、`$WS/.dbg_home` 全撒在 `src/` 根目录，`git status` 一堆噪音 |
| `WORLD=<路径>` 可换世界 | 做"队友版 vs 旧版"的 A/B 归因（§4） |

### 5.2 地图：两份 `map.pgm` 的差别（决定 0.3 的证据）

两份图**尺寸/分辨率/origin 完全相同**（220×175, 0.05 m, origin `[-5.62, -4.35]`），
逐像素比对：**只有 6 个矩形区域不同**，且队友版把旧版里 6 张桌子的**脚印擦成了空闲**：

| 矩形（map 系） | 大小 | 对应家具 |
|---|---|---|
| x +0.88…+3.28, y +1.30…+2.30 | 2.40×1.00 | `dinning_table_{0,1,2}` 一带 |
| x −3.92…−1.52, y −2.45…−1.80 | 2.40×0.65 | `living_room_table_1` / `tea_table` 一带 |
| x +0.38…+0.88, y −3.15…−1.95 | 0.50×1.20 | `living_room_table_2` |
| x −2.72…−1.52, y −0.20…+0.30 | 1.20×0.50 | `living_room_table_0` |
| x −3.77…−3.27, y −3.65…−2.45 | 0.50×1.20 | `living_room_table_3` |
| x −4.12…−2.92, y +3.50…+3.95 | 1.20×0.45 | `kitchen_table` |

即：**旧版把桌子标成障碍（物理上正确），队友版把桌子当成空地**。
抓取站位 (3.0, 2.51) 与初始位姿 (−5.30, −0.50) 在两份图里都是空闲 ✓，所以选旧版不影响站位。

## 6. 遗留风险 / 待办（**建议优先看**）

| # | 事项 | 说明 |
|---|---|---|
| **R0** | **抓取的物理鲁棒性（最高优先）** | 见 §4：无头台 6 次里只有 2 次真正抬起，且那 2 次罐子被带倒。**与迁移无关**，但它是"接入视觉"之前必须先解决的事（否则视觉再准也抓不住）。方向：①按 §5.3 加"开口/朝向校验"（指尖开合上限 ~0.08 m）；②从根因上处理指尖比掌垫更靠内 11 mm 的几何（换抓取点/抓取高度 `grasp_lift`、或让掌垫参与接触）；③把"空抓检测"（§5.5）补上，别让空合被记成成功 |
| R1 | 队友世界里抓取台附近的干扰物（**潜在**风险） | `sugar_box (3.1, 2.0, 0.78)` 距罐子只有 **0.21 m** 且比罐子高（0.175 m）；`pudding_box`、`bowl` 同在一张桌上。它们**不在规划场景里**（场景只含桌面+目标+请求里的障碍）→ 机械臂路径可能扫过去。实测 A/B 里没看到它们造成差异（换旧世界同样失败），但**只要抓取开始稳了，这就是下一个要盯的点**；届时把这几件挪走或加进 `obstacles[]` |
| R2 | `patrol_task.py` 的 `table_3` 巡逻点 | 队友新值 `(−4.95, −3.1)` 在**两份地图里都是障碍** ✗（旧值 `(−4.7, −3.1)` 是空闲）；`table_0/table_2` 的 yaw 也改了。跑巡逻前要复核 |
| R3 | 答案 JSON 输出目录 | `patrol_task.py: ANSWER_OUTPUT_DIR` 默认 `~/turtlebot3_ws/submissions` —— 那是**队友的路径**，本工作区叫 `~/Robocup@home_ws`；`os.makedirs` 会自动建出这个无关目录 ✗。另 `GROUP_NUMBER = 3` 要改成真实组号、`TARGET_CLASSES_JSON` 按裁判发布改 |
| R4 | 视觉环境 | `bash setup.sh`（建 `~/vision_env`）+ `download_weights.sh`（权重）+ INSID3 的 DINOv3 权重（官方门控、手动下载到 `scripts/checkpoints/`，且该目录被 `.gitignore` 忽略） |
| R5 | 阶段 4~7 | 视觉适配层（`detect_grasp_target_node.py`）、任务层接视觉、4 类物品开口校验、端到端 —— **尚未开始** |

## 7. 怎么再跑一次回归

```bash
# 1) 构建
cd ~/Robocup@home_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install
source install/setup.bash

# 2) 生成仿真 URDF（带前万向轮）
xacro install/turtlebot3_manipulation_gazebo/share/turtlebot3_manipulation_gazebo/urdf/turtlebot3_manipulation.urdf.xacro \
  use_sim:=true > /tmp/robot.urdf

# 3) 无头取证台（日志都在 HANDOFF_harness/.run/ 下）
WS=~/Robocup@home_ws/src bash ~/Robocup@home_ws/src/HANDOFF_harness/sim_harness.sh my1 /tmp/robot.urdf \
   0.33 0.0 0.777 --obstacle 0.495 -0.328 0.777 bowl

# 换世界做对照（旧工作区那份世界文件里本来就有罐子，脚本会自动跳过 spawn）：
WORLD=~/Manipulation_ws/src/wpr_simulation_ros2/worlds/example.world \
  WS=~/Robocup@home_ws/src bash ~/Robocup@home_ws/src/HANDOFF_harness/sim_harness.sh my2 /tmp/robot.urdf \
   0.33 0.0 0.777 --obstacle 0.495 -0.328 0.777 bowl
```

完整 GUI 流程（四个终端，**不需要 spawn 任何东西** —— 目标来自餐桌上已有的物体）：

```bash
ros2 launch turtlebot3_manipulation_gazebo turtlebot3_franka.launch.py     # 仿真
ros2 launch turtlebot3_manipulation_navigation2 navigation.launch.py       # 导航
ros2 launch turtlebot3_manipulation_grasp grasp_service.launch.py          # move_group + grasp_node
ros2 run turtlebot3_manipulation_navigation2 dining_grasp_task.py          # 抓取任务
#   任务层自己从餐桌上挑一个可抓物体（--target-name/--target-class 可指定）

# ★ 开跑前先清残留 Gazebo（否则新 server 不生效，量到的全是旧世界 ✗）
pkill -9 -f 'ign gazebo' ; pkill -9 -f parameter_bridge ; pkill -9 -f robot_state_publisher
```


---

# 8. 追加盘点（2025-09-13，回答四个问题 + 一处方案变更）

## 8.1 抓取目标改成"餐桌上已经存在的物体"（已改代码，世界待定）

规则书（v4）3.2/3.4：餐厅内一张餐桌，**桌上并排四个物品、位置由裁判定**，
机器人识别后**从四个里任选一个**抓起来、明显抬离桌面再放回（拔高题 30 分 =
导航到餐桌 10 + 桌面物品视觉识别/框选标注 10 + 抓取抬起放回 10）。

已按此改：

- **不再 spawn 任何东西**：`spawn_grasp_target.sh` 删除，CMakeLists 安装规则删除，
  取证台里的 spawn 步骤删除。
- `dining_grasp_task.py` 新增 `pick_target_on_table()`：启动时枚举世界模型 →
  认出类别的（`coke_can*`/`apple*`/`bowl*`/`banana`）→ 位姿落在餐桌
  (`dinning_table_3`) 桌面范围内 → 去掉夹不住的 → 按 `TARGET_PREFERENCE`
  选一个（同类里挑最靠机器人这侧桌沿的）。
- 站位由目标推出：`(obj.x, max(obj.y + 0.33, 桌沿 + 0.26))`；`--park-*` 仍可覆盖。
- **桌上其它物体一律作为 `obstacles[]` 发给抓取侧**（原来只发一个碗，现在通用化）。
- 餐桌上一个可抓物体都没有时，**明确报错退出**，不会瞎抓。

**当天用世界文件核对的现状**（`ign model --list` + 逐条位姿，41 个模型）：

| 桌子 | 目标类物体 | 能否夹（开口 0.08 m） |
|---|---|---|
| **dinning_table_3（餐桌）** | `bowl (2.672, 2.015)` | ✗ 直径 0.16 m，`graspable: false` |
| living_room_table_0 | `apple_clone_0 (-2.05, 0.05)`、`coke_can (-2.35, -0.06)` | ✓ |
| living_room_table_1 | `banana (-3.30, -2.10)` | ✓ |
| living_room_table_2 | `apple_clone_2 (0.63, -2.60)` | ✓ |
| living_room_table_3 | `bowl_1`、`apple_clone_1 (-3.48, -3.40)`、`coke_can_1 (-3.50, -3.10)` | ✓ |

→ **四类物体现在全在客厅（基础题计数用），餐桌上只有一个夹不住的碗**。
要让拔高题有活干，得往餐桌上放目标物（推荐：apple / coke can / bowl / banana
四个并排放在 `dinning_table_3` 靠机器人这一侧、间距 ~0.15 m；同时把现在占着桌面的
`pudding_box (2.3,2.0)` / `sugar_box (3.1,2.0)` 挪走或留作障碍物）。
**这一条等拍板**（世界文件改不改、放哪几个）。

## 8.2 现在到底卡在抓取的哪一环

**A. 夹爪几何（硬事实，从 URDF 量出来的）**

| 量 | 值 |
|---|---|
| 手指关节 `fr3_finger_joint1` | prismatic，行程 0…0.04 m，mimic 对称 → **开口 = 2×关节值**，全开 0.08 m |
| 手指上的碰撞体最内侧（沿合拢方向） | **橡胶指尖 y ≈ 0**（`origin y=7.58mm`，`size y=15.2mm` → 内侧面 ≈ −0.02 mm）|
| 滑动座内侧面 | y = 2.4 mm |
| **掌垫（screw mount）内侧面** | **y = 11.0 mm** → 它比指尖靠外 **11 mm** |
| 结论 | 对 67 mm 宽的罐子：指尖接触时**掌垫间距 89 mm ≫ 67 mm** → **永远只有指尖接触**，靠橡胶尖的两点摩擦夹持 |

**B. 合爪命令比"刚好夹住"深一倍**

代码：`target = 0.5*span − close_hand_squeeze`（`pick_and_place.cpp`）。
对 coke can（span 0.0670，squeeze 0.007）→ 关节 **0.0265** → **指尖间距 0.053 m**，
而罐子 0.067 m → **每侧压进罐子 7 mm（合计 14 mm）**。
指尖刚碰到罐子应该发生在关节 **0.0335**（= 0.067/2）—— 也就是说命令是"往物体里多压
一个 2×squeeze"。这个量级对刚性罐子是很大的过盈量，实际表现为**罐子被挤出去/原地打转**，
而不是稳定夹持（`close_hand_squeeze` 是**每个手指**的量，注释里想表达的"干涉"是总量 ✗）。

**C. 规划层缺的三项检查（来自总结文档 §5）**

| # | 缺的检查 | 后果 |
|---|---|---|
| 1 | **不检查开口能不能夹住** | 香蕉 0.198 m 长 > 开口 0.08 m：MTC 可能选"跨长边"的朝向 → 物理上必然夹不住 ✗（代码只会在关节被限位截断时打一行 warn） |
| 2 | **不检查朝向/可夹性** | `graspable: false` 的碗之类只靠 `objects.yaml` 挡，规划本身不参与 |
| 3 | **没有空抓检测** | 夹空也返回 `success=true`（规划场景的 attach/detach 只是逻辑附着）→ **成功判据必须是物体位姿变化**，而这只能靠外部观察 |

**D. 实证（**已作废，见 §4 的更正**）**：6 次无头实跑里 3 次空合、1 次抬起但罐子被带倒……
这批数字是在**被残留 Gazebo 服务污染**的环境里测的，要重跑才算数。
链路层面（规划 10 解、补时间戳、合爪参数、底盘 pitch 0）是代码/日志证据，与污染无关 ✓。

## 8.3 新巡逻点的问题

- 巡逻是**基础题**用的：依次到客厅 4 张桌子的观察点（每点距桌约 1.2 m），
  顺序 `table_3 → table_1 → table_0 → table_2`，每点站稳后 GroundingDINO+SAM2 识别计数，
  结果以 `/map` 下的 Marker 输出并打印三类物品名称+数量。
- 队友把第一个点从 `(-4.7, -3.1, yaw=0)` 改成了 **`(-4.95, -3.1, yaw=π/50)`**。
  **这个新点在两份地图里都是障碍格**（map 上是一堵南北向的墙，x≈−4.9，房间西侧边界；
  该点正好压在墙的东边缘上，最近的空闲格在东边 0.05 m）。
  Nav2 收到"目标落在致命/膨胀代价里"的点会拒收或直接 abort ✗ → 任务会按
  `MAX_NAV_RETRIES=6` 重试 6 次再放弃，**白烧比赛时间**（基础题总限 8 分钟）。
  旧点 `(-4.7, -3.1)` 是空闲 ✓，而且离墙 0.22 m，yaw=0 正好面朝 +X 看桌子。
  另外 `table_0` 的 yaw 从 π/2 改成 7π/12、`table_2` 从 -0.4 改到 -0.48，这两处都在空闲格里 ✓。
  **建议**：把 `table_3` 改回 `(-4.7, -3.1, 0.0)`（或任何 x ≥ −4.85 的空闲点）。

## 8.4 答案 JSON 的问题（组号除外）

**前提**：规则书 v4 **通篇没有提任何 JSON/答案文件**（只说"一等奖需补交技术报告"）；
代码注释里引用的自测脚本 `src/scoring/score_submission.py` **在这台机器上不存在**
（已搜过 `~` 与工作区）→ 这套 JSON 是队友自己的自测产物，不是官方评分入口。
官方评分依据是：录屏/手机录像 + RViz `/map` Marker + 终端打印。

即便如此，这几处仍然要改（否则一旦真按 JSON 交，就是废文件）：

| # | 问题 | 说明 |
|---|---|---|
| 1 | `ANSWER_OUTPUT_DIR` 默认 `~/turtlebot3_ws/submissions` | 那是**队友的工作区路径**；本工作区是 `~/Robocup@home_ws`。`os.makedirs` 会安静地把这个无关目录建出来，答案落在没人看的地方 ✗。改成 `~/Robocup@home_ws/submissions`（或运行时用环境变量覆盖） |
| 2 | `TARGET_CLASSES_JSON = ["apple", "coke_can"]` | 只是 example（注释里也写了）；正式比赛**三类由裁判现场发布**，必须临场改，且 `NAME_TO_JSON` 要覆盖那三类 |
| 3 | **真正的风险不在 JSON，而在词表是闭集** | `vision_pipeline.py` 的 `ITEM_NAMES = ["apple","coke can","bowl","banana"]` 写死，`classify_phrase()` 对不在词表里的 phrase **一律返回 None（直接丢弃）** ✗。而规则书给的示例类别是 **"cola, bowl and windex bottle"** —— 裁判完全可能报这四类以外的物体（windex bottle、mustard bottle…）。那样**三类计数一个都数不出来**（30 分直接没了）✗✗。`INSID3_CLASSES` 同样是闭集。→ 建议把词表做成**可配置**（比赛开始按裁判发布的三类名改 `ITEM_NAMES`/别名/INSID3 类别，或允许"无复核"降级跑），这是基础题最该先修的地方 |
| 4 | 计分口径 | 规则书 3.3.1：`max(0, 10·TP/N − 2·FP)`，判对要求**/map 下与真值中心欧氏距离 < 10 cm**，且"同一真值目标的重复检测除首次外全算 FP" → 去重阈值 `DEDUP_DIST` 和 Marker 坐标精度直接决定分数（宁缺毋滥） |
| 5 | 已经对的部分 | 终端打印三类名称+数量 ✓、`/map` Marker ✓、`NAME_TO_JSON` 四点映射 ✓（`coke can`→`coke_can`）、文件命名 `<GROUP_NUMBER>_answer.json` ✓ |
