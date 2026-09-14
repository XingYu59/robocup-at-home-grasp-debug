# 抓取链修复记录（2026-09-14）：抓取高度几何 + 对正桌子 + 彻底不用 gz 真值

本文只记"为什么之前抓不成、改了什么、怎么验证的"。三个问题的结论先放最前面。

---

## 一、为什么用了 gz 真值，MTC 的规划还是失败？

因为失败**根本不在位置**上，而在**抓取高度**上 —— 真值再准也救不了。

证据链（都可复现）：

1. 日志 `grasp pose IK (0/25): 0.261799 eef in collision: fr3_hand - object`
2. 查 MTC 源码 `moveit_task_constructor/core/src/stages/compute_ik.cpp`：
   `isTargetPoseCollidingInEEF()` 会把**手部组的碰撞体按目标位姿摆好**，在
   `allow collision (hand,object)` 生效**之前**先查一次碰撞；一旦相交就
   `spawn(failure); return;` —— **连 IK 都不试** ✗
   → 报错里的 `0/25` 不是"25 个 IK 都失败"，而是"25 个采样角**全被这一道碰撞检查拒掉**"。
3. 手部碰撞体的真实尺寸（从 `franka_hand.xacro` + `hand.stl` 实测）：

   | 部位 | 相对指尖平面（fr3_hand_tcp） |
   |---|---|
   | 掌面（fr3_hand 碰撞网格最低可及处） | **+37.4 mm** |
   | 指腹（橡胶指尖，唯一真正夹住物体的面） | **+9.0 mm ~ −9.5 mm** |

   ⇒ 物体顶面必须落在指尖平面上方 **0 ~ 37.4 mm** 之间：
   高了物体顶段插进手掌（`fr3_hand - object` ✗），低了指腹悬在物体上方（夹空 ✗）。
4. 老配置把 `grasp_lift` 当成"高度的 1/4"（`sugar_box`/`mustard_bottle` 都是 0.030），
   而这两个物体高 0.175/0.19 m —— 指尖平面停在**箱心上方 3 cm**，
   物体顶面高出指尖平面 **65 mm ≫ 37.4 mm** → 必然 0/25 ✗

离线复现工具：`HANDOFF_harness/check_grasp_geometry.py`
（把整只手按目标位姿摆好，对 24 个采样角逐个做精确碰撞检测；已用两个已知样本标定：
`coke can @0.040` → 12 个角可行 = 老工作区确实能规划 ✓；
`mustard_bottle @0.030` → **0 个角可行** = 与现场日志一致 ✓）

**修法**：`objects.yaml` 的 `grasp_lift` 全部按几何反推重算，取值 = 下式窗口的中点
（扁物体由"指尖别插进桌面"定下界，高物体由"物体顶面别顶到手掌"定下界）：

```
lo = max(h/2 − 37.4mm,  5mm + 9.5mm − h/2)
hi = h/2 − 9.0mm
grasp_lift = (lo + hi) / 2
```

| 类别 | 旧 | 新 | 可行采样角（24 个里） |
|---|---|---|---|
| sugar_box | 0.030 ✗ | **0.064** | 0 → 14 ✓ |
| mustard_bottle | 0.030 ✗ | **0.072** | 0 → 10 ✓ |
| cracker_box | 0.030 ✗ | **0.082** | 0 → 10 ✓ |
| bleach_cleanser | 0.030 ✗ | **0.102** | 0 → 6 ✓ |
| chips_can | 0.030 ✗ | **0.102** | 0 → 4 ✓ |
| apple / banana / gelatin_box | 0.020/0.010/0.010 | 0.013/0.003/0.003 | 4/2/2 ⚠（方形足迹对开口 0.08 天生紧） |
| coke can / tomato_soup_can / potted_meat_can | 0.040/0.025/0.020 | 不变 ✓ | 12/12/10 ✓ |

另外在 `pick_and_place.cpp` 加了一道**运行期自检**：每次抓取前打印
`抓取高度: "X" h=… → lift=…（可行窗口 [lo, hi] ✓）`，越界就 `RCLCPP_WARN`
并提示建议值 —— 以后谁改了高度，日志里立刻能看见。

## 二、观察点/站位：机械臂必须正对桌子

**旧实现**：接近方向 = "物体 → 机器人当前位姿"这个方向。它含 AMCL 定位误差，
也含观察位停位偏差 → 实测机器人停在 (2.6, 2.93)（观察位是 (2.7, 3.2)），
算出来站位 yaw = **−1.155 rad**，而桌沿法线是 **−1.571 rad** →
**机械臂斜 24° 伸到桌上**（日志/实测位姿都能对上：车停在 (2.843, 2.377) yaw=−1.156）。

**新实现**：观察位与站位**共用同一条"桌沿法线"**：

* `approach_normal()`：4 条桌沿法线（map 的 ±x/±y）里挑一条
  —— 先按"物体→机器人"的方位排序（少绕路），再用**导航地图**逐条筛"站得下人" ✓
  （只看方位是不够的：机器人从起点出发时物体在"西边"，而西侧桌沿外是
   dining_table_1 的桌面 → 观察位曾被算到 (1.55, 2.0)**桌子里** ✗ 实测踩过）
* 观察位 = 桌心 + 1.15 m × 法线，朝桌心；站位 = 物体 + g × 法线，朝物体
* 终点校验：站位必须在桌沿外 EDGE_CLEARANCE(0.26 m)、且地图上是空闲空间

实测日志：
```
桌沿法线 = (+0.000, +1.000) → 机械臂正对桌子（yaw = -1.571）
② 抓取站位 = (2.993, 2.510, yaw=-1.571)；目标在 map(2.993, 1.960)
```
车头垂直桌沿、物体在正前方、位姿与定位误差无关 ✓

## 三、gz 真值：已全部删除

用户要求（包括调试）不许读 gz 真值 —— 理由是它会让"视觉到底行不行"这个结论失效。
删除清单：

| 位置 | 删掉的东西 |
|---|---|
| `grasp_phase.py` | `truth_targets()` / `_gz_model_names()` / `_gz_model_pose()` / `_pose_cache` / `MODEL_PREFIX_TO_CLASS` / `class_of_model()` / `TRUTH_POSE_TTL` / `use_truth_fallback` / `fetch_targets(force_truth=…)` / creep 与主流程里的真值退路 |
| `patrol_task.py` | `grasp_truth_fallback`、`--no-truth-fallback` |
| `dining_grasp_task.py` | `--truth-only`、`--no-vision`、`--vision-only` |
| `detect_grasp_target_node.py` | `_truth_delta()`、`debug_truth` 参数 |
| `grasp_service.launch.py` | `debug_truth:=true` |
| `HANDOFF_harness/` | `sim_harness.sh`、`grasp_client.py`（整套"读真值判成败"的取证台）|

现在整条链只有视觉 + TF + 导航：`grep -rn "ign model\|gz model\|truth"` 只剩注释说明。

---

## 四、修完之后的实测结果（真仿真、真导航、零真值）

`dining_grasp_task.py`（独立抓取入口）一次跑通：

```
① 观察位由桌沿法线算出 (2.700, 3.150, yaw=-1.571)
② 抓取站位 (2.993, 2.510, yaw=-1.571)   ← 正对桌子
   相对微调 67 轮收敛：物体 base(+0.336, +0.000)
   抓取高度: "chips_can" h=0.250 → lift=0.1020（可行窗口 [0.0876, 0.1160] ✓）
   找到 10 个解，执行第一个 → 抓取结束: success=true stage=0
✅ Phase 2 完成：抓取成功
```

即：**规划不再 0/25，且整条 pick&place 执行成功**（这是本工作区第一次跑到 stage=0）。

### ⚠️ 但"抓起来了"≠"真夹住了"：现在的瓶颈已经转到视觉标签

`success=true stage=0` 只说明**轨迹都执行完了**（attach/detach 是逻辑附着）。
这一轮的目标被标成 `chips_can`（碰撞箱 0.25 m 高、抓取高度 0.102），
而它实际位置 (2.186, 2.017) 上是世界文件里的 **pudding_box（只有 0.035 m 高）**
→ 手指是在物体**上方 10 cm 的空气里**合拢的 ✗。

世界文件里 dining_table_3 上只有三个物体：
`bowl(2.672,2.015)`、`pudding_box(2.300,2.000)`、`sugar_box(3.100,2.000)`，
其中**只有 sugar_box 夹得住**（另两个窄边 0.16/0.089 > 开口 0.08）。
而开集检测给出的标签是 `mustard_bottle / tomato_soup_can / cracker_box / chips_can /
tuna_fish_can / apple / sugar_box`，置信度 0.3~0.6，**标签逐帧还会变**
（同一个糖盒这帧叫 mustard_bottle、下帧叫 tomato_soup_can），位置误差 0.1~1.0 m。

已经为此做的稳健化（都在本轮内验证过）：

* 开集标签不稳 → creep 不再"按类别名找目标"，改成**按位置跟踪**
  （找不到该类别时用离上次跟踪位置最近的那个检测，日志里如实打印替代标签）
* `measure()` 的里程计跟踪有效期 30 s → **600 s**（一次视觉调用就要 25~30 s，
  30 s 的基准等于没有 → creep 每轮都去重新调视觉、又认不出 → 直接放弃微调 ✗
  实测连续 4 轮都卡在这里；改成 600 s 后 creep 67 轮稳定收敛 ✓）
* 支撑面校验：容差 0.20 m 会让 table_1 东端的 chips_can(1.9,2.0) 混进来 →
  现在**显式排掉邻居餐桌（table_1/table_2）自己的脚印**
* 观察位太远时开集置信度只有 0.30~0.47（低于阈值全被拒）→ 增加"**近看位**"
  （离桌心 0.68 m，阈值放宽到 0.25）
* 抓取模式的**窄词表复核**上限 `len(voted) <= 6` → 放宽到 12：
  实测经常投出 7~8 个候选导致复核被**跳过**，拿到的正是最差的那批标签/位置
  （复核跑上之后同一批目标置信度 0.35~0.46 → 0.39~0.59）

**下一步（视觉侧，建议优先）**：把"开集标签"换掉 —— 桌上物体应当是**闭集**的
（规则书当天发四类），用 INSID3 + DINOv3 对**每个 mask**做闭集打分并取最大值，
而不是靠 GroundingDINO 的自由文本 phrase；同时用 mask 的像素高度 + 深度反推
**物体高度**，与目录表尺寸做一致性校验（现在是只校横向尺寸）。
`vision_pipeline.py` / `insid3_review.py` 是队友文件，改之前先跟他们确认。

---

## 五、改动文件清单（本轮）

| 文件 | 改了什么 |
|---|---|
| `turtlebot3_manipulation_grasp/config/objects.yaml` | **全部 grasp_lift 按手部几何重算** + 把几何约束写成注释 |
| `turtlebot3_manipulation_grasp/src/pick_and_place.cpp` | 抓取高度运行期自检（可选：未配置时按几何自算） |
| `turtlebot3_manipulation_grasp/scripts/detect_grasp_target_node.py` | 删 `_truth_delta`/`debug_truth`；窄词表复核上限 6→12 |
| `turtlebot3_manipulation_grasp/launch/grasp_service.launch.py` | 删 `debug_truth` |
| `turtlebot3_manipulation_navigation2/scripts/grasp_phase.py` | 桌沿法线（观察位/站位同源）+ 导航地图空闲校验 + 删全部真值 + creep 跟踪 600 s + 邻居桌排除 + 近看位 |
| `turtlebot3_manipulation_navigation2/scripts/patrol_task.py` | 删真值退路开关；观察位默认交给抓取侧算 |
| `turtlebot3_manipulation_navigation2/scripts/dining_grasp_task.py` | 删真值入口；加 `--keep-pose` / `--observation` |
| `HANDOFF_harness/check_grasp_geometry.py` | **新增**：顶抓几何自检（改高度前后必跑） |
| `HANDOFF_harness/README.md` | 记下删掉了哪套真值取证台、以及替代的跑法 |
