# 视觉 ↔ 抓取 接口说明（本次新增：视觉适配层）

> 背景：旧工作区是"抓固定物体"，抓取侧从 gz 真值直接算目标；这次要接视觉，
> 所以需要一层**把视觉检测结果翻译成抓取输入**的接口。
> 好消息：**契约在迁移时就已经带过来了**（是旧工作区里冻结好的），本次缺的只是
> **实现**。本文件记录契约、新实现、以及实现过程中发现的一个关键几何问题。

---

## 1. 三份冻结契约（在 `turtlebot3_manipulation_grasp/` 里，本次**未改**）

| 文件 | 谁提供 | 谁调用 | 作用 |
|---|---|---|---|
| `msg/GraspTargetStamped.msg` | —— | —— | 目标数据结构：`header`(帧+时刻) / `class_id` / `confidence` / `point` |
| `srv/DetectGraspTarget.srv` | **视觉侧节点**（本次新增的实现） | 编排节点 `dining_grasp_task.py` | 按需检测：请求 `class_ids[]`，响应 `success/message/targets[]`（按置信度降序） |
| `srv/GraspFixedObject.srv` | `grasp_node`（已有） | 编排节点 | 触发一次抓取：`target` + `obstacles[]` → `success/stage/message` |

**语义要点（照抄契约，两边必须一致）**

1. `header.frame_id` 必须是**相对底盘**的帧（`base_footprint` 或相机光学帧），**不要用 `map`**
   —— map 那段 TF 含 AMCL 定位误差，等于把定位误差搬进抓取。
2. `point` = **物体中心轴 ∩ 支撑面**（z = 支撑面高度），**不是**"相机看到的那一点"。
   抓取侧据此算箱心：`box_center = point + (0,0,box_height/2)`（box 查 objects.yaml）。
3. `class_id` = `vision_pipeline.ITEM_NAMES` 的**原样字符串**（含空格，如 `"coke can"`），
   抓取侧 `objects.yaml` 的 key 必须逐字相同。
4. `targets[]` 按置信度降序：规则书是"桌上四个物品任选一个抓"，里面可能夹着夹不住的
   （如 `bowl`）→ 调用方按顺序挑，失败了换下一个；**剩下的项正好当 `obstacles[]` 传下去**。
5. `stamp` = 本次检测时刻（抓取侧会做新鲜度校验，>10 s 拒收）。

## 2. 本次实现：`turtlebot3_manipulation_grasp/scripts/detect_grasp_target_node.py`

数据流：

```
/camera/image_raw ─┐
/camera/depth/image_raw ─┼→ 掩码 + 内参 → 相机系点云 → TF → base_footprint → 契约点
/camera/camera_info ─┘      ↑
                    vision_pipeline.VisionPipeline().detect()  ← 队友的代码，原样调用
```

| 实现要点 | 说明 |
|---|---|
| 复用队友的视觉 | 只调 `VisionPipeline().detect()` 与 `classify_phrase()`；**视觉侧代码一行没改** ✓ |
| 依赖哪些话题 | `/camera/image_raw`、`/camera/depth/image_raw`、`/camera/camera_info`（launch 已把三者 frame_id 统一成 `camera_rgb_optical_frame`，像素一一对应 ✓） |
| 深度解码 | 手动按 encoding 解（`32FC1` 浮点米 / `16UC1` 毫米），不用 `cv_bridge`（numpy 2.x ABI 不兼容） |
| 支撑面高度 | **自己读 `grasp_params.yaml` 的 `support_surface`**（pose.z + thickness/2 = 0.780），与抓取侧同一个数，不另写常量 ✓ |
| 类别尺寸 | 读同包的 `objects.yaml`（拿"窄边/2"做下面的后退补偿） |
| 服务名 / 类型 | `/detect_grasp_target`，`turtlebot3_manipulation_grasp/srv/DetectGraspTarget` |
| 运行环境 | **必须在视觉 venv 里**：`source ~/venvs/vision_env/bin/activate`（torch/groundingdino/sam2）。忘了激活会打一行明确错误，抓取服务照常可用 |
| 自测 | `ros2 run turtlebot3_manipulation_grasp detect_grasp_target_node.py --self-test` |
| 调试对照 | 仿真里每检测到一个目标，额外打一行"**视觉 vs gz 真值**"的误差（mm），用来量化视觉精度 ✓ |
| 线程模型 | `MultiThreadedExecutor`（服务回调里要等相机数据，订阅回调必须在别的线程继续跑，否则死锁 ✗） |

## 3. ⚠ 实现中发现的关键几何问题（契约里那句做法在本机位**不成立**）

契约写的是：*"由 camera_info + 掩码质心像素得到视线，与该支撑面（z = 支撑面高度）求交"*。
那是**俯视相机**（腕部/高位）的做法。本机器人的相机装在**`z = 0.80 m`**（`camera_joint` 在 waffle URDF 里），
而桌面顶是 **0.78 m** —— 相机几乎与桌面等高：

| 量 | 值 |
|---|---|
| 相机光学帧位置（base_footprint） | `(0.076, 0.000, 0.809)` |
| 物体中心（桌面 + 罐高/2 = 0.84）相对相机的仰角（站位 0.33 m） | **+6.9°（向上）** |
| 物体底面（0.78）相对相机的仰角 | −3.5° |

→ 穿过物体**中心**的那条视线是**水平甚至略微向上**的，与支撑面**没有前向交点** ✗
（强行求交会得到相机后方或无穷远的点）。

**采用的等效算法**（意图不变：要"轴心∩支撑面"，不要"可见面那一点"）：

```
可见表面点云质心(base)  +  沿【水平视线方向】后退 (物体窄边 / 2)  →  z 取支撑面高度
```

理由：相机从水平方向看物体时，可见表面本来就比轴心**近半个窄边**，沿视线退回来正好落在轴心上；
对圆柱（可乐罐）和方盒（cracker_box 之类）都成立。

**单元测试（合成数据，已跑）**：把可乐罐放在 base(0.33, 0)，相机位姿按 URDF 取 `(0.076,0,0.809)`
+ 光学系旋转，点云加 2 mm 噪声 → 换算结果 `(0.3307, 0.0002, 0.7800)`，
真值 `(0.3300, 0.0000, 0.7800)`，**水平误差 0.7 mm** ✓，z 严格等于支撑面 ✓。

> 相机规格（`turtlebot3_waffle_pi.gazebo.xacro`）：`rgbd_camera` 640×480，
> 水平 FOV 1.0856 rad（62.2°）→ 垂直 ±24.3°。站位 0.33 m 时物体底面 −3.5°、顶面 +15.7°
> → **整只物体都在视野内** ✓（这个站位既能抓也能看，不用另找观察位）。

## 4. 怎么用

```bash
# 0) 构建（一次）
cd ~/Robocup@home_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install
source install/setup.bash

# 1) 仿真 + 导航（两个终端，保持原来的用法）
ros2 launch turtlebot3_manipulation_gazebo turtlebot3_franka.launch.py
ros2 launch turtlebot3_manipulation_navigation2 navigation.launch.py

# 2) 抓取服务 + 视觉适配层（⚠ 必须先激活视觉 venv，否则适配层起不来）
source ~/venvs/vision_env/bin/activate
ros2 launch turtlebot3_manipulation_grasp grasp_service.launch.py          # enable_vision:=true（默认）
#   没装视觉环境 / 只想测抓取：  ... grasp_service.launch.py enable_vision:=false

# 3) 直接问视觉（机器人停稳后）
ros2 service call /detect_grasp_target turtlebot3_manipulation_grasp/srv/DetectGraspTarget \
  "{class_ids: ['coke can']}"

# 4) 不用编排节点、单独自测一次
ros2 run turtlebot3_manipulation_grasp detect_grasp_target_node.py --self-test
```

**已验证**（离线，无仿真）：参数读取（支撑面 0.780 / objects.yaml 4 类）、服务创建、
权重加载路径、以及没有相机数据时的**优雅失败**（6 s 内返回 `no image stream: RGB/深度/内参`，不崩）✓
**未验证**：真仿真里的检测精度与端到端抓取 —— 需要把仿真+导航起起来（你可以直接跑第 3 步）。

## 5. 编排已接上（阶段 5，已实现）

**结构（一份实现、两个入口）**：

```
scripts/grasp_phase.py（新增，唯一实现）
   ├── patrol_task.py       比赛主流程：Phase 1（巡逻+计数+写答案 JSON）跑完 → 自动接 Phase 2
   └── dining_grasp_task.py 独立调试入口（薄壳，CLI 与旧版兼容）
```

`patrol_task.py` 的改动很小：`_all_done()` 写完答案 JSON 之后调 `_run_grasp_phase()`；
`main()` 换成 `MultiThreadedExecutor`（抓取阶段在轮询等 future，回调必须在别的线程继续跑）；
`_wait_for_nav2()` 不再用 `rclpy.spin_once`（会把节点挂到全局 executor 上，和上面的多线程冲突 ✗）。

**跑法**：

**谁需要视觉虚拟环境（重要，别搞混）**

| 终端 | 跑什么 | 要 venv 吗 | 原因 |
|---|---|---|---|
| 1 | 仿真 `turtlebot3_franka.launch.py` | ✗ | 纯 ROS/Gazebo |
| 2 | 导航 `navigation.launch.py` | ✗ | Nav2/AMCL |
| 3 | `grasp_service.launch.py` | **✗（默认）** | launch 会**显式用 `~/venvs/vision_env/bin/python` 启动适配层**（可用环境变量 `VISION_PYTHON` 覆盖）；venv 不在或脚本没装时才退回普通 Node，那时才需要手动 `source`。`enable_vision:=false` 就完全不需要 |
| 4 | `patrol_task.py` | **✓ 必须** | 它在**模块顶层** `import vision_pipeline`（基础题计数用），那个文件要 torch/groundingdino/sam2 |
| 调试 | `dining_grasp_task.py` | ✗ | 只 import rclpy + grasp_phase，不碰视觉库；视觉走服务（那个进程在终端 3）✓ |

```bash
# 终端 1/2：仿真 + 导航（照旧，系统 python）
ros2 launch turtlebot3_manipulation_gazebo turtlebot3_franka.launch.py
ros2 launch turtlebot3_manipulation_navigation2 navigation.launch.py
# 终端 3：抓取服务 + 视觉适配层（不必 source；适配层由 venv 解释器显式启动）
ros2 launch turtlebot3_manipulation_grasp grasp_service.launch.py
# 终端 4：主流程（Phase 1 巡逻计数 → Phase 2 抓取）—— ★ 必须先激活视觉 venv
source ~/venvs/vision_env/bin/activate
ros2 run turtlebot3_manipulation_navigation2 patrol_task.py
#   只做基础题：            ... patrol_task.py --no-grasp
#   只抓某几类：            ... patrol_task.py --grasp-classes "coke can"
#   换观察位：              ... patrol_task.py --grasp-observation 2.7,3.2,-1.5708
# 想单独调抓取（不跑巡逻）：
ros2 run turtlebot3_manipulation_navigation2 dining_grasp_task.py [--truth-only] [--classes "coke can"]
```

**Phase 2 内部顺序**（细节见 `docs/HANDOFF_grasp_orchestration_design.md`）：
观察位看整桌 → 选一个"可夹 + 可达"的目标（按优先级档位）→ 算站位 → 导航 → **站位上重测** →
creep 相对闭环（方位容差已收紧到 0.013 rad ≈ ±4.3 mm，且要连续 3 轮达标）→
`/grasp_fixed_object`（target + 其余物品当 obstacles）→ `BAD_TARGET/NO_SOLUTION` 就换下一个目标。

**评分证据**：适配层每次检测都会把"框 + 类别 + 解算出的轴心点"画到相机原图上，
发到 `/detect_grasp_target/annotated_image` 并存盘到 `~/turtlebot3_detections/detect_<stamp>.png`
（对应拔高题里"框出拟抓取目标、标注物品名称"那 10 分）✓

**视觉侧已验证（2026-09-13，装上 INSID3 权重后）**：

| 项 | 结果 |
|---|---|
| INSID3 闭集复核器 | **加载成功 ✓**（权重 `scripts/checkpoints/dinov3_vitb16_pretrain_lvd1689m-73cec8be.pth`，327 MB，用户提供）|
| 误判纠正 | `apple/front.png` 从 **`coke can` 0.602 ✗** 变成 **`apple` 0.602 ✓** |
| 干扰物剔除 | `mustard_bottle`/`windex_bottle`/`cracker_box`/`sugar_box` 参考图**全部被剔除** ✓；`potted_meat_can` 漏一个 0.31 假阳性 → 被适配层 `min_confidence=0.5` 挡掉 ✓ |
| 四类目标 | `coke can` 0.912 / `apple` 0.602 / `banana` 0.680 / `bowl` 0.669，标签全对 ✓ |
| ⚠ 构造耗时 | **无复核 3.8 s → 有复核 77.3 s**（DINOv3 骨干 + 原型预计算）→ 已在适配层与 `patrol_task` 加**后台预热**（与导航并行），避免到点后干等 |
| 单帧推理 | ~4.7~5.0 s/帧（含复核）|

**已验证（离线）**：三个脚本语法/导入 OK；`grasp_phase` 的选择与站位解算过了单元测试
（跳过夹不住的 bowl、只放方盒时也能选、站位复现旧工作区验证过的 `(3.0, 2.51, −90°)`）；
`patrol_task` 在视觉 venv 里可导入；两个入口的 CLI 正常；无仿真时优雅失败不崩 ✓
**未验证**：真仿真里的端到端（需要你把三个终端起起来跑一次）。


---

## 6. 抓取模式：跳过 INSID3 闭集复核、按抓取集的定义识别（2026-09-13 追加）

**背景**：计数任务的词表只有 4 类（`apple/coke can/bowl/banana`），而餐桌上的可抓物体
（如 `sugar_box`）不在里面 → 默认路径在这张桌上只能认出夹不住的碗 ✗

**做法（不改队友代码）**：`detect()` 是逐帧读 `self.text_prompt`、并按 `self.reviewer is not None`
决定要不要复核 ✓，所以适配层只改**实例属性**：

| 步 | 动作 | 实测效果 |
|---|---|---|
| 1 | 先跑队友默认（4 类 + 复核） | `sugar_box` 图 → 0 框 ✓（被复核正确剔除）|
| 2 | 结果里没有【可抓类别】→ 抓取模式：换成 objects.yaml 的类别 prompt + `reviewer=None` | `sugar box` 0.42 ✓（长词表置信度低、有假阳性）|
| 3 | **窄词表复核**：用第 2 步认出的候选类组成短 prompt 再检一次 | `sugar box` **0.85 排第一** ✓（cracker_box 那张从"排错位"变成 0.76 排第一 ✓）|

★ 换 prompt 时**必须同时重算分词缓存** `self._input_ids` ✗
（`detect()` 的短语解码读的就是它；不重算会拿旧词表的 token 解码新词表 → 出现
"只填 sugar box 却报 apple"这种鬼结果，实测踩过）

**跳过复核的代价（实测，必须知道）**：

| 风险 | 证据 | 缓解（已做/建议） |
|---|---|---|
| **标签会自信地错** | 复核关掉后 `sugar_box` 图 → `coke can` **0.75** ✗ | 窄词表复核（已做 ✓）；`min_confidence` ✓ |
| **干扰物不再被剔除** | 复核的作用就是判掉 14 个干扰类（实测 mustard/windex/cracker/sugar 全被剔 ✓）| 编排层只挑 `graspable` 的类 ✓ + 可达性过滤 ✓ |
| 长词表置信度塌、假阳性多 | 18 类 0.42 vs 5 类 0.79 | 两阶段召回-复核（已做 ✓）；调用方指定类别时直接用窄词表 ✓ |
| 尺寸接近的类会互相误标 | `cracker_box` ↔ `tuna_fish_can` 同时出现 | 尺寸核对只能拦"量级差得远"的（碗↔罐 ✓），同类尺寸的拦不住 ✗ → 建议加**多帧投票** |
