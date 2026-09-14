# Agent 执行计划：迁移到新工作区 + 接入视觉

> 配套文档：`HANDOFF_conversation_summary.md`（背景、已做改动、根因与陷阱清单）。
> 本计划假设：**新工作区用队友仓库做基座**（`https://github.com/Jasmine-GYZ/turtlebot3_franka_public`），
> 把**本工作区已验证的抓取链**迁过去，然后接入视觉。

---

## 0. 前置条件（需要人先拍板）

| # | 事项 | 说明 |
|---|---|---|
| 0.1 | **新工作区路径** | 形如 `~/xxx_ws`，里面 `src/` 已经放好队友仓库内容。**不要把克隆放进任何 `src/` 下** ✗（同名包会让 colcon 因重复包名直接失败） |
| 0.2 | **世界文件用哪份** | 两边 `wpr_simulation_ros2/worlds/example.world` **不同** ✗。必须挑**含餐桌场景 + `coke_can_dining` 罐子**的那份（判据：`grep -c coke_can_dining <world>` 且能看到 `dinning_table_3`）。**搞错这份 → 桌面场景/罐子就没了，抓取无从谈起** |
| 0.3 | **地图用哪份** | `turtlebot3_manipulation_navigation2/map/map.pgm` 两边不同 ✗（本工作区还多一份 `map_backup.pgm`）。按比赛场地确认 |
| 0.4 | 权限 | 若执行 agent 的文件权限不含新工作区，则本计划需以**脚本形式**交付、由人执行；或逐条申请写权限 |

## 阶段 1：迁移（先拷贝，后打补丁）

**原则：队友仓库里"更新的部分"一律保留 ✓；只把本工作区的抓取链与修好的物理模型搬过去 ✓。**
动手前把新工作区的 `src/` 备份一份（或确认它在 git 下），便于回滚。

### 1.1 整包拷贝（队友仓库里**没有**，直接拷）

```bash
SRC=<本工作区>/src            # 当前这个工作区的 src
DST=<新工作区>/src
cp -a "$SRC/turtlebot3_manipulation_grasp"  "$DST/"
cp -a "$SRC/turtlebot3_moveit_config"       "$DST/"
cp -a "$SRC/moveit_task_constructor"        "$DST/"     # ← 抓取包依赖它，大件，别漏
cp -a "$SRC/turtlebot3_manipulation_navigation2/scripts/dining_grasp_task.py" \
      "$DST/turtlebot3_manipulation_navigation2/scripts/"
```

> 说明：`turtlebot3_manipulation_grasp` 是本项目的抓取包（服务契约 + 适配层 + MTC 规划），
> 队友仓库里**没有**；`turtlebot3_moveit_config` 与 `moveit_task_constructor` 同理。

### 1.2 打补丁（只改这几行，**不要整文件覆盖**）

**a) 前万向轮**（`turtlebot3_manipulation_gazebo/urdf/turtlebot3_waffle_pi.urdf.xacro`）
把下面这段插到既有两个 `caster_back_*` 之后（结构照抄后轮，只有 xyz 不同）：

```xml
  <joint name="${prefix}caster_front_joint" type="fixed">
    <parent link="${prefix}base_link"/>
    <child link="${prefix}caster_front_link"/>
    <origin xyz="0.18 0.0 -0.004" rpy="-1.57 0 0"/>
  </joint>

  <link name="${prefix}caster_front_link">
    <collision>
      <origin xyz="0 0.001 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.030 0.009 0.020"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" />
      <mass value="0.005" />
      <inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0"
               izz="0.001" />
    </inertial>
  </link>
```

> **为什么必须做**：底盘 1.55 kg 而机械臂 19.6 kg、质心高 0.87 m，且支撑多边形只有 `[−0.177, 0]`（万向轮全在后方）
> → 机械臂质心前移 **0.2 mm** 就越界 → 实测抓取时 pitch 达 **12.7°** 且回不来 → 末端偏离约 7 cm，完全抓不到。
> 加前轮后实测 **瞬态 2.0°、稳态 0.2°** ✓。

**b) Nav2 停位容差**（`turtlebot3_manipulation_navigation2/param/turtlebot3.yaml` 与 `turtlebot3_use_sim_time.yaml`）
`general_goal_checker.xy_goal_tolerance: 0.25 → 0.4`（`yaw_goal_tolerance` 保持 0.25；DWB 自己的那个不要动）

**c) CMakeLists 合并**（`turtlebot3_manipulation_navigation2/CMakeLists.txt`）
队友版装了 `insid3_review.py` ✓ 但**删掉了 `dining_grasp_task.py`** ✗ → 结果应是两者都在：

```cmake
install(
  PROGRAMS scripts/patrol_task.py scripts/cmd_vel_relay.py
           scripts/vision_pipeline.py scripts/insid3_review.py
           scripts/dining_grasp_task.py
  DESTINATION lib/${PROJECT_NAME}
)
```

**d) `objects.yaml` 启用 banana**（`turtlebot3_manipulation_grasp/config/objects.yaml`）
把注释行 `# "banana": {...}` 打开（实测尺寸已算好）：
`height: 0.0366, depth: 0.1984, width: 0.0750, grasp_lift: 0.010, graspable: true`

**e) 世界文件** 按 0.2 的结论选定后放入 `wpr_simulation_ros2/worlds/example.world`

### 1.3 阶段 1 验收

```bash
grep -c caster_front <DST>/turtlebot3_manipulation_gazebo/urdf/turtlebot3_waffle_pi.urdf.xacro   # 期望 3
grep -c coke_can_dining <DST>/wpr_simulation_ros2/worlds/example.world                           # 期望 ≥1
grep -n "xy_goal_tolerance: 0.4" <DST>/turtlebot3_manipulation_navigation2/param/*.yaml          # 期望各 1 处
grep -c "def creep_to_standoff" <DST>/turtlebot3_manipulation_navigation2/scripts/dining_grasp_task.py  # 期望 1
grep -c "close_hand_squeeze\|close_hand_speed_scaling" <DST>/turtlebot3_manipulation_grasp/config/grasp_params.yaml
ls <DST>/turtlebot3_manipulation_grasp/src/{grasp_node.cpp,pick_and_place.cpp}
ls -d <DST>/turtlebot3_moveit_config <DST>/moveit_task_constructor
```
任一不符 → 回阶段 1 对应项。

## 阶段 2：构建与环境

```bash
cd <新工作区> && source /opt/ros/humble/setup.bash
colcon build --packages-select turtlebot3_manipulation_grasp turtlebot3_moveit_config \
                              turtlebot3_manipulation_gazebo turtlebot3_manipulation_navigation2 wpr_simulation_ros2
# 全量构建（MTC 也在里面，首次较慢）
colcon build
source install/setup.bash
```

视觉环境（按队友 README）：
```bash
bash setup.sh                 # 建 ~/vision_env（含 torch/sam2/groundingdino-py + rclpy）
bash download_weights.sh      # 下模型权重
source ~/vision_env/bin/activate
```
**验收**：`colcon build` 无错无警告级问题；`ros2 pkg list | grep -E "grasp|moveit_config"` 能看到；venv 里 `python -c "import rclpy, torch"` 均成功。

## 阶段 3：抓取链回归（**关键关卡，先于任何视觉工作**）

重建无头取证台（做法与坑见总结文档 §6），然后：

```bash
bash .dbg_sim.sh <tag> <URDF> 0.33 0.0 0.777 --obstacle 0.495 -0.328 0.777 bowl
```

**验收指标（必须全部满足，否则先修抓取链再谈视觉）**：

| 指标 | 期望 | 历史值 |
|---|---|---|
| 底盘 pitch 峰值 | **≤ 2°** | 无前轮时 12.7° ✗ |
| 罐子 z 抬升 | **≥ 3 cm**（= 真被抓起） | 空合时 0.0000 m ✗ |
| 罐子结束姿态 | 直立（roll ≈ 0）、xy 大致回到原位 | 曾被碰倒（roll −90°）✗ |
| `grasp_node` 返回 | `success=true stage=0` | ✗/✓ |
| 日志关键行 | `合爪目标 …→ 0.0265`、`放置点 … [放回原处]`、`补时间戳` ×3 | — |

> 判据提醒：**`success=true` 不等于真抓起** ✗（规划场景的 attach/detach 是逻辑附着）→ 必须看**罐子位姿**。
> 通过后再用 GUI 复核一次（人眼确认抬手动作干净）。

## 阶段 4：视觉适配层（新增工作，放抓取包里，**不动视觉侧代码**）

### 4.1 现状与缺口

- 队友 `vision_pipeline.py`：`VisionPipeline.detect(img_bgr)` → 2D 检测（phrase + mask ✓），`_mask_center()` 给**像素坐标** ✗；`classify_phrase()` 归一到 `ITEM_NAMES` ✓
- **没有** `DetectGraspTarget` 服务 ✗、**没有 3D** ✗ → 这两块由适配层补齐

### 4.2 实现步骤

新增 `<grasp pkg>/scripts/detect_grasp_target_node.py`（在 `~/vision_env` 里运行，模式照抄 `patrol_task.py` 的环境处理）：

1. 订阅/抓取一帧 RGB（`/camera/image_raw`）+ 对齐深度（`/camera/depth/image_raw`）+ `/camera/camera_info`
   （启动文件已把三者的 `frame_id` 统一覆盖成 `camera_rgb_optical_frame` ✓，像素一一对应 ✓）
2. `VisionPipeline().detect(frame_bgr)` → 对每个检测 `classify_phrase(d["phrase"])`；不在词表里的丢弃
3. **像素 → 3D**：取该 mask 内深度的**中位数**（抗噪）→ 用 `camera_info` 内参反投影 → `camera_rgb_optical_frame` 下的 3D 点；
   用 mask 内点云求**物体质心**
4. **TF**：`camera_rgb_optical_frame → base_footprint`（**完整 3D**，这样底盘有俯仰也自动正确 ✓；
   `lookupTransform` 用当前 stamp，spin 到位再查 ✓）
5. **换算成契约要求的点**：`point = (质心.x, 质心.y, 支撑面高度)`，即"物体中心轴 ∩ 支撑面" ✓
   （支撑面高度取 `grasp_params.yaml` 的 `support_surface` 换算值，或"质心 z − objects.yaml 高度/2"）
6. `confidence` 用检测分数；**按置信度排序**填进响应（最优在前）
7. **打一行对照日志**：`视觉目标 vs gz 真值`（差多少、差在哪个轴）—— 用来量化视觉精度 ✓

### 4.3 阶段 4 验收

- `ros2 service call /detect_grasp_target …` 能返回 targets（`class_id` 与 `objects.yaml` 一致、frame 是 `base_footprint` 或相机系）
- **视觉目标与 gz 真值的差 ≤ 2 cm**（理想 ≤ 1 cm）；超了先查：深度尺度、内参、TF 链、mask 是否含桌面
- **时间戳新鲜度**：grasp_node 不再报"视觉目标不新鲜"（`use_sim_time` 必须跟 /clock ✓；节点要 spin 到时钟前进再打戳 ✓）

## 阶段 5：任务层接视觉

改 `dining_grasp_task.py`：把"gz 真值替身"换成调 `/detect_grasp_target`（`_measure_targets()`/`_read_target_xy()` 这两处），
**保留 gz 真值作为对照打印** ✓（方便随时验证视觉精度）；失败时退化回原路径并在日志里告警 ✓。

**验收**：全流程用视觉驱动跑通一次（导航→微调→抓取），日志里视觉目标与真值对照在阈值内，罐子被真正抓起 ✓。

## 阶段 6：4 类物品适配（比赛清单要求"任选一个"）

| 项 | 动作 |
|---|---|
| banana 启用 | 阶段 1 已做 ✓ |
| **开口超限校验（重要）** | 指尖开合上限约 **0.08 m** ✗，而规划**不检查"开口能否夹住"** ✗。香蕉 0.198×0.075：若抓取朝向跨了**长边**则物理上必然夹不住 ✗。做法：按类给出**优先闭合朝向**（用 `objects.yaml` 的 width/depth 决定），或在适配层对 `max(width,depth)` 超过开口的类别直接降权/拒绝 |
| 各类 `grasp_lift` | 已按类配置 ✓；薄/矮物体按实测再调 |
| 不可夹类别 | `graspable: false` 提前挡掉 ✓ |

## 阶段 7：端到端与比赛检查

1. 完整跑一遍比赛流程（含巡逻 + 计数 + 抓取），记录**每段耗时**（抓取链约 6~7 s，微调 3~6 s）
2. 确认**答案 JSON**：`GROUP_NUMBER` 改成本队真实组号（否则评分按无效文件处理 ✗）、`NAME_TO_JSON` 用规范名（`coke_can` 下划线）、`TARGET_CLASSES_JSON` 按裁判发布的类别改
3. 异常路径：视觉拿不到目标 / 抓取失败时的**退化与上报**（不要卡死）；`--no-creep`、`--skip-nav` 保留为调试开关
4. 清理：临时脚本、调试开关、`HOME`/日志目录等

## 每阶段的回滚点

- 阶段 1 之前备份 `src/`（或确保在 git 下）✓
- 改动**只碰**计划里列出的文件；其余文件保持队友版本 ✓
- 抓取链回归（阶段 3）不通过时，**不要开始视觉工作** ✗ —— 先按总结文档 §3.5/3.6/3.7 的根因逐条核对（前轮是否真加上、合爪两参数是否生效、时间戳兜底是否在跑）

## 交付物清单

- [ ] 新工作区可 `colcon build` 通过
- [ ] 抓取链回归全部达标（表见阶段 3）
- [ ] `detect_grasp_target_node.py` + 对照日志
- [ ] `dining_grasp_task.py` 走视觉路径并可退化
- [ ] 4 类物品适配（含开口校验）
- [ ] 端到端跑通 + 答案 JSON 配置正确
