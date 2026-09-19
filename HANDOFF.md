# 抓取调试交接入口（**先看这份**）

> 本文件是接手调试/继续开发的**唯一入口**；所有交接文档集中在 `docs/handoff/` ✓
> 最近一次更新：2026-09-19

## ★ 最新的一个未解问题：抓取"手停在物体顶上 + 合爪合到指令间距却空合"（2026-09-19）

现场（potted_meat_can）：**夹取高度不够低 + 夹爪合的也宽**。日志已排除运动精度
（`★ 落点核对: 差 (+0,+0) mm`、姿态竖直/水平都正常、`[empty]` = 合到指令间距而两指之间没东西）
⇒ 问题在**送进规划的朝向与高度**这两侧：

1. **合拢轴是算出来的**：`box_yaw = object_yaw_map(常数 0) − robot_yaw_map(TF=AMCL)`
   ⇒ 朝向链全挂在这个 TF 偏航上。长方体需要的开口 `span(φ) = d·|cosφ| + w·|sinφ|`，
   **potted_meat_can 50×97 mm 在 φ > 19.8° 时需要的开口就超过 80 mm 夹爪上限**
   ⇒ 几何上跨不进去、两指压在罐顶把罐子推走、然后空合 ✓
   （现场日志 `箱体偏航(规划帧)=115.7°`，而站位是命令"车头正对桌沿" ⇒ φ ≈ 25.7°）
2. **高度贴在几何下限**：手部碰撞体最低点在指尖平面上方 37.4 mm ⇒ 82 mm 高的罐子只能
   `lift ≥ 0.0036`，而 objects.yaml 取 0.004 ⇒ **只剩 0.4 mm 余量**（臂实测 ±2 mm、视觉 z 误差几 mm）
   ⇒ 手会蹭到罐顶、把罐子推走。

### 追加⑥（2026-09-19 12:25）：改崩了又修回来 —— 并用冒烟测试兜住

`[ERROR] 抓取阶段异常: 'GraspPhase' object has no attribute 'graspable'`
= 我把 `self.all_classes = sorted(self.graspable)` 写在了 `self.graspable = load_graspable()`
**之前** ⇒ 构造就崩、Phase 2 一步不跑 ✗（`py_compile` 与 8 套单测都没拦住，因为都不构造它 ✗）
⇒ 新增 **`HANDOFF_harness/test_grasp_phase_ctor.py`**（假 node 真构造，20 项断言 ✓）
另修视觉侧：**调用方点了类就一律走闭集复核**（旧判据「命中一类就不切」会让整帧只给出
apple/coke can/bowl/banana ⇒ 抓取侧拿到隔壁桌的幻影类别 ⇒ 抓空 ✗）✓
「不带类别 ⇒ 从桌上挑一个物体抓」的完整路径与验收关键词见
`docs/handoff/HANDOFF_closing_axis_and_grasp_height_2026-09-19.md` §15 ✓

### 追加⑤（2026-09-19 12:20 那一趟）：不带 `--classes` 抓空 —— 两处已修

失效链：① 请求里类别为空 ⇒ 视觉先走队友默认 4 类词表（apple/coke can/bowl/banana），
桌上真正的物体经常整帧拿不到，还混进**邻居餐桌的幻影**（apple@map(1.74,2.04)、
potted_meat_can@1.55 m）✗；② "标签抖动就用别的类别代替"的半径太大（0.25/0.35 m）✗
⇒ 0.119 m 外的**可乐罐被当成"番茄罐"**，微调去对正可乐罐、抓取点又用旧值
⇒ 夹爪落在两个物体中间 ⇒ `[empty]` 抓空 ✗✗
修法：① `--classes` 为空时请求 **objects.yaml 全部类别**（视觉直接走闭集复核 ✓）；
② 新增 `LABEL_FLICKER_RADIUS = 0.10 m`（同物体标签抖动只差 2~6 cm，本桌不同物体 ≥0.25 m）
⇒ 0.10 m 以外一律**拒绝替代**并说明原因 ✓（离线自测 22 项 ✓）

### 追加⑦（2026-09-19 14:13）：换 world 后"完全跑不起来"的真因 —— 新文件没进 install

`[ERROR] [ign-2]: process has died [… exit code 255, cmd 'ign gazebo <install>/…/worlds/official-ros2.world -r']`
⇒ `install/.../worlds/` 里是**逐个文件**的软链（`--symlink-install` 只在构建时为**已存在**的文件建链 ✗），
新丢进去的 `.world` 在那里不存在 ⇒ Gazebo 找不到文件退出 255 ⇒ 机器人也没 spawn 上 ✗
**解法**：启动时给**文件名或源码树路径**（`world:=official-ros2.world` 即可 ✓），
或先 `colcon build --symlink-install --packages-select wpr_simulation_ros2`（几秒）。
**已加防呆**：launch 会按「给的原路径 → install/worlds/<名> → 源码树 worlds/<名>」三处找，
打印实际用的路径；都找不到就明确报错 ✓（不再沉默 255）。详见 runbook §6 ✓

### 追加④（2026-09-19）测试布局：potted_meat_can 已挪到餐桌正中（用户要求）

`example.world` 的 4 物体测试布局改为（间距 0.30 → 0.25 m）：
tomato 2.20 / coke 2.45 / **potted_meat_can 2.70（桌子正中）** / cracker 2.95。
原来它在最西端 2.30 ⇒ 观察位偏轴 −21.5°、近看位 −41.8°，只有扫到 ±25°/±45° 才进画面中心
（而它只有 40~70 px ⇒ 逐帧碰运气 ✗）；现在**两个位点的 0° 偏轴都是 0.0°** ⇒ 第一帧就正对它 ✓✓
（`check_dining_view_geometry.py` 已把这条加成断言 ✓）
方向仍保持 yaw=0（50 mm 窄边沿 map x）——**不要转 90°** ✗（要跨 97 mm > 开口 80 mm，夹不进去）。
**重启仿真即生效**（世界文件不用重建）；C++ `grasp_node` 仍需重建一次 ✓

### 追加③（2026-09-19 10:42 那一趟）：改标规则把番茄罐改成了扁罐头 ⇒ 抓错物体还报成功

现象：跑 `--classes potted_meat_can`，最后抓起来的是**番茄罐**。
根因（两层）：
1. 我的"按实测尺寸改标"规则**太松**：番茄罐（66×66×101）掩码实测 **55×90 mm**（SAM2 在几十像素
   物体上系统性偏小 ~11%），与 potted_meat_can(50×97×82) 的高度只差 8 mm ⇒ 被"符合"了 ✗
   ⇒ **单视角的 (横向,竖直) 分不开 82 mm 方盒与 101 mm 圆柱**（长宽比 0.61 vs 0.65）
   ⇒ 已改成 `relabel_decision`：只有标签偏差 > max(30 mm, 45%·标签高度) **且** 调用方要的类
     偏差 ≤ max(12 mm, 18%) **且** 没有别的类更接近时才改；标签自洽就绝不改 ✓
2. 成功判据只问"手指被挡住没有"：实测 61.3 mm 比目标窄边 50 mm 宽 **+11.3 mm** 仍报 `[contact]` ✗
   ⇒ 新增 `wrong_object` 判据（宽 >4 mm ⇒ 抓错物体 ⇒ 不算成功、停止偏置扫描）✓

### 追加②（2026-09-19 10:32 那一趟）：potted_meat_can 就在画面正中，却被叫成了 chips_can

直接读运行中仿真的相机 + 深度 + TF（只读）核对过：四个物体都在桌上、坐标与世界文件对得上；
`potted_meat_can` 在近看位 **−45°** 那一帧里落在画面正中（+3°、0.65 m）✓。
但闭集分类器给它的标签是 **chips_can 0.48 / sugar_box 0.35**（目录高度 0.25 / 0.175 m，
而掩码实测高度只有 **0.074 m**）✗ ⇒ `_height_ok()` 按设计把错标签丢掉 ⇒
响应里 `want` 过滤再滤一次 ⇒ 调用方要的类从没进过候选 ⇒ 恒 "no class matched" ✗✗

★ 关键：`_size_ok()` / `_height_ok()` **已经量对了尺寸**（实测 55×74 mm vs 目录 50×97×82 ✓），
但它们只会"否决"、不会"纠正" ⇒ 缺的一步就是**用实测尺寸把标签改回来** ✓
修法：`shape_fit_class()`（只在**调用方点的类**里按实测高度/横向挑，歧义就不猜）——
现场那条数字喂进去 ⇒ 正确纠正成 `potted_meat_can` ✓；番茄罐（101 mm）不会被误当成它 ✓

### 追加（2026-09-19 10:25 那一趟）：抓取前"一个目标都识别不出来"的真因

现象：观察位 3 个角度 + 近看位 5 个角度，**8 次调用全部** `no detection / no class matched`、0 目标。
视觉日志里那几帧其实认出了 `coke can 0.59~0.88 / apple 0.35~0.43` —— 全是**队友默认 4 类词表**
（apple / coke can / bowl / banana）里的东西，而调用方要的是 `potted_meat_can` ✗。

根因（`detect_grasp_target_node.py` 的模式判据）：旧判据只问"默认 4 类词表里有没有**可夹**的东西"，
**完全不管调用方点了哪几类** ⇒ 画面里只要出现任何可夹的东西（哪怕是隔壁桌的 coke can），
就永远不切"抓取模式"（= 闭集复核，能命名 objects.yaml 的 18 类）⇒ 调用方点的类永远没机会被命名 ✗
修法：抽成纯函数 `needs_grasp_mode(found, want_pre, graspable, mode)` —— **调用方点了类就按它判**
（离线自测把现场那 8 帧喂进去：新判据 8/8 会切，旧判据 0/8）✓
顺带：视觉日志与驱动日志都会把"第 1 帧认出了什么 / 调用方点了什么 / 请求的类别"打出来 ✓

本轮已落地：

1. **诊断**：`grasp_phase.py` 新增 **`★ 合拢轴核对`**（打出 原始 φ / 已注入 e / 生效 φ，
   只用生效 φ 判"夹不夹得进去"）；`check_grasp_geometry.py` 新增 **【下沿余量】** 列
   （实测 10 个可夹类别里有 **5 个余量 = 0.0 mm**：coke can / cracker_box / mustard_bottle /
   potted_meat_can / sugar_box ⇒ "取窗下沿"这条统一规则本身就该改 ✓）。
2. **P1 修法（本轮重点）**：AMCL 的偏航误差 e 是**常量偏置**，动车站消不掉
   ⇒ 用**激光扫餐桌 4 条腿**（世界文件真值，map 里 (2.115/3.285, 1.765/2.235)）反解车的真实偏航，
   把 `e = TF偏航 − 桌腿反解偏航` **注入 `grasp_node` 的 `object_yaw_map`**
   ⇒ `box_yaw = (0+e) − robot_yaw_map` 自动抵消 ✓（C++ 侧已改成**每请求重读**该参数）。
   护栏：命中 <3 腿 / 残差 >20 mm / |e| >34° / 无 `/scan` ⇒ 不注入、保持原行为 ✓
   ⚠ **必须重建**：`colcon build --symlink-install --packages-select turtlebot3_manipulation_grasp`

详见 **[docs/handoff/HANDOFF_closing_axis_and_grasp_height_2026-09-19.md](docs/handoff/HANDOFF_closing_axis_and_grasp_height_2026-09-19.md)**
（含实测几何表、临界角推导、§8 P1 的推导/落地/验收判据、下一步 P2~P4 清单）✓

## ★★ 最高优先级：Fortress 不支持 URDF mimic ⇒ 第二根手指卡在"闭合端"（2026-09-18 已修，待重建重启验证）

`fr3_finger_joint2` 在 URDF 里靠 `<mimic>` 联动，但 **Fortress 的 sdformat12 = SDF 1.9 不支持 mimic**
（实测 `ign sdf -p <urdf> | grep -c mimic` ⇒ **0**），而 `ros2_control` 只声明了 joint1
⇒ joint2 在仿真里是**被动关节**，停在默认值 0（= 闭合端），而 joint1 上电是 0.04（张开）
⇒ **物理上右指内侧面永远停在夹爪中线附近**，规划层却以为它在 −q：
计划 80 mm 的对称开口，实际只有 ~40 mm 且偏轴 ~20 mm ✗
⇒ 顶抓时右指落在物体**轴线**上，被物体顶面挡住、臂停在半路
（番茄罐停在比计划高 22 mm、sugar_box 高 45 mm ⇒ 现场"指尖顶在物体顶面、伸不下去" ✓）。

**修法**：joint2 也声明成受控关节 + 新脚本 `finger_mimic_relay.py` 把 joint1 的位置转发给它（见
`docs/handoff/HANDOFF_nav_survey_and_grasp_tolerance_2026-09-18.md` §10）。
**落地要重建一次并重启仿真**：
```bash
cd ~/Robocup@home_ws && colcon build --symlink-install --packages-select turtlebot3_manipulation_gazebo
source install/setup.bash && ros2 launch turtlebot3_manipulation_gazebo turtlebot3_franka.launch.py
# 验收：两根手指一起动；/joint_states 里两个 finger 关节值几乎相同；无"✗ 两指不同步"告警
```
> ⚠ 2026-09-18 首次上线时 Gazebo 闪退过：新脚本没有 +x ⇒ Permission denied（退出码 126）⇒
> `ros2 launch` 默认"任一进程非零退出就全部关闭" ⇒ 仿真被一起带走 ✗。
> 已 `chmod 755` 并把它包进 shell（失败只提示、不再关全场）✓；**内容与权限走软链，不需要重新 build** ✓
> 重启仿真前先清进程：`pkill -9 -f 'ign gazebo'; pkill -9 -f parameter_bridge; pkill -9 -f robot_state_publisher; pkill -9 -f ros_gz_bridge`
> 这条同时解释了最顽固的老现象"夹爪看着包住物体、能闭合，物体纹丝不动"：
> 以前 `两指真实间距 = 2×关节值` 是**运动学理想值**（TF 侧应用了 mimic），不是物理值 ✗

## 一句话现状

整条链（巡逻计数 → 观察位选目标 → 导航站位 → 相对微调 → MTC 抓取 → 放回）**已跑通到 `stage=0`** ✓；
2026-09-18 修掉了两个用户现场问题（**未在仿真里复跑，先跑离线自检 ✓**）：
1. **"到观察点后绕到餐桌另一边并撞桌子"** —— 3 个根因已定位并修（近看位按桌心算 ⇒ 东西侧只剩 0.08 m／
   站位余量 0.26 m < Nav2 `robot_radius` 0.28 m ⇒ 目标点在 inscribed 区到不了／支撑面门关掉 ⇒ 被邻桌幻影劫持）；
   现在观察位**原地扫视** + 近看位**沿同一条法线用 cmd_vel 前进**（不经过 Nav2 ⇒ 不可能绕桌）✓
2. **"夹爪碰到物体但夹不住"** —— 判据改成**两指真实间距 vs 指令间距**
   （`★ 夹到没有: [contact]/[empty]/[unclosed]`），空合就**换偏置重抓**（先更深 +40/+80 mm，再侧向 ±12 mm）；
   **服务报 stage=0 不再算成功** ✓
   视觉侧修了 3 个：订阅/服务回调组互斥（服务期间收不到新帧）＋ 尺寸核对按距离放宽（远处真目标不再被整帧丢弃）
   ＋ **桌面剔除**（掩码吃进"物体前方那条桌面"会把抓取点拉近 ~100 mm —— 用户实跑一次的"抓取点不够靠后"就是它，
   同一个原因也解释了"在另外的位点放下"：放置点 = 抓取点，一起偏 ✗）✓

## 按这个顺序读

| # | 文档 | 内容 |
|---|---|---|
| 0 | **[docs/handoff/HANDOFF_closing_axis_and_grasp_height_2026-09-19.md](docs/handoff/HANDOFF_closing_axis_and_grasp_height_2026-09-19.md)** | **最新**：抓取"停在物体顶上/合爪空合"的两个根因候选（合拢轴 φ 与开口上限、高度只剩 0.4 mm 余量）+ 新增的 `★ 合拢轴核对` 诊断（**先看这份** ✓）|
| 1 | [docs/handoff/HANDOFF_nav_survey_and_grasp_tolerance_2026-09-18.md](docs/handoff/HANDOFF_nav_survey_and_grasp_tolerance_2026-09-18.md) | 幻影目标/绕桌 + 接触判据 + 桌面剔除 的根因/改动/离线自检/仿真验收判据 |
| 2 | [docs/handoff/HANDOFF_agent_plan_vision_nav_2026-09-18.md](docs/handoff/HANDOFF_agent_plan_vision_nav_2026-09-18.md) | 视觉目标选择 + 导航站位执行计划（幻影故障链 / P0-P2 清单）|
| 3 | [docs/handoff/HANDOFF_debug_state_2026-09-14.md](docs/handoff/HANDOFF_debug_state_2026-09-14.md) | 抓取物理层进度 + 未解问题与方向 + 工具 + 复现步骤 + 踩过的坑 |
| 4 | [docs/handoff/HANDOFF_grasp_fix_2026-09-14.md](docs/handoff/HANDOFF_grasp_fix_2026-09-14.md) | 抓取几何/规划层的修复记录（抓取高度、对正桌子、去真值）|
| 5 | [docs/handoff/README.md](docs/handoff/README.md) | 交接文档索引（更早的迁移/设计/接口文档）|
| 8 | **[docs/handoff/HANDOFF_count_items.md](docs/handoff/HANDOFF_count_items.md)** | 基础题（视觉识别+计数）**换待计数物品清单**：一处 `count_items.yaml` 同步三处词表 + 硬约束/验收 ✓ |
| 7 | **[docs/handoff/HANDOFF_world_swap_runbook.md](docs/handoff/HANDOFF_world_swap_runbook.md)** | **比赛当天换 world 的最快流程**（`world_tools.py check/map` + 6 处同步 + 验收清单）✓ |
| 6 | [HANDOFF_harness/README.md](HANDOFF_harness/README.md) | 调试工具怎么用（几何自检/视觉探针/抓取取证/抓取点扫描）|

## 改完先跑这两份离线自检（**不用起仿真**）

```bash
cd ~/Robocup@home_ws && source install/setup.bash
python3 src/HANDOFF_harness/check_dining_view_geometry.py   # 观察/近看/站位几何 + 目标选择（全绿 ✓）
python3 src/HANDOFF_harness/test_grasp_retry.py             # 接触判据/偏置阶梯/尺寸核对（33 项 ✓）
python3 src/HANDOFF_harness/test_table_pixel_filter.py      # 桌面剔除（复现"抓取点偏浅 132 mm"并验证修好 ✓）
python3 src/HANDOFF_harness/test_closure_phase.py           # 合拢判定 + closing_span_mm（47 项 ✓）
python3 src/HANDOFF_harness/test_table_yaw_probe.py         # 桌腿基准：反解偏航 + 注入符号（35 项 ✓）
python3 src/HANDOFF_harness/test_grasp_phase_ctor.py       # ★ 构造冒烟（20 项 ✓）
python3 src/HANDOFF_harness/test_count_items.py            # ★ 待计数清单三处联动（16 项 ✓）
python3 src/HANDOFF_harness/test_grasp_mode_switch.py       # ★ 视觉模式判据（13 项 ✓）
python3 src/HANDOFF_harness/test_shape_relabel.py           # ★ 改标/抓错物体/标签抖动半径（22 项 ✓）
python3 src/HANDOFF_harness/check_grasp_geometry.py potted_meat_can --sweep  # 逐高度可行角（高度策略的依据 ✓）
```

## 最关键的一条判据（抓取日志）

```
★ 夹到没有: [contact] …   ⇒ 手指被挡住 = **夹到了** ✓（这一档偏置 = 物体真实估位，可据此标定视觉）
★ 夹到没有: [empty] …     ⇒ 两指之间是空的 ✗（驱动会自动换偏置重抓，最多 5 档）
★ 夹到没有: [unclosed] …  ⇒ 夹爪根本没动（规划/执行失败）**也不算夹到** ✗
```
- 旧判据（`两指真实间距: 起始 80.0 mm → 最小 X mm`）仍会打印，可交叉核对 ✓
- **别看 `stage=0 / success=true`**：MTC 的 `attach` 只是规划场景里的逻辑附着，空合也报成功 ✗

## 复现（四个终端）

```bash
ros2 launch turtlebot3_manipulation_gazebo turtlebot3_franka.launch.py      # 1 仿真
ros2 launch turtlebot3_manipulation_navigation2 navigation.launch.py        # 2 导航
source ~/venvs/vision_env/bin/activate                                      # 3 抓取服务（含视觉）
ros2 launch turtlebot3_manipulation_grasp grasp_service.launch.py
ros2 run turtlebot3_manipulation_navigation2 dining_grasp_task.py           # 4 抓取阶段
#   也可以只抓指定类别：  --classes "tomato_soup_can"
#   车已停好快速验：      --keep-pose --skip-nav
```

## 三个最该先做的事

1. **修视觉的绝对位置精度**（车停住连调 5 次看抖动；最优先 ✗）
2. 用**圆柱**（world 里已把餐桌3 的 pudding_box 换成 `tomato_soup_can` ✓）做干净对照
3. `sugar_box` 的**可视网格比碰撞体胖 11.5 mm** ⚠️ → 跟裁判确认（`docs/handoff/HANDOFF_debug_state_2026-09-14.md` §2.7）
