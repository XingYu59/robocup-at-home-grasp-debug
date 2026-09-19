# 换世界（比赛当天裁判给新 world）最快流程 —— 20 分钟清单

> 目标：**不改代码**，把新 world 跑起来（定位/导航/视觉/抓取整条链 ✓）。
> 工具（都不需要起仿真）：`HANDOFF_harness/world_tools.py`
> ```bash
> python3 src/HANDOFF_harness/world_tools.py check <新world>     # 体检 + 打印"该改哪一行、改成什么"
> python3 src/HANDOFF_harness/world_tools.py map   <新world>     # 直接从 world 生成 map.pgm/yaml（不用跑 SLAM ✓）
> ```

## 0. 三步跑起来（TL;DR）

```bash
cd ~/Robocup@home_ws && source install/setup.bash

# ① 体检 + 拿到该改的值（会打印 support_surface / NEIGHBOR_TABLES / 缺模型 / 缺物体目录）
python3 src/HANDOFF_harness/world_tools.py check /path/to/arena.world

# ② 按打印结果改两处配置（都只有几行）
#    turtlebot3_manipulation_grasp/config/grasp_params.yaml  → support_surface.pose/length/width
#    turtlebot3_manipulation_navigation2/scripts/grasp_phase.py → INITIAL_X/Y/YAW（=出生位姿）、NEIGHBOR_TABLES

# ③ 生成地图并覆盖 nav2 的图
python3 src/HANDOFF_harness/world_tools.py map /path/to/arena.world
#    按它打印的 cp 命令覆盖 turtlebot3_manipulation_navigation2/map/map.{pgm,yaml}

# ④ 启动（world 与出生位姿都是 launch 参数 ✓ 不用改代码）
ros2 launch turtlebot3_manipulation_gazebo turtlebot3_franka.launch.py \
     world:=/path/to/arena.world spawn_x:=-5.30 spawn_y:=-0.50 spawn_yaw:=0.0
```

## 1. 换世界一共牵动 6 个地方（现状与状态）

| # | 东西 | 在哪 | 现状 |
|---|---|---|---|
| 1 | 世界文件路径 | `turtlebot3_manipulation_gazebo/launch/turtlebot3_franka.launch.py` | ✅ **已参数化**（`world:=…`，默认 example.world）|
| 2 | 机器人出生位姿 | 同上 | ✅ **已参数化**（`spawn_x/y/z/yaw:=…`）|
| 3 | AMCL 初始位姿 | `grasp_phase.py: INITIAL_X, INITIAL_Y, INITIAL_YAW` | ⚠ 仍写死，**必须与 ② 一致**（不一致 ⇒ 一启动定位就偏 ✗）|
| 4 | 抓哪张桌子（支撑面）| `grasp_params.yaml: support_surface.pose/length/width/thickness` | ⚠ 现在是餐厅 3 号桌 (2.7, 2.0)，换场地要改（`check` 会打印要粘贴的三行 ✓）|
| 5 | 邻居桌（判"隔着本桌看到的物体不算目标"）| `grasp_phase.py: NEIGHBOR_TABLES / NEIGHBOR_HALF` | ⚠ 只有 `TABLE_IS_DINING`（=支撑面就是餐厅那张）时才启用；换场地后多半要改 ✓ |
| 6 | 地图 + 物体目录 | `nav2/map/map.{pgm,yaml}`、`grasp/config/objects.yaml` | ✅ 地图可由 `world_tools.py map` 秒级生成；目录缺条目时 `check` 会给出可粘贴片段 ✓ |

> 另有 **模型缺失** 这个硬坑：新 world 里若引用了我们没有的 `model://…`，Gazebo 会报错/模型消失 ✗
> —— `check` 第一段就会逐个列出来 ✓（把模型目录放进 `wpr_simulation_ros2/models/` 即可）

## 2. 每一步的验收（按顺序，别跳）

```bash
# 2.1 世界起得来、没有缺模型
ros2 launch ... turtlebot3_franka.launch.py world:=/path/to/arena.world
#     看：Gazebo 无 "Unable to find model" / 没有模型悬浮或消失 ✓

# 2.2 定位收敛（出生位姿与 AMCL 初始位姿一致 ⇒ 一开始就该对上）
ros2 run tf2_ros tf2_echo map base_footprint     # 位姿应≈spawn_*，别跳
ros2 topic echo /scan --once                     # 有数据、frame=base_scan

# 2.3 地图对得上（rviz 里叠加 LaserScan 与 Map：墙/桌沿应重合成一条线 ✓）
#     若整体平移/旋转 ⇒ map.yaml 的 origin 或 AMCL 初始位姿不对 ✗

# 2.4 目标桌子选对了
python3 src/HANDOFF_harness/check_dining_view_geometry.py    # 观察位/近看位覆盖账（会用新的桌位重算 ✓）
python3 src/HANDOFF_harness/world_tools.py check /path/to/arena.world   # 确认"★ 物体最多的桌子"就是要抓的那张

# 2.5 整条链
ros2 run turtlebot3_manipulation_navigation2 dining_grasp_task --ros-args -p use_sim_time:=true
#     看：扫视找到 ≥1 个目标（且位置在本桌 ✓）→ 站位 → 微调 → ★ 夹到没有: [contact]
```

## 3. 万一 `check` 认不出桌子/物体（新 world 形状怪）

- **桌子**：`check` 判"薄而大（z≤0.12 m、x/y≥0.4 m、0.4~1.1 m 高）的盒体"。认不出的桌子直接
  在 Gazebo 里点一下模型看它的 pose，把 `support_surface.pose` 按 `[x, y, 0.765, 0, 0, yaw]` 填 ✓
  （z 用"桌面顶高度 − 0.015"；桌面尺寸填桌面的长宽 ✓）
- **桌子只有网格碰撞**：`check` 会在最后点名；这种桌子要手工量尺寸 ✓
- **物体不在 objects.yaml**：`check` 会打印可粘贴的条目（含尺寸）✓；`grasp_lift` 用
  `python3 src/HANDOFF_harness/check_grasp_geometry.py <名称> --sweep` 扫出可行高度再填 ✓
  （本手册的`graspable` 判据：窄边 ≤ 0.075 m 且几何自检有可行角 ✓）

## 4. 已知会"看着像坏了"的两个现象

| 现象 | 原因 | 处理 |
|---|---|---|
| 一启动就定位偏、rviz 里机器人位置飘 | ③ INITIAL_* 与 ② spawn_* 不一致 ✗ | 改成一致 ✓（`check` 会提醒）|
| 观察位/站位算到桌子里面或墙上 | 地图没换（旧图 + 新 world 必然对不上）✗ | 重新 `world_tools.py map` 覆盖 ✓ |

> 本手册对应的模型解析范围：world 里的 `<include>`/内联 `<model>` + 盒体碰撞 ✓；
> 只有网格碰撞的模型不参与地图栅格化（`check` 会点名 ✓）。

## 5. 实例：裁判下发的 `example_patch-ros2.world` 实测结论（2026-09-19）

`world_tools.py check` + 与我们的 `example.world` 逐条比对后：

| 项目 | 结论 |
|---|---|
| 世界名 / 插件 / 重力 / 物理步长 | `robocup_home`；4 个插件与我们的**完全一致** ⇒ **我们的 launch/桥接/机器人 xacro 原样可用** ✓ |
| `model://` 解析 | **0 个缺失** ✓（不会出现模型消失 ✗）|
| 桌子位姿 | 与我们**完全相同**（dinning_table_3 = (2.7, 2.0) yaw 1.57 ✓ …）|
| 地图 | 场馆模型（`RoboCup_Home`）与我们的一字不差（**逐字节相同，只有名字不同** ✓）⇒ **不用重画地图** ✓：两图只在 SLAM 未知区与 0.10 m 薄墙的 1~2 格边界上不同，桌心/过道/起点全一致 ✓ |
| 物体 | 它**删掉了我们加的全部物体**（4 个 `*_t3` + 其它桌上的 banana/beer/chips_can/…），只留 4 个：apple×2 / coke_can / bowl |

⚠ **关键差异（会直接决定"抓得到 / 抓不到"）**：这 4 个物体的落点是

```
apple_clone_0  (-2.05, +0.05)  → living_room_table_0(-2.1, 0) 上   ✓ 可夹(72mm,4 角)
coke_can       (-2.35, -0.06)  → living_room_table_0 上            ✓ 可夹(67mm,12 角)
apple_clone_1  (-3.48, -3.40)  → living_room_table_3(-3.5,-3.1) 上 ✓ 可夹
bowl           (+2.67, +2.02)  → dinning_table_3(2.7, 2.0) 上      ✗ **不可夹**（0.16 m > 开口 80 mm）
```

⇒ 用官方 world 时**餐厅 3 号桌上没有任何可夹物体** ✗ ⇒ 必须把
`grasp_params.yaml` 的 `support_surface` 指到 **`living_room_table_0`**：

```yaml
support_surface.pose: [-2.1, 0, 0.765, 0.0, 0.0, 1.5707963267948966]   # living_room_table_0
support_surface.length: 0.5
support_surface.width: 1.2
```

（`world_tools.py check` 会自动算出这三行并标出"★ 物体最多的桌子" ✓；
另外 `NEIGHBOR_TABLES` 只在本桌=餐厅桌时才起作用，换到客厅桌后可以不改 ✓）

### 5.1 两个可直接启动的世界（已放进 `wpr_simulation_ros2/worlds/`）

| 文件 | 内容 | 用途 |
|---|---|---|
| `robocup_home.world` | 裁判原样（一个字节没改 ✓）| 贴近比赛的验收 |
| `robocup_home_testobjects.world` | 官方 world + 我们餐桌 3 的 4 个测试物体（potted 在正中 ✓）| 在**官方场地**上验证抓取逻辑 ✓ |
| `example.world` | 原来的仓库示例（含 8 张桌 + 各种物体）| 老场景/回归 |

```bash
# 官方世界（先按上面把 support_surface 指到 living_room_table_0）
ros2 launch turtlebot3_manipulation_gazebo turtlebot3_franka.launch.py \
     world:=$PWD/src/wpr_simulation_ros2/worlds/robocup_home.world
# 官方世界 + 我们的测试物体（support_surface 仍指餐厅 3 号桌 = 现在的配置 ✓ 不用改）
ros2 launch turtlebot3_manipulation_gazebo turtlebot3_franka.launch.py \
     world:=$PWD/src/wpr_simulation_ros2/worlds/robocup_home_testobjects.world
```

> 换 world **不需要重建**：`install/.../worlds/` 是源码树的软链 ✓；只要模型目录在
> `IGN_GAZEBO_RESOURCE_PATH` 里（launch 已设好 ✓）就能直接起 ✓

## 6. ⚠ 踩过的坑：新加的 `.world` **不会自动进 install**（2026-09-19 现场）

**症状**：往 `src/wpr_simulation_ros2/worlds/` 丢进一个新的 `.world` 后启动，
Gazebo 起不来、机器人也没 spawn，launch 直接退出：

```
[ERROR] [ign-2]: process has died [pid …​, exit code 255,
  cmd 'ign gazebo /home/xing/Robocup@home_ws/install/wpr_simulation_ros2/share/
       wpr_simulation_ros2/worlds/official-ros2.world -r']
[INFO]  [ros_gz_sim]: Requesting list of world names.   ← create 一直取不到 world 列表 ⇒ 也死了
```

**根因**：`install/.../share/wpr_simulation_ros2/worlds/` 里是**逐个文件**的软链
（`--symlink-install` 只在**构建时**为已存在的文件建软链 ✗）——实测那个目录里只有
`example.world` 一个软链 ⇒ 新文件在 install 里**不存在** ⇒ `ign gazebo` 报"文件找不到"、
退出码 255 ⇒ 整个 launch 连机器人一起没起来 ✗

**三种解法（任选）**

```bash
# ① 直接给具体文件（最省事）：文件名就行，launch 会自动去 install/源码里找 ✓
ros2 launch turtlebot3_manipulation_gazebo turtlebot3_franka.launch.py world:=official-ros2.world
# ② 给源码树的绝对路径（一定对）
ros2 launch ... world:=$PWD/src/wpr_simulation_ros2/worlds/official-ros2.world
# ③ 让 install 也认它（几秒钟，之后 install 路径也能用）
colcon build --symlink-install --packages-select wpr_simulation_ros2
```

**已加防呆**：launch 现在按 `给的原路径 → install/worlds/<文件名> → 源码树 worlds/<文件名>`
三处找（源码目录由 install 里 `example.world` 软链反推 ✓），并打印**实际用的是哪个**；
都找不到就打印全部候选并**明确报错**（不再让 Gazebo 用 255 闷掉 ✓）。
