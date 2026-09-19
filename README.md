> ## 🔧 抓取调试交接（2026-09-18 更新）
> **接手调试请先看 [HANDOFF.md](HANDOFF.md)** —— 现状、未解问题与方向、工具、复现命令、踩过的坑都在那里；
> 详细交接文档集中在 [`docs/handoff/`](docs/handoff/)（最新一份：
> [观察位扫视 + 导航不再绕桌 / 抓取精度与容错](docs/handoff/HANDOFF_nav_survey_and_grasp_tolerance_2026-09-18.md)）。
> 一句话现状：整条链跑到 `stage=0` ✓，但 `stage=0` **不等于夹住了**（MTC 的 attach 只是逻辑附着 ✗）；
> 现在唯一判据是日志里的 `★ 夹到没有: [contact] / [empty]`，空合会自动换偏置重抓 ✓
> **改完先跑三条离线自检**（不用起仿真）：
> ```bash
> cd ~/Robocup@home_ws && source install/setup.bash
> python3 src/HANDOFF_harness/check_dining_view_geometry.py   # 观察/近看/站位几何 + 目标选择
> python3 src/HANDOFF_harness/test_grasp_retry.py             # 接触判据 + 偏置重抓阶梯
> python3 src/HANDOFF_harness/test_closure_phase.py           # 合拢时刻判定（既有回归）
> ```

# TurtleBot3 + FR3 导航仿真

TurtleBot3 Waffle Pi + FR3 机械臂在 **Gazebo Fortress (Ignition Gazebo)** 中的仿真包，
包含机器人仿真、仿真场景、导航（Nav2）与串行巡逻任务节点，以及一套
GroundingDINO + SAM2（+ INSID3 复核）的开放集视觉识别流水线。

## 包含内容

| 包 / 目录 | 说明 |
|---|---|
| `franka_description` | FR3 机械臂 URDF/meshes（tag 2.8.1），仿真 URDF 的硬依赖 |
| `turtlebot3_manipulation_gazebo` | 仿真 spawn 启动（`turtlebot3_franka.launch.py`）、TB3+FR3 URDF、ros2_control 配置、网格 |
| `turtlebot3_manipulation_navigation2` | Nav2 启动（`navigation.launch.py`）、地图/参数、任务节点 `patrol_task.py`、视觉识别流水线 `vision_pipeline.py`、INSID3 复核 `insid3_review.py`、cmd_vel 桥接 |
| `wpr_simulation_ros2` | 仿真场景资源（`worlds/example.world` + `models/`） |
| `third_party/` | 视觉识别用的第三方源码（git submodule，固定 commit） |
| `docs/` | 两份说明文档（见下） |

## 目录结构

```
.
├── franka_description/                 # FR3 机械臂 URDF/meshes（依赖）
├── turtlebot3_manipulation_gazebo/     # 仿真包
├── turtlebot3_manipulation_navigation2/  # 导航 + 任务 + 视觉识别包
├── wpr_simulation_ros2/                # 仿真场景包
├── third_party/                        # git submodule（固定 commit）
│   ├── sam2/      # facebookresearch/sam2
│   ├── INSID3/    # visinf/INSID3
│   └── dinov3/    # facebookresearch/dinov3
├── setup.sh                            # 一键搭建环境
├── download_weights.sh                 # 下载模型权重
├── build.sh                            # 干净环境构建
├── requirements.txt                    # 视觉识别 Python 依赖
├── docs/
│   ├── turtlebot3-fr3-fortress-integration.md  # TB3+FR3 集成细节（含改动清单）
│   └── task-navigation.md              # 任务节点关键配置（位姿/顺序/流程）
└── README.md
```

## 环境要求

- **Ubuntu 22.04 + ROS 2 Humble**
- **Gazebo Fortress (Ignition Gazebo)** + `ros_gz_sim` / `ros_gz_bridge` / `gz_ros2_control`
- **Navigation2**：`ros-humble-navigation2`、`ros-humble-nav2-bringup`
- **Python 3.10**（虚拟环境，见下）

## 快速开始（一键）

```bash
# 1. 克隆（含第三方 submodule）
cd ~/turtlebot3_ws/src
git clone --recursive https://github.com/Jasmine-GYZ/turtlebot3_franka_public.git

# 2. 搭建环境（ROS 包 + submodule + 虚拟环境 + 权重）
cd turtlebot3_franka
bash setup.sh              # 无 GPU 用 CPU 版 torch；有 NVIDIA GPU 加 --cuda

# 3. 构建
cd ~/turtlebot3_ws
bash src/turtlebot3_franka/build.sh   # 或见下方「构建」一节的 colcon 命令
source install/setup.bash
```

> 若 clone 时忘了 `--recursive`，在仓库根补一次 `git submodule update --init --recursive`。

## 额外依赖（apt 安装即可，`setup.sh` 已自动安装）

| 依赖 | 安装 |
|---|---|
| `xacro`、`robot_state_publisher`、`joint_state_publisher_gui` | `sudo apt install ros-humble-xacro ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui` |
| `ros2_control`、`ros2_controllers`、`gripper_controllers` | `sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gripper-controllers` |
| `ros_gz_sim`、`ros_gz_bridge`、`gz_ros2_control` | `sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-gz-ros2-control` |
| `rviz2` | `sudo apt install ros-humble-rviz2` |

> `franka_description` 已打包在本仓库里，无需再单独 clone。

## 视觉识别（GroundingDINO + SAM2 + INSID3 复核）

巡逻节点到达观察点后调用 `vision_pipeline.py` 做开放集检测 + 闭集复核 + 实例分割：

```
相机帧 → GroundingDINO 检测（原始 logits 逐框 argmax 取单一标签，消除多标签拼接）
       → NMS + 置信度过滤
       → INSID3 闭集复核（冻结 DINOv3 骨干，剔除干扰物 / 修正误判标签）
       → SAM2 框提示分割 → 掩码像素中心 + 深度反投影 → /map 3D 坐标
       → 目标物品计数 + RViz Marker 标记
```

### 依赖

- **第三方源码（submodule）**：`third_party/` 下的 `sam2`、`INSID3`、`dinov3`，
  已用 git submodule 固定 commit，无需再 clone；GroundingDINO 用 PyPI 的 `groundingdino-py`
  包（模型代码 + config 都来自它），无需额外源码仓库。
- **模型权重**（见 `download_weights.sh`）：
  - GroundingDINO / SAM2（公开 URL，自动下载到 `$MODEL_WEIGHTS_DIR`，默认 `~/model_weights`）：
    - `$MODEL_WEIGHTS_DIR/groundingdino/groundingdino_swint_ogc.pth`
    - `$MODEL_WEIGHTS_DIR/sam2/sam2_hiera_small.pt`
  - INSID3 用 DINOv3 base 骨干（约 342MB，官方门控，需手动下载，放在仓库内）：
    - `turtlebot3_manipulation_navigation2/scripts/checkpoints/dinov3_vitb16_pretrain_lvd1689m-73cec8be.pth`
- **INSID3 参考图**：`turtlebot3_manipulation_navigation2/scripts/reference_views/` 下 18 类物体、
  每类 6 视角（前/后/左/右/上/下）的 Gazebo 渲染截图，用作闭集复核的类别原型。
- **Python 虚拟环境**：`setup.sh` 会创建（默认 `~/vision_env`，可用 `VISION_ENV_DIR` 改），
  含 torch、groundingdino-py、sam2、supervision、hydra，以及 INSID3 依赖 einops/scikit-learn、
  rclpy/sensor_msgs 等。运行任务节点前必须先 `source ~/vision_env/bin/activate`。

### 关键说明

- 相机 RGB 话题：`/camera/image_raw`，深度 `/camera/depth/image_raw`，内参 `/camera/camera_info`。
- 待计数物品 / 阈值在 `vision_pipeline.py` 顶部配置区（`ITEM_NAMES`、`BOX_THRESHOLD` 等）。
- 闭集复核在 `insid3_review.py`：用冻结的 INSID3（DINOv3 骨干 + 位置偏置去相关，Train-Free 只推理零训练）
  对每个候选框 crop 与各类参考原型做余弦相似度 argmax；命中目标类则保留并修正标签，命中干扰类或低置信则丢弃。
  类别集合与参考图路径集中在文件顶部 `INSID3_CLASSES` / `REF_MODELS_ROOT` 配置区；权重缺失时
  `vision_pipeline.py` 会捕获异常并降级为「无复核」继续运行。
- 四种物品（`apple` / `coke can` / `bowl` / `banana`）计数结果：扫描完成后在终端打印，同时累计在 `item_counts`。
- 识别到的物品在 RViz 的 `/map` 坐标系下以 `visualization_msgs/Marker`（话题 `/detected_items`）标记；
  位置由「像素中心 + 对齐深度反投影 + TF 相机光学帧→map」解算得到。
- 相邻观察点可能扫到同一物体（或把远处物体认错），按 `/map` 坐标去重（`DEDUP_DIST` 阈值），
  同一位置只计一次、只标一个 Marker，以**首次登记**为准（后续重复检测直接丢弃）。
- 单个观察点内同一物体不会被框两次：GroundingDINO 输出的框先 NMS，再按**框中心距离**合并
  同一物体的重复框（只留得分最高者）。
- 空桌不框选：低于桌面高度（`MIN_OBJECT_Z=0.70m`，/map z）的检测判为桌腿/地面等结构，直接丢弃。
- 识别结果（数量 + 每个目标像素中心 + /map 3D 位置）汇总在 `detection_results`，标注图存到 `~/turtlebot3_detections/`。

### 路径 / 配置（环境变量）

| 环境变量 | 默认值 | 说明 |
|---|---|---|
| `MODEL_WEIGHTS_DIR` | `~/model_weights` | GroundingDINO / SAM2 权重根目录 |
| `VISION_ENV_DIR` | `~/vision_env` | 虚拟环境目录（setup.sh 用） |
| `ANSWER_OUTPUT_DIR` | `~/turtlebot3_ws/submissions` | 答案 JSON 输出目录 |

## 构建

把本仓库放到你自己的 `turtlebot3_ws/src/` 下（克隆或软链均可），然后：

```bash
cd ~/turtlebot3_ws
colcon build --symlink-install \
  --packages-select franka_description \
  turtlebot3_manipulation_gazebo \
  turtlebot3_manipulation_navigation2 \
  wpr_simulation_ros2
source install/setup.bash
```

> 用 `--symlink-install`，之后改 `patrol_task.py` 等 Python 脚本无需重新编译。
> 仓库根的 `build.sh`（克隆后位于 `<ws>/src/turtlebot3_franka/build.sh`）
> 会在干净环境里构建（清空 ROS 前缀、只 source 系统 ROS），避免脏终端污染。

## 运行（三个终端）

**终端 1 — 仿真 + spawn 机器人（含 FR3 臂）**

```bash
ros2 launch turtlebot3_manipulation_gazebo turtlebot3_franka.launch.py
```

**终端 2 — 导航（Nav2 + 地图 + 参数）**

```bash
ros2 launch turtlebot3_manipulation_navigation2 navigation.launch.py
```

**终端 3 — 任务节点（初始定位 → 串行巡逻 + 视觉识别）**

```bash
source ~/vision_env/bin/activate   # 视觉识别依赖环境（必须先激活）
ros2 run turtlebot3_manipulation_navigation2 patrol_task.py
```

任务节点会依次导航到 4 个客厅观察点（顺序 `table_3 → table_1 → table_0 → table_2`），
到达后站稳识别（GroundingDINO + SAM2），再移动到下一个。详见 `docs/task-navigation.md`。

### 比赛答案 JSON（评分用）

`patrol_task.py` 顶部有评分相关的配置，换队伍 / 换题时务必检查：

- `GROUP_NUMBER`：组号，答案文件会命名为 `<GROUP_NUMBER>_answer.json`。
- `TARGET_CLASSES_JSON`：本轮需写入答案的目标类别（规范名），须与裁判发布的类别集合完全一致。
- `NAME_TO_JSON`：内部类别名 → 评分规范类别名的映射。
- `ANSWER_OUTPUT_DIR`：答案输出目录（默认 `~/turtlebot3_ws/submissions`，可用环境变量覆盖）。

## 文档说明

- `docs/turtlebot3-fr3-fortress-integration.md` — TB3+FR3 在 Fortress 的集成改动清单、控制器/传感器验证结果、转运姿态关节值。
- `docs/task-navigation.md` — 任务节点的初始位姿、访问顺序、串行导航流程、地图坐标系约定。

## 进程清理（重启仿真前）

```bash
pkill -9 -f 'ign gazebo'
pkill -9 -f parameter_bridge
pkill -9 -f robot_state_publisher
pkill -9 -f ros_gz_bridge
```
