#!/usr/bin/env python3
"""
抓取阶段（Phase 2）—— **唯一实现**，两个入口共用：

    patrol_task.py       比赛主流程：Phase 1（巡逻计数 + 写答案 JSON）跑完 → 调 GraspPhase
    dining_grasp_task.py 独立调试入口（薄壳）：只做参数解析 → 调同一个 GraspPhase

为什么不各写一份：旧工作区的 dining_grasp_task.py 是独立驱动，而规则书 3.1 要求
"基础题做完不回起点、直接从客厅去餐厅" → 抓取必须并进主流程。两份逻辑各自漂移是坑 ✗

流程（每个位置各司其职）：
    ① 观察位（距桌心 ~1.15 m，能看到整张桌子）：调 /detect_grasp_target → 选一个可夹的目标
       （规则书 3.2/3.4：桌上四个物品任选一个），其余物品留作 obstacles
       ★ 观察位与站位的【朝向】都由 支撑面（餐桌）的桌沿法线 定，不由"机器人当前位姿"定：
         这样车头垂直桌沿、物体在机械臂正前方（"对正桌子"），
         也免得 AMCL 的定位误差 + Nav2 停位偏差把停车角度带偏几度~二十几度 ✗
         （实测踩过：按"机器人当前方位"算站位 → 站位 yaw = −1.155 rad，
          比桌沿法线 −1.571 偏 24° → 机械臂斜着伸到桌上）
    ② 站位（物体正前方 0.33~0.39 m，yaw = 法线反方向）→ 导航过去
    ③ 站位上【重新测一次】（相机与桌面等高，站位上整只物体可见 ✓），
       用相对量做 creep 微调，把物体开到 base_footprint(0.33, 0)
    ④ 调 /grasp_fixed_object（target + obstacles）；BAD_TARGET/NO_SOLUTION 就换下一个目标

★ 只用视觉（相机 + GroundingDINO/SAM2/INSID3 复核），**不读 gz 真值**：
  规则书禁止读仿真真值；本文件里也没有任何 `ign/gz model` 调用、没有"读不到就退真值"的退路。
  唯一的外部配置来源是 objects.yaml / grasp_params.yaml（碰撞箱尺寸、支撑面几何）。

★ 本模块**绝不调用 rclpy.spin\***：它在 patrol_task 的动作回调链里运行，
  再 spin 会死锁。所有等待都是"轮询 + time.sleep"，
  依赖调用方使用 MultiThreadedExecutor（订阅/服务回调在别的线程继续跑）。
"""

import math
import os
import time

import rclpy
import tf2_ros
from rclpy.action import ActionClient
from rclpy.time import Time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, LaserScan
from nav2_msgs.action import NavigateToPose

from turtlebot3_manipulation_grasp.msg import GraspTargetStamped
from turtlebot3_manipulation_grasp.srv import DetectGraspTarget, GraspFixedObject

# ★ 2026-09-16：支撑面门改成【只提示不拦截】
#   实测 map 侧位姿在扫视/导航期间会偏移 0.5 m 以上（同一罐子两次 map 位置差 0.54 m）
#   ⇒ 这道门会误杀真目标 → 整段抓取放弃 ✗。抓取本身走 base 系（可靠 ✓）；
#   "车开上桌子"的真正防线是站位的 nav-map 自由空间校验（仍强制 ✓）。
#   想恢复严格拦截：把下面改成 True
SUPPORT_GATE_ENFORCE = False


# ═══════════════════════════════════════════════════════════════
# 固定场景常量（map 帧 = Gazebo world 帧）
# ═══════════════════════════════════════════════════════════════

# 初始位姿（与 turtlebot3_franka.launch.py spawn 参数、patrol_task 一致）
INITIAL_X, INITIAL_Y, INITIAL_YAW = -5.30, -0.50, 0.0


# ── 观察位 / 站位的朝向：一律用【支撑面桌沿的法线】 ────────────────
# 桌沿法线（map 帧）：桌面在本场景里是轴对齐矩形（TABLE_YAW = ±π/2），
# 所以 4 条边对应 map 的 ±x / ±y 四个方向。选哪一条 = "机器人现在这一侧"，
# 判据只用【物体 → 机器人的方向】（不要求准，AMCL 差十几度也不影响选边）。
TABLE_FACE_NORMALS = ((1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0))
# 相邻两张餐桌（世界文件 example.world 的静态位姿，yaw=π/2 → 世界系半尺寸 x±0.6 / y±0.25）：
# 相机水平视野 62°，站在本桌前会把它们一起看进来 → 它们桌上的物体必须被判"不在本桌"
NEIGHBOR_TABLES = ((1.5, 2.0), (2.7, 1.5))     # dinning_table_1 / dinning_table_2
NEIGHBOR_HALF = (0.6, 0.25)
OBSERVATION_DIST = 1.15     # m，观察位到桌心的距离（Nav2 停偏 0.4 m 也还在画面内）
# ★ "近看"位：观察位太远时开集检测的置信度会掉到阈值以下 ✗
#   实测（2026-09-14）：站在观察位（离桌心 1.15 m、物体 0.9~1.5 m）时 7 个目标的
#   置信度只有 0.30~0.47 < 0.50，一个都不够格；而贴到站位（离物体 0.4 m）时同样是
#   mustard_bottle/sugar_box 能到 0.84 ✓。物体在 640x480 画面里只有几十像素 →
#   置信度是【距离】的函数。所以观察位一个都不够格时，再靠近到近看位重看一次。
# ★ 2026-09-18 改：近看位的距离【不再用"到桌心的距离"】—— 那是错的 ✗
#   餐桌脚印是 1.2 (map x) × 0.5 (map y) 的长方形（TABLE_HALF_X=0.6 / TABLE_HALF_Y=0.25），
#   同一个"离桌心 0.68 m"在长边（南/北）那一侧是桌沿外 0.43 m ✓，
#   在短边（东/西）那一侧只有桌沿外 **0.08 m** ✗✗
#   ⇒ 从东/西侧看时，近看位直接落进桌子脚印里、而且落进 Nav2 的 inscribed 区
#     （param: robot_radius = 0.28）⇒ DWB 判"非法轨迹"→ 到不了 → 反复重试，
#     最后把车顶到桌子上（用户现场看到的"第一次观察之后就开到餐桌另一边并撞桌子"）。
#   现在改成【按桌沿算】：近看位到【桌沿】CLOSE_LOOK_EDGE 米 —— 四个方向完全一致 ✓；
#   而且只用 cmd_vel 沿【同一条法线向前】走（不交给 Nav2、不绕到桌子另一边 ✓）
CLOSE_LOOK_EDGE = 0.42      # m，近看位到桌沿的距离（长边那一侧 = 原来的"离桌心 0.68 m"）
# ★ 多角度扫视：单个视场 62°（半角 31°）覆盖不了 1.2 m 长的桌子 ✗
#   （实测：罐子在 1.14 m 处只有 29×45 像素、检测器完全认不出 ✗）
#   ⇒ 在同一位置原地转几个角度各拍一次，再按 map 坐标合并（同一物体去重）✓
#   覆盖账（距离 d 处半宽 w 需要 atan(w/d)）：
#     观察位（离桌沿 0.90 m）：桌半宽 0.6 → 33.7°，直拍 31° 差一点 ⇒ ±25° 补扫 ✓
#     近看位（离桌沿 0.42 m，即离桌心 0.67 m）：桌半宽 0.6 → 41.9° ⇒ ±25° 补扫 ✓
#   ★ 顺序 = 0° 最前（直拍最正、定位最好），**找到能用的候选就停** ——
#     原来固定跑满 3 个角度 = 3 次视觉调用（~1 分钟），还把已经对正的目标推到
#     画面边缘去检测（边缘视角的检测/定位质量都更差）✗
SURVEY_YAWS = (0.0, -0.44, 0.44)     # rad；0° 最前，然后左右各 25°
# ★ 2026-09-19：改成 **默认跑满所有角度**（原来"扫到能用的就停" ✗）
#   现场实证（用户 2026-09-19）：桌上 4 个物体一字排开时，最靠边那个
#     potted_meat_can（map 2.30,2.15）从观察位看是 **偏轴 21.8°**，正好落在
#     62° 视场的**边缘**（半角 31°）—— 边缘的检测/分割质量最差 ⇒ 认不出来 ✗✗
#   而原来"直拍一认出中间那个罐子就停" ⇒ **那个能把它转到画面中心的 −25° 从来没拍** ✗
#   ⇒ 现在三个角度都拍：① 每个物体至少有一次落在画面中心附近 ✓
#                      ② 顺带把整桌的障碍物清单一并拿全（MTC 要用它避障）✓
#   时间代价：3 次视觉调用 ≈ 18 s（实测一次 ~6 s）✓ 完全付得起
SURVEY_ALL_ANGLES = True     # False = 回到"扫到能用即停"（省时间，但会漏掉边缘物体 ✗）
# ★ 近看位（离桌心 0.67 m）时整张桌子张角 ±43.9° ⇒ ±25° 只把两端带到 18.7°/7.4° 偏心
#   ⇒ 再加 ±45° 两档，四个物体都能落到画面中心 ±11° 内 ✓（离线账见
#     HANDOFF_harness/check_dining_view_geometry.py 第⑦节）
SURVEY_YAWS_NEAR = (0.0, -0.44, 0.44, -0.79, 0.79)   # 0° / ±25° / ±45°
SURVEY_MERGE_DIST = 0.08             # m，map 坐标相距 < 8 cm 视为同一个物体 ✓
CLOSE_MIN_CONFIDENCE = 0.25 # 近看这一趟放宽到这里（配合适配层的尺寸核对防误检）

# 臂基座（fr3_link0）在 base_footprint 下的 x 偏移（实测）。只用来看够不够得着。
ARM_BASE_X = -0.092

# 抓取服务返回的 stage → 人类可读（与 srv/GraspFixedObject.srv 的常量一一对应）
STAGE_TEXT = {
    0: "OK 完成",
    1: "NO_TARGET 没有可用目标",
    2: "BAD_TARGET 目标不合法（查表/桌面/可达性校验没过）",
    3: "SCENE_FAILED 场景没建起来（move_group / TF）",
    4: "INIT_FAILED MTC 任务初始化失败（配置问题）",
    5: "NO_SOLUTION 规划无解",
    6: "EXEC_FAILED 执行失败（控制器 / 碰撞）",
    7: "BUSY 已有抓取在执行",
}

# ── 目标优先级（见 docs/handoff/HANDOFF_grasp_orchestration_design.md §2）─────────
# 越靠前越稳：圆柱类位置换算误差≈0；方盒有 8~20 mm 的【径向】偏差 + 掀翻风险；
# 香蕉长边 0.198 必须跨窄边、薄件指尖容易骑到顶面 → 放最后。
TARGET_TIERS = [
    ["coke can", "tomato_soup_can", "chips_can"],                  # 圆柱
    ["apple", "potted_meat_can"],                                  # 球 / 矮罐
    ["cracker_box", "mustard_bottle", "bleach_cleanser", "sugar_box"],  # 高瘦方盒
    ["banana", "gelatin_box"],                                     # 长条 / 薄件
]
# 夹爪开口 0.08 m，这些类别的窄边 ≥ 0.08 → 物理上夹不住（见 objects.yaml）
NON_GRASPABLE = {"bowl", "pudding_box", "tuna_fish_can", "master_chef_can",
                 "pitcher_base", "beer", "windex_bottle"}

# ── 站位与 creep ─────────────────────────────────────────────────
STANDOFF_DIST = 0.33        # m，车心到物体（实测可稳定顶抓的几何）
# 相机在 base_footprint 前方多远（URDF: camera_joint x=0.073 + rgb 0.003 ≈ 0.076）
CAMERA_FWD = 0.08
# 相机碰撞盒在 0.7865~0.8135 m，桌面顶 0.795 m → 相机若伸到桌面上方就会撞桌板 ✗
# 所以站位要保证"相机仍在桌沿外"：g ≥ 物体纵深 + CAMERA_FWD + 余量
STANDOFF_EDGE_MARGIN = 0.05
# ★ 2026-09-18 改：原来 0.26 —— 比 Nav2 的 robot_radius **还小** ✗✗
#   param/turtlebot3.yaml: robot_radius = 0.28 ⇒ 距障碍 < 0.28 m 的栅格在代价地图里是
#   INSCRIBED_INFLATED_OBSTACLE(253)，而 DWB 的 BaseObstacleCritic 把 253 判为**非法轨迹**
#   ⇒ 站位（导航目标点）落在桌沿外 0.26 m 时，控制器永远到不了那个点：
#     一路"目标被拒/控制器失败→重试"，车头一直朝桌子顶，最后**物理上撞到桌面** ✗
#   （用户现场："观察之后导航到餐桌另一边并且撞桌子"；实测 free(0.28)=False）
#   现在取 robot_radius + 6 cm = 0.34 m ⇒ 目标点回到可合法到达的自由区 ✓
NAV_ROBOT_RADIUS = 0.28     # ★ 必须与 param/turtlebot3.yaml 的 robot_radius 同步（改一处要改两处）
EDGE_CLEARANCE = 0.34       # m，站位（导航目标）到桌沿的最小距离 = robot_radius + 余量
CREEP_GOAL_XY = (0.33, 0.0)  # 物体应落到的 base_footprint 位置
# ★ 容差按"指尖夹持窗口 ±6.5 mm"定（合爪前开口 0.08 − 罐宽 0.067）：
#   原来 bearing tol = 0.05 rad → 在 0.33 m 处等于 ±16.5 mm，比窗口大一倍多 ✗
#   方位容差 0.013 rad ≈ ±4.3 mm ✓；距离（径向）宽容，因为物体在该方向自身就有几 cm
CREEP_TOL = 0.008
CREEP_BEARING_TOL = 0.013
CREEP_OK_ROUNDS = 3         # 连续 N 轮都在容差内才收工（防被单帧噪声骗停）
CREEP_V_MAX, CREEP_W_MAX = 0.12, 0.60
CREEP_KV, CREEP_KW = 1.2, 1.5
CREEP_SPIN_BEARING = 0.35
CREEP_TIMEOUT = 45.0        # s（视觉每次 1~5 s，收紧容差后轮数变多，预算放宽）
CREEP_DT = 0.10
CMD_VEL_TOPIC = "/cmd_vel"  # navigation.launch.py 的 cmd_vel_relay 转到 /diff_controller

# ── 服务与超时 ───────────────────────────────────────────────────
VISION_SERVICE = "/detect_grasp_target"
GRASP_SERVICE = "/grasp_fixed_object"
# grasp_node（C++ 适配层）的节点名：★ 桌腿基准 要把修正后的 object_yaw_map 用
# {GRASP_NODE_NAME}/set_parameters 注入它（见 _inject_object_yaw）✓
GRASP_NODE_NAME = "/grasp_node"
NAV_ACTION = "/navigate_to_pose"
SERVICE_WAIT = 15.0
# ★ 视觉服务单次调用预算（2026-09-14 现场踩过）：
#   抓取模式 = 3 帧（18 类开集 + INSID3 闭集复核，每帧 ~15.5 s）+ 窄词表复核(11 s)
#   ≈ 60~75 s ⇒ 原来 60 s 的预算**经常超时** ✗
#   超时的后果很隐蔽：驱动退回"观察位（1 m 外）那次的估计"⇒ 抓取点偏十几厘米 ✗
#   ⇒ 机械臂看着"停在物体旁"却夹不到（现场就是这个现象）
VISION_CALL_TIMEOUT = 150.0     # 视觉单次调用预算（含 3 帧投票 + 窄词表复核）
GRASP_SERVICE_TIMEOUT = 900.0
MAX_NAV_RETRIES = 6
GOAL_ACCEPT_TIMEOUT = 15.0
TF_RETRY = 30
MIN_CONFIDENCE = 0.35           # 低于它不当作候选（开集检测在 1 m 外只有 0.35~0.46，
                                # 定 0.50 会把真目标全部拒掉 ✗；误检靠尺寸核对 +
                                # 支撑面校验 + 多帧投票压住，见 docs/handoff/HANDOFF_vision_grasp_interface.md）
REACH_MIN, REACH_MAX = 0.10, 0.75
MAX_TARGET_TRIES = 3            # 抓失败换目标的次数（规则书：四个里任选一个）
                                #   2026-09-18：2 → 3（位置型重试要占轮次，见 MAX_POSITION_ROUNDS）

# ═══════════════════════════════════════════════════════════════
# ★ 抓取闭环：用【夹到没有】当唯一判据，空合就换偏置重抓（2026-09-18）
# ═══════════════════════════════════════════════════════════════
# 为什么必须加这一段（用户 2026-09-14 起一直没解决的问题）：
#   "夹爪看着包住物体、也能闭合，物体纹丝不动" —— 而服务照样返回
#   success=true / stage=0 ✗（MTC 的 attach 只是**规划场景里的逻辑附着**，
#   空合也报成功，见 pick_and_place.cpp）。所以**不能拿 stage 当成功判据** ✓
#   唯一直接证据是"手指有没有被挡住"：
#     · 夹到 → 两指停在物体窄边（实测间距 ≈ 窄边，例：罐 66 mm）
#     · 空合 → 两指走到指令值（= 窄边 − 2×close_hand_squeeze，例：66−3 = 63 mm）
#   ⇒ 判据用"实测最小间距 − 指令间距"，它和 squeeze 取值无关，阈值固定 2.5 mm ✓
CONTACT_TOL_MM = 2.5        # mm；实测间距比指令高出这么多 ⇒ 手指被挡 = 夹到了 ✓
SPAN_CONTACT_MM = 1.5       # mm；实测间距落在"窄边 − 这个值"以内 ⇒ 也算被挡住 ✓（见 classify_closure）
# 空合后的重抓偏置阶梯（base_footprint 下：x = 朝物体方向，y = 侧向 = 两指合拢方向）
# ★ 顺序不是拍脑袋，依据 2026-09-16 的 sweep_offset 实验（.run/sweep_offset_*.log）：
#   只沿【径向】扫 ±40 mm **9 档全部空合** ⇒ 那次的主因不是"径向偏远" ✗
#   罐直径 66 mm、开口 80 mm ⇒ 侧向（合爪方向）只有 ±7 mm 余量：
#   侧向一偏，两指就从物体旁边过去，而且**任何径向偏置都救不回来** ✗
#   ⇒ 阶梯【先侧向 ±12/±24 mm】、再【侧向 + 径向 −20/−40 mm】组合 ✓
#   y 偏置 = 把抓取点整体横移 ⇒ 等效于"车/物体横向对齐"，
#   注意这不是"改类别"，只是同一物体的位姿微调 ✓
# ★ 2026-09-18 第二次实跑后定稿（两条新证据）：
#   ① 现场观察："夹爪落在番茄罐**前面一些**（靠桌面外侧）" ⇒ 残余偏浅只有 1~2 cm，
#      不再是糖盒那次的上百毫米（那个由"桌面剔除"补丁③ 修掉 ✓）
#   ② 用户观察："第一次抓取完之后，番茄罐发生了位移，导致后几次基本是空抓" ⇒
#      每档重抓之前必须**用视觉重测**（见 _grasp_with_offsets），否则是拿着过期点瞎试 ✗
#   ⇒ 阶梯幅度收小、并以【侧向】优先（罐的侧向余量只有 ±7 mm，是最窄的一维）✓
GRASP_RETRY_OFFSETS = ((0.000, 0.000),
                       (0.000, 0.012), (0.000, -0.012),    # 侧向（最窄的一维）
                       (0.020, 0.000), (-0.020, 0.000),    # 径向 ±20 mm
                       (0.000, 0.024), (0.000, -0.024),
                       (0.045, 0.000), (-0.045, 0.000),    # 径向 ±45 mm（较大的估计误差兜底）
                       (0.080, 0.000))
# 夹爪最大开口（两指间距，m）：fr3_hand 的 SRDF "open" = 每指 0.04 ⇒ 0.08 m。
#   用途：判"按当前合拢轴算，物体跨得过去吗"（见 call_grasp 的 ★ 合拢轴核对）✓
GRIPPER_OPEN_M = 0.080
# ★ "同一物体标签抖动"的判定半径（2026-09-19）：开集/闭集标签会逐帧变，位置几乎不变
#   （实测同一物体的两次检测差 2~6 cm）；而本桌上**不同物体**至少隔 0.25 m
#   ⇒ 10 cm 是个干净的界线：≤10 cm 才允许"用别的类别名代替位置"✓
#   现场教训（12:20）：半径 0.25/0.35 m 时，番茄罐没被认出，就用 **0.12 m 外**的
#   可乐罐检测代替 ⇒ 微调去对正可乐罐、最终抓取点又用了陈旧值 ⇒ 夹爪落在两个物体中间 ⇒ 抓空 ✗✗
LABEL_FLICKER_RADIUS = 0.10
# ── ★★ 桌腿基准（2026-09-19）：用激光扫到的桌腿反解"车在 map 里的真实偏航" ──
#   为什么需要：合拢轴不是量出来的，而是算出来的 ——
#       box_yaw = object_yaw_map(常数 0) − robot_yaw_map(TF/AMCL)      （grasp_node.cpp:408）
#   于是整条朝向链挂在 AMCL 的偏航上。而 AMCL 的偏航误差**没法靠动车站消除**：
#   车一转，belief 跟着转，误差 e = belief − truth 保持不变 ✗
#   ⇒ 必须用一条【不依赖 AMCL】的基准把它量出来，再补偿掉。
#   这里选的基准 = 餐桌的 4 条腿：它们在世界文件里的位置是真值（局部 ±0.235/±0.585，
#   截面 0.03×0.03），激光在 0.18 m 高、从桌沿外看得见 → 反解出车的 map 位姿（含偏航）✓
#   补偿方式：把 e = TF 偏航 − 桌腿反解偏航 写进 grasp_node 的 object_yaw_map，
#   则 box_yaw = (0 + e) − robot_yaw_map = −真实偏航 ⇒ 误差被抵消 ✓
SCAN_TOPIC = "/scan"
TABLE_LEG_LOCAL = ((-0.235, -0.585), (0.235, -0.585), (-0.235, 0.585), (0.235, 0.585))
LEG_RANGE_TOL = 0.15        # m；按【半径】匹配（半径对偏航不敏感 ⇒ 偏航偏 25° 也能对上 ✓）
LEG_BEARING_TOL = 0.61      # rad（35°）；方位窗（腿之间方位差 40°+ ⇒ 不会串台 ✓）
LEG_FACE_OFFSET = 0.015     # m；扫到的是腿的【近侧面】⇒ 沿视线往外补半个截面，靠回腿轴 ✓
YAW_ERR_USABLE = 0.60       # rad；反解出的偏差比它大 ⇒ 判不可信（宁可不用，也不乱改参数）
YAW_ERR_APPLY = 0.035       # rad（2°）；小于它就当没偏差（省一次参数写入，行为不变）
YAW_FIT_RMS_MAX = 0.020     # m；4 条腿拟合的残差超过它 ⇒ 判不可信
SCAN_MAX_AGE = 1.5          # s；激光数据比它旧就重新等一帧
MAX_GRASP_TRIES = 3         # 一个目标上最多试几个偏置（一个 MTC 周期实测 ~8~11 s）
                            #   ★ 2026-09-19：5 → 3。判据修好之后不需要那么多次；
                            #   而且万一"真夹住被误判"，少试几次就少重复几轮 pick&place ✓
# ── ★★ 跟随位移（2026-09-19）：只有"被推走"时才用，见 _grasp_with_offsets 的推导 ──
PLACE_CHECK = True          # 成功后量一次"放回误差"（多一次视觉调用 ~6 s ✓）
AMCL_JUMP_TOL = 0.05        # m；跟踪期间 AMCL 位姿跳这么多 ⇒ 不信里程计外推，强制重测 ✓
FOLLOW_MIN_MM = 0.008       # m；观测到物体位移超过它就认为"上一次碰到了它" ⇒ 位移方向可信 ✓
FOLLOW_GAIN = 1.0           # 下一档 = 上一档探点 + 增益×位移
                            #   ★ 2026-09-19：2.0 → 1.0。现场踩过：potted_meat_can 那次
                            #     第一次空合把物体推了 62 mm，×2 直接给出 +120 mm 偏置
                            #     ⇒ 第二次深了 180 mm，从"太靠外"一下跳到"太靠桌子内部" ✗✗
                            #     位移本身就已经是"被推走多少"，跟过去即可；宁可多走一步 ✓
FOLLOW_BIAS_MAX = 0.060     # m；修正偏置的幅值上限（再大就该怀疑是别的问题，不是慢慢试）✓
MAX_POSITION_ROUNDS = 1     # "位置型失败"（全空合/夹爪没动）时，重测后重试同一目标的轮数
                            #   ★ 时间账（实测一个 MTC 周期 ~11 s、一次视觉 ~6 s）：
                            #     一轮 ≈ 观察 6 s + 5 档抓取(55 s) + 4 次重测(24 s) ≈ 85 s
                            #     最坏 3+1 轮 ≈ 5~6 min；顺利时第 1 档就成功 ≈ 40 s ✓
                            #   限时紧就把它设 0（关掉"重测后重试同一目标"）✓


# ═══════════════════════════════════════════════════════════════
# ★ 合拢时刻判定（纯函数，离线单测：HANDOFF_harness/test_closure_phase.py）
# ═══════════════════════════════════════════════════════════════
# 为什么加这一行日志（2026-09-15 现场）：
#   日志里 `指尖轨迹` 报"本次最远 x 0.400"，`落点核对` 同时报"距契约点 0 mm" ——
#   这两条**并不矛盾**：落点核对只取【轨迹里最接近契约点的那一次采样】，
#   那可能只是路过的一瞬间 ✗（所以 0 mm 不能证明"夹爪停在了契约点"）
#   能真正区分"空合"与"正常抓取"的只有一个量：**两指开始合拢那一刻，指尖 x 在哪**
#     · 那一刻 x≈0.400（比契约点深 34 mm）⇒ 指腹已在罐子背面之后 ⇒ 空合 ✓ 自洽
#     · 那一刻 x≈0.366，而 0.400 出现在抬升/放回段 ⇒ 这条线索作废 ✗
#   判定只用【时间戳 + 间距序列】，不猜：
#     合拢开始 = 间距自张开值起累计减小 > CLOSE_DROP_MM
#     合拢结束 = 间距连续 STILL_HOLD_S 内变化 < STILL_TOL_MM
#     阶段归属 = "最大 x 那次采样"的序号 vs 合拢开始/结束的序号（时间序，不用 z 猜）
CLOSE_DROP_MM = 2.0     # mm，间距累计减小超过它 ⇒ 认定"开始合拢"
STILL_TOL_MM = 0.2      # mm，间距变化小于它 ⇒ 视为"手指不动"
STILL_HOLD_S = 0.5      # s，连续不动这么久 ⇒ 认定"合拢结束"


def analyze_closure(samples, close_drop_mm=CLOSE_DROP_MM,
                    still_tol_mm=STILL_TOL_MM, still_hold_s=STILL_HOLD_S):
    """判定"合拢开始/结束"时刻，并把"最大 x"归到 接近/合拢/之后 哪一段。

    samples = [(t, x, y, z, gap_mm), ...]，按时间升序；gap_mm 允许是 None
    （那一次 TF 没查到）。x/y/z = fr3_hand_tcp 在 base_footprint 里的位置。
    返回 dict；样本太少、或本次调用里根本没有合拢动作时返回 None（不硬凑结论 ✗）。
    """
    if len(samples) < 3:
        return None
    # ── 合拢开始：张开值取"到目前见过的最大间距"（容忍噪声上跳，不要求严格单调 ✓）
    #    并要求【连续两帧】都过阈值：单帧 TF 抖动跳一下不算合拢（这条日志就是用来
    #    定"合拢那一刻指尖在哪"的，误触发会把结论带偏 ✗）；记录的仍是第一帧过阈值的
    #    那一帧，只是等下一帧确认它没弹回去 ✓
    i_start, i_pend, ref_gap = None, None, None
    for i, s in enumerate(samples):
        g = s[4]
        if g is None:
            continue
        ref_gap = g if ref_gap is None else max(ref_gap, g)
        if ref_gap - g <= close_drop_mm:
            i_pend = None                    # 弹回张开值附近 ⇒ 上一次过阈值是抖动
            continue
        if i_pend is None:
            i_pend = i
            continue
        i_start = i_pend
        break
    if i_start is None:
        return None                       # 间距从没（连续两帧）累计减小 > 2 mm ⇒ 没合拢
    # ── 合拢结束：自开始起，间距连续 still_hold_s 内变化 < still_tol_mm
    i_end, end_estimated = None, False
    t_ref, g_ref, i_move = samples[i_start][0], samples[i_start][4], i_start
    for i in range(i_start + 1, len(samples)):
        t, g = samples[i][0], samples[i][4]
        if g is None:
            continue
        if abs(g - g_ref) >= still_tol_mm:
            t_ref, g_ref, i_move = t, g, i
        elif t - t_ref >= still_hold_s:
            i_end = i
            break
    if i_end is None:
        # 兜底：整个调用里没满足静止判据（TF 抖动 / 服务在判据满足前就返回）→
        # 用"最后一次间距明显变化"的时刻当结束，并在日志里标【估计】，不静默当成精确值 ✗
        i_end, end_estimated = i_move, True
    # ── 汇总
    #    ★ 合拢触发帧必然会滞后：2 mm 阈值 + 10 Hz 采样，间距关得快时一帧就差几 mm
    #     （间距 29 mm/s ⇒ 一帧 2.9 mm）⇒ 把"触发前一帧"也带上，
    #      这样"合拢刚启动时指尖在哪"就是 [前一帧 x, 触发帧 x] 这个区间，不含糊 ✓
    i_pre = next((i for i in range(i_start - 1, -1, -1) if samples[i][4] is not None),
                 i_start)
    gaps = [s[4] for s in samples[:i_end + 1] if s[4] is not None]
    gap_end = next((samples[i][4] for i in range(i_end, -1, -1)
                    if samples[i][4] is not None), None)
    xs_close = [samples[i][1] for i in range(i_start, i_end + 1)]
    i_maxx = max(range(len(samples)), key=lambda k: samples[k][1])
    t0 = samples[0][0]
    return {
        "i_start": i_start, "i_end": i_end, "end_estimated": end_estimated,
        "t_start": samples[i_start][0] - t0, "t_end": samples[i_end][0] - t0,
        "xyz_start": samples[i_start][1:4], "xyz_end": samples[i_end][1:4],
        "xyz_pre": samples[i_pre][1:4], "t_pre": samples[i_pre][0] - t0,
        "gap_pre": samples[i_pre][4],
        "gap_open": max(gaps) if gaps else None, "gap_end": gap_end,
        "x_close_min": min(xs_close), "x_close_max": max(xs_close),
        "x_max": samples[i_maxx][1], "z_at_maxx": samples[i_maxx][3],
        "t_maxx": samples[i_maxx][0] - t0,
        "phase": ("after" if i_maxx > i_end else
                  "during" if i_maxx >= i_start else "before"),
        "n": len(samples), "n_gap": sum(1 for s in samples if s[4] is not None),
    }


def format_closure_report(f, tgt_x, tgt_y):
    """把 analyze_closure 的结果排成可直接粘贴的日志行（列表，1~2 行，纯字符串）。"""
    if f is None:
        return ["  ★ 合拢时刻: 没采到足够的间距/指尖样本（或本次调用里没有合拢动作）"
                "→ 无法判定 ✗"]
    x0, y0, z0 = f["xyz_start"]
    xe, ye, ze = f["xyz_end"]
    dx, dy = (x0 - tgt_x) * 1000.0, (y0 - tgt_y) * 1000.0
    dxe, dye = (xe - tgt_x) * 1000.0, (ye - tgt_y) * 1000.0
    line = ("  ★ 合拢时刻: 指尖 base({:+.3f},{:+.3f},{:.3f}) 距契约点"
            " (Δx={:+.0f} mm, Δy={:+.0f} mm) | 合拢前 {} → 结束 {} | "
            "合拢期间 x 范围 [{:.3f}, {:.3f}]".format(
                x0, y0, z0, dx, dy,
                "?" if f["gap_open"] is None else "{:.1f} mm".format(f["gap_open"]),
                "?" if f["gap_end"] is None else "{:.1f} mm".format(f["gap_end"]),
                f["x_close_min"], f["x_close_max"]))
    # ── 阶段归属：最大 x 落在【合拢之前/期间/结束之后】哪一段（按采样序号=时间序）
    xm, ts, te, tm = f["x_max"], f["t_start"], f["t_end"], f["t_maxx"]
    dm = (xm - tgt_x) * 1000.0
    if f["phase"] == "during":
        line += ("（本次最大 x {:.3f} 出现在合拢【期间】（t=+{:.1f} s，开始 t=+{:.1f} s / "
                 "结束 t=+{:.1f} s）✗ ⇒ Δx={:+.0f} mm 的深偏置确实发生在合拢时刻，"
                 "不是路过）".format(xm, tm, ts, te, dm))
    elif f["phase"] == "after":
        line += ("（本次最大 x {:.3f} 出现在合拢结束【之后】（t=+{:.1f} s > 结束 t=+{:.1f} s；"
                 "z {:.3f}→{:.3f}{}）⇒ 属抬升/放回，与抓取无关）".format(
                     xm, tm, te, z0, f["z_at_maxx"],
                     " 在上升" if f["z_at_maxx"] > z0 else " 未上升"))
    else:
        line += ("（本次最大 x {:.3f} 出现在合拢【之前】（接近段，t=+{:.1f} s < 开始 t=+{:.1f} s）"
                 "⇒ 与合拢时刻无关，合拢时刻看本行前面的 x）".format(xm, tm, ts))
    return [line,
            "  合拢结束: 指尖 base({:+.3f},{:+.3f},{:.3f}) 距契约点 (Δx={:+.0f} mm, Δy={:+.0f} mm)"
            "｜合拢前最后一帧 x={:+.3f}（{} mm, t=+{:.1f} s）→ 触发帧 x={:+.3f}"
            "（2 mm 阈值滞后 1 帧 ⇒ 合拢刚启动时的 x 在这两者之间）"
            "｜合拢段 {:.1f} s、采样 {} 次（间距有效 {} 次）{}".format(
                xe, ye, ze, dxe, dye,
                f["xyz_pre"][0],
                "?" if f["gap_pre"] is None else "{:.1f}".format(f["gap_pre"]),
                f["t_pre"], x0, te - ts, f["n"], f["n_gap"],
                "（⚠ 结束时刻为估计值：没满足 {:.1f} s 静止判据）".format(STILL_HOLD_S)
                if f["end_estimated"] else "")]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def table_legs_map(center_xy, table_yaw, legs_local=TABLE_LEG_LOCAL):
    """餐桌 4 条腿在 map 里的 (x, y) —— 世界文件真值（不是估的）。

    世界文件 `example.world` 的 `dinning_table_3`：桌面 0.5×1.2、4 条 0.03×0.03 的腿
    位于桌子局部 (±0.235, ±0.585)（模型自带 pose 已由调用方从世界文件读出并传入 ✓）。
    """
    c, s = math.cos(table_yaw), math.sin(table_yaw)
    return [(center_xy[0] + c * lx - s * ly, center_xy[1] + s * lx + c * ly)
            for (lx, ly) in legs_local]


def fit_pose_from_landmarks(meas_base, ref_map):
    """两组一一对应的 2D 点（meas = base 系实测，ref = map 系已知）→ (x, y, yaw, rms)。

    求使 `R(yaw)·meas + t ≈ ref` 的位姿 = **车在 map 里的位姿**（因为 meas 是车体系量到的、
    ref 是 map 里已知的同一批点 ✓）。闭式解（2D Kabsch，纯 math，不用 numpy）：

        先各自减质心 ⇒ yaw = atan2(Σ(rx·my − ry·mx), Σ(rx·mx + ry·my))
        t = ref 质心 − R(yaw)·meas 质心

    为什么用它：这是**唯一**能不依赖 AMCL 拿到"车在 map 里的偏航"的办法 ✓
    （AMCL 的偏航误差是常量偏置，动车站消不掉，见 SCAN_TOPIC 那段说明）
    """
    n = len(meas_base)
    if n < 3 or n != len(ref_map):
        return None
    mx = sum(p[0] for p in meas_base) / n
    my = sum(p[1] for p in meas_base) / n
    rx = sum(p[0] for p in ref_map) / n
    ry = sum(p[1] for p in ref_map) / n
    num = den = 0.0
    for (bx, by), (px, py) in zip(meas_base, ref_map):
        ax, ay = bx - mx, by - my            # 实测点（减质心）
        cx_, cy_ = px - rx, py - ry          # 已知点（减质心）
        num += ax * cy_ - ay * cx_           # Σ (r × m)
        den += ax * cx_ + ay * cy_           # Σ (r · m)
    yaw = math.atan2(num, den)
    c, s = math.cos(yaw), math.sin(yaw)
    tx = rx - (c * mx - s * my)
    ty = ry - (s * mx + c * my)
    err = 0.0
    for (bx, by), (px, py) in zip(meas_base, ref_map):
        ex = c * bx - s * by + tx - px
        ey = s * bx + c * by + ty - py
        err += ex * ex + ey * ey
    return (tx, ty, yaw, math.sqrt(err / n))


def legs_from_scan(scan_pts_base, legs_map, robot_pose_believed,
                   range_tol=LEG_RANGE_TOL, bearing_tol=LEG_BEARING_TOL,
                   face_offset=LEG_FACE_OFFSET):
    """把激光点分配给 4 条桌腿 → (每条腿的实测 base 位置, 命中下标, 未命中下标)。

    ★ 匹配为什么用【半径】而不是【位置】：本函数的目的就是量出 belief 的偏航误差 e，
      而**半径对偏航不敏感**（r = |腿 − 车|，只跟车的**位置**有关 —— AMCL 位置通常只差几厘米），
      用位置匹配时 25° 的偏航误差会把 0.9 m 处的腿预测偏 0.39 m ⇒ 全都对不上 ✗
      （实测：e=+25.7° 时 4 条腿只对上 1 条 ⇒ 反解直接失败）。
      腿之间的半径差 ≥0.27 m（0.38/0.85/1.12/1.38 m）⇒ ±0.15 m 的半径窗不会歧义 ✓
      再加一个 ±35° 的方位窗防串台（腿之间方位差 40°+ ✓）。
    ★ face_offset：激光打到的是腿的**近侧面**，比腿轴靠近车约半个截面（15 mm）⇒
      沿"车→点"方向往外推 15 mm，靠回腿轴（对偏航影响很小，对位置影响 ~15 mm ✓）
    """
    cx, cy, ct = robot_pose_believed
    c, s = math.cos(ct), math.sin(ct)
    got, idx, miss = [], [], []
    for k, (lx, ly) in enumerate(legs_map):
        dx, dy = lx - cx, ly - cy                       # map 里的差矢量
        r_pred = math.hypot(dx, dy)
        b_pred = math.atan2(-s * dx + c * dy, c * dx + s * dy)   # 预测方位（base 系）
        hit = []
        for (px, py) in scan_pts_base:
            r = math.hypot(px, py)
            if abs(r - r_pred) > range_tol:
                continue
            db = math.atan2(math.sin(math.atan2(py, px) - b_pred),
                            math.cos(math.atan2(py, px) - b_pred))
            if abs(db) > bearing_tol:
                continue
            hit.append((px, py))
        if len(hit) < 1:
            miss.append(k)
            continue
        hx = sum(p[0] for p in hit) / len(hit)
        hy = sum(p[1] for p in hit) / len(hit)
        # 沿"车→点"方向往外补半个腿截面（face_offset 天然带方向 ✓）
        r = math.hypot(hx, hy)
        f = (r + face_offset) / r if r > 1e-6 else 1.0
        got.append((hx * f, hy * f))
        idx.append(k)
    return got, idx, miss


def closing_span_mm(depth_m, width_m, phi_rad):
    """合拢轴偏了 φ 时，长方体在【合拢方向】上的投影宽度（mm）= 两指需要张开的尺寸。

    为什么需要这个式子（2026-09-19 potted_meat_can "停在物体顶上、合爪合空"）：
        C++ 侧的 box_yaw = object_yaw_map(常数 0) − robot_yaw_map(TF) ⇒ 合拢轴是**算出来的**，
        整条朝向链挂在 TF 的 map←base_footprint 偏航上。这个偏航偏 φ，合拢轴就跟着偏 φ，
        而长方体在合拢方向的投影是
            span(φ) = depth·|cos φ| + width·|sin φ|
        ⇒ 物体越"扁长"，φ 造成的开口膨胀越快：
            potted_meat_can 50×97 mm：φ=0 → 50.0；15° → 73.4；19.8° → 80.2（= 开口上限）
            cracker_box   60×158 mm：φ=0 → 60.0；10° → 86.5 ⇒ 一偏就超过 80 ✗
        ⇒ 只要 span > 开口，两指**几何上跨不进去**：会压在物体顶面把它推走，
          然后"合到指令间距而两指之间是空的"（现场"夹爪合的也宽"）✓
    """
    return 1000.0 * (float(depth_m) * abs(math.cos(phi_rad))
                     + float(width_m) * abs(math.sin(phi_rad)))


def classify_closure(commanded_mm, measured_mm, span_mm=None, open_mm=None,
                     tol_mm=CONTACT_TOL_MM):
    """判"这次合爪到底夹到东西没有" → ("contact"|"empty"|"unclosed"|"unknown", 说明)。

    输入都是毫米：
      commanded_mm = 本次发给夹爪的指令间距（= 窄边 − 2×close_hand_squeeze）×1000
      measured_mm  = 合爪过程中实测到的【最小两指间距】（TF: leftfinger↔rightfinger）
      open_mm      = 合爪【前】的张开间距（≈80 mm）：传了它才能识别"根本没合爪"那一档 ✓
      span_mm      = 物体窄边（可选，只用于日志/交叉核对，不参与判据）

    ★ 为什么判据是"实测 − 指令"而不是"实测 vs 窄边"：
      squeeze 是**每指挤入量**（现场在 0.0015 与 0.0045 之间调过），
      "空合"的实测值 = 指令值，而"夹到"的实测值 ≈ 窄边 ——
      两者之差恰好 = 2×squeeze，会随参数漂 ✗；用"实测 − 指令"则与参数无关：
        空合 → 差 ≈ 0（手指走满，中间没东西）
        夹到 → 差 ≈ 2×squeeze（手指被挡住，走不满）
      顺手把"实测 vs 窄边"也打出来，方便人核对 ✓
    ★ 2026-09-18 补 "unclosed"（现场日志抓到的真 bug）：
      偏置重抓第 2 档时 MTC 直接 stage=5（规划无解），夹爪**一动没动**（间距全程 80 mm），
      而"实测 80 − 指令 35 = +45 mm"被旧判据当成"手指被挡住 = 夹到了" ✗✗
      ⇒ 驱动据此报"抓取成功"，实际连夹爪都没合上。
      现在先要求"确实合过爪"：实际行程 = 张开值 − 实测最小间距，指令行程 = 张开值 − 指令值，
      实际行程 < 25% 指令行程 ⇒ "unclosed"（规划失败/夹爪没动），**不算夹到** ✗
      （只在"张开值 − 指令值 > 5 mm"时启用这条：本来就合着的时候不适用 ✓）
    ★ 已知边界：squeeze 取到 0.0015 时"空合"与"夹到"只差 3 mm，
      和默认阈值 2.5 mm 贴得很近 ⇒ 判别余量小。
      想要更稳的判别力就把 grasp_params.yaml 的 close_hand_squeeze 调到 ≥0.004
      （两档差 ≥8 mm），代价是每指多挤 4 mm（现场记录 0.0070 会把薄盒挤跑）。
    """
    if commanded_mm is None or measured_mm is None:
        return "unknown", "没采到间距/指令值"
    extra = "" if span_mm is None else "；实测 {:.1f} vs 窄边 {:.1f} mm".format(
        measured_mm, span_mm)
    if open_mm is not None:
        travel_cmd = float(open_mm) - float(commanded_mm)
        travel_done = float(open_mm) - float(measured_mm)
        if travel_cmd > 5.0 and travel_done < 0.25 * travel_cmd:
            return "unclosed", ("夹爪几乎没动：张开 {:.1f} → 最小 {:.1f} mm（指令 {:.1f} mm；"
                                "只走了 {:.1f}/{:.1f} mm）⇒ 合爪没真正发生（多半是规划/执行"
                                "失败）**不算夹到** ✗{}".format(
                                    open_mm, measured_mm, commanded_mm,
                                    travel_done, travel_cmd, extra))
    d = float(measured_mm) - float(commanded_mm)
    # ★★ 2026-09-19 加的第三判据【抓错物体】（现场教训，代价很大）：
    #   那一趟调用方要 potted_meat_can(窄边 50 mm)，实际夹住的是番茄罐(66 mm)：
    #   实测最小间距 61.3 mm，比"目标窄边 50 mm"宽了 +11.3 mm
    #   —— 手指确实被挡住了（物体比指令宽得多）⇒ 旧判据报 [contact] ✓ 抓取成功 ✗✗
    #   ⇒ "被挡住"不等于"夹的是目标类别"：实测间距**比目标窄边明显宽**就是抓错了对象 ✓
    #   阈值 4 mm：夹爪重复性 ~0.1 mm、目录尺寸 ±2 mm、squeeze 1.5 mm ⇒ 4 mm 已足够松 ✓
    if span_mm is not None and measured_mm > float(span_mm) + 4.0:
        return "wrong_object", ("实测最小间距 {:.1f} mm 比目标类别窄边 {:.1f} mm **宽 {:+.1f} mm**"
                                " ⇒ 夹住的是更宽的东西 = **抓错物体了**（不是这一类）✗{}".format(
                                    measured_mm, span_mm, measured_mm - float(span_mm), extra))
    # ★ 2026-09-19 加的第二判据（现场实证：一次真夹住只比指令高 2.9 mm，阈值 2.5 mm ⇒ 只差 0.4 mm ✗）：
    #   手指停在**物体窄边附近**（≥ 窄边 − 1.5 mm）本身就是"被挡住"的直接证据 ✓
    #   为什么需要：close_hand_squeeze=0.0015 时"空合"= 窄边−3 mm，"夹到"≈ 窄边 ⇒
    #   两者只差 3 mm，落在阈值两侧 ⇒ 真夹住也会被判成空合 ✗ ⇒ 驱动以为失败 ⇒
    #   **再抓一次**（一整轮 pick&place）⇒ 现场看到的就是"成功之后还在不停重复抓取流程" ✗✗
    if (span_mm is not None and measured_mm >= float(span_mm) - SPAN_CONTACT_MM
            and (open_mm is None
                 or (float(open_mm) - float(measured_mm)) >= 0.25 * (float(open_mm) - float(commanded_mm)))):
        return "contact", ("实测最小间距 {:.1f} mm 已停在物体窄边 {:.1f} mm 附近（指令 {:.1f} mm）"
                           " ⇒ 手指被物体挡住 = **夹到了** ✓{}".format(
                               measured_mm, span_mm, commanded_mm, extra))
    if d >= tol_mm:
        return "contact", ("实测最小间距 {:.1f} mm 比指令 {:.1f} mm 高 {:+.1f} mm"
                           " ⇒ 手指被挡住 = **夹到了** ✓{}".format(
                               measured_mm, commanded_mm, d, extra))
    if d <= -tol_mm:
        return "empty", ("实测最小间距 {:.1f} mm 比指令 {:.1f} mm 还低 {:+.1f} mm"
                         " ⇒ 异常（合过头了？）✗{}".format(
                             measured_mm, commanded_mm, d, extra))
    return "empty", ("实测最小间距 {:.1f} mm ≈ 指令 {:.1f} mm（差 {:+.1f} mm）"
                     " ⇒ **两指之间是空的，没夹到东西** ✗{}".format(
                         measured_mm, commanded_mm, d, extra))


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quat_rotate(q, v):
    qx, qy, qz, qw = q
    vx, vy, vz = v
    c = (qy * vz - qz * vy, qz * vx - qx * vz, qx * vy - qy * vx)
    c2 = (qy * c[2] - qz * c[1], qz * c[0] - qx * c[2], qx * c[1] - qy * c[0])
    return (vx + 2 * qw * c[0] + 2 * c2[0],
            vy + 2 * qw * c[1] + 2 * c2[1],
            vz + 2 * qw * c[2] + 2 * c2[2])


def find_grasp_config(name):
    """找抓取包的 config/<name>（objects.yaml / grasp_params.yaml）。

    本脚本在 nav2 包里，配置在 grasp 包里 → 先看源码树，再按 AMENT_PREFIX_PATH 找 share。
    """
    here = os.path.dirname(os.path.realpath(__file__))
    cands = [os.path.join(os.path.dirname(os.path.dirname(here)),
                          "turtlebot3_manipulation_grasp", "config", name)]
    for prefix in (os.environ.get("AMENT_PREFIX_PATH") or "").split(os.pathsep):
        if prefix:
            cands.append(os.path.join(prefix, "share", "turtlebot3_manipulation_grasp",
                                      "config", name))
    for c in cands:
        if os.path.isfile(c):
            return c
    return None


def load_close_squeeze(default=0.0045):
    """读 grasp_params.yaml 的 close_hand_squeeze（驱动侧只用它算"预期合爪间隙"）。

    ★ 必须读文件而不是在驱动里再写一套公式：C++ 与驱动各写一套就会漂移 ✗
    """
    path = find_grasp_config("grasp_params.yaml")
    if not path:
        return default
    try:
        import yaml
        doc = yaml.safe_load(open(path, encoding="utf-8")) or {}
        node = (doc.get("/**") or {}).get("ros__parameters") or {}
        v = node.get("close_hand_squeeze")
        return float(v) if v is not None else default
    except Exception:
        return default


def load_object_sizes():
    """读 objects.yaml 的三维尺寸 → {class_id: (depth, width, height)}（读不到返回 {}）。

    用途：驱动侧要知道"这次合爪应该合到多少"，才能判断实测行程是不是夹到了物体 ✓
    """
    path = find_grasp_config("objects.yaml")
    if not path:
        return {}
    try:
        import yaml
        doc = yaml.safe_load(open(path, encoding="utf-8")) or {}
        out = {}
        for k, v in (doc.get("objects") or {}).items():
            if isinstance(v, dict) and all(x in v for x in ("depth", "width", "height")):
                out[k] = (float(v["depth"]), float(v["width"]), float(v["height"]))
        return out
    except Exception:
        return {}


# 支撑面（要抓的物体所在的那张桌子）：**从 grasp_params.yaml 读**，不再写死 ✓
#   为什么必须同源：抓取侧（grasp_node）也是读这个文件 ✓，两处各写一份就会漂移 ✗
#   （原来这里硬编码 dinning_table_3，换张桌子就得改两处代码 ✗）
def load_support_surface():
    """→ (center_xyz, yaw, dims, half_x, half_y, top_z)；读不到就用餐厅那张桌子。"""
    cx, cy, cz, yaw, dims = 2.7, 2.0, 0.765, math.pi / 2, (0.5, 1.2, 0.03)
    path = find_grasp_config("grasp_params.yaml")
    if path:
        try:
            import yaml
            doc = yaml.safe_load(open(path, encoding="utf-8")) or {}
            n = (doc.get("/**") or {}).get("ros__parameters") or {}
            pose = n.get("support_surface.pose")
            if pose and len(pose) >= 3:
                cx, cy, cz = float(pose[0]), float(pose[1]), float(pose[2])
                yaw = float(pose[5]) if len(pose) > 5 else 0.0
            if n.get("support_surface.length") and n.get("support_surface.width"):
                dims = (float(n["support_surface.length"]),
                        float(n["support_surface.width"]),
                        float(n.get("support_surface.thickness", 0.03)))
        except Exception:
            pass
    half_x, half_y = (dims[1] / 2.0, dims[0] / 2.0) if abs(yaw) > 1.0 else (dims[0] / 2.0, dims[1] / 2.0)
    return (cx, cy, cz), yaw, dims, half_x, half_y


TABLE_CENTER_XYZ, TABLE_YAW, TABLE_DIMS, TABLE_HALF_X, TABLE_HALF_Y = load_support_surface()
TABLE_TOP_Z = TABLE_CENTER_XYZ[2] + TABLE_DIMS[2] / 2.0     # 桌面顶
TABLE_IS_DINING = (abs(TABLE_CENTER_XYZ[0] - 2.7) < 0.2 and abs(TABLE_CENTER_XYZ[1] - 2.0) < 0.2)


def load_graspable():
    """读 objects.yaml 的 graspable 标志 → {class_id: bool}（读不到返回 {}）。

    为什么要读它而不是在代码里写死名单：能夹什么由**碰撞箱尺寸 + 夹爪开口**决定，
    那是 objects.yaml 的职责（抓取侧查表用的也是它）→ 一处维护，别两处 ✗
    """
    path = find_grasp_config("objects.yaml")
    if not path:
        return {}
    try:
        import yaml
        doc = yaml.safe_load(open(path, encoding="utf-8")) or {}
        return {k: bool(v.get("graspable", False))
                for k, v in (doc.get("objects") or {}).items() if isinstance(v, dict)}
    except Exception:
        return {}


# ═══════════════════════════════════════════════════════════════
# 导航地图（判"这个点是不是空闲空间"）
# ═══════════════════════════════════════════════════════════════
# ★ 为什么要读地图：选"从哪条桌沿接近"时，光看"机器人现在在物体哪一侧"是不够的 ✗
#   实测踩过：机器人从起点 (-5.30,-0.50) 出发时，物体方向是"西边" → 选西侧桌沿
#   → 观察位算成 (1.55, 2.0)，而那里是 dining_table_1 的桌面里 ✗✗
#   （餐桌是并排的：table_1 x∈[0.9,2.1]、table_2 y∈[1.25,1.75]、table_3 在中间）
#   地图里这些桌子都是障碍 → 直接查地图就知道哪一面站得下人 ✓

ROBOT_CLEAR_OBS = 0.32      # m，观察位要求周围这么空（车半径 0.28 + 余量）
# ★ 2026-09-18 改：0.10 → 0.30。原来只判"别压在桌子上"，可是**导航目标点**要满足的不是
#   "车心不在障碍里"，而是"Nav2 能合法地把车开到这个点"：DWB 要求 ≥ robot_radius(0.28)。
#   实测：站位被外推到桌沿外 0.26 m 时 free(0.28)=False（=目标点在 inscribed 区，
#   控制器到不了 → 反复重试 → 车头顶到桌子上）✗；0.34 m 时 free(0.28)=True ✓
ROBOT_CLEAR_PARK = 0.30     # m，站位（导航目标点）要求周围这么空 ≈ NAV_ROBOT_RADIUS + 余量


def find_nav_map():
    """找导航地图 map.yaml（本包 map/ 下；装好的 share 里也找一遍）。"""
    here = os.path.dirname(os.path.realpath(__file__))
    cands = [os.path.join(os.path.dirname(here), "map", "map.yaml")]
    for prefix in (os.environ.get("AMENT_PREFIX_PATH") or "").split(os.pathsep):
        if prefix:
            cands.append(os.path.join(prefix, "share", "turtlebot3_manipulation_navigation2",
                                      "map", "map.yaml"))
    for c in cands:
        if os.path.isfile(c):
            return c
    return None


class NavMap:
    """占用栅格（map.yaml + map.pgm），只用来回答"点 (x,y) 周围空不空"。"""

    def __init__(self, yaml_path):
        import yaml
        doc = yaml.safe_load(open(yaml_path, encoding="utf-8")) or {}
        self.res = float(doc["resolution"])
        self.ox, self.oy = float(doc["origin"][0]), float(doc["origin"][1])
        self.negate = bool(doc.get("negate", 0))
        self.occupied_thresh = float(doc.get("occupied_thresh", 0.65))
        path = os.path.join(os.path.dirname(yaml_path), doc["image"])
        with open(path, "rb") as fh:
            blob = fh.read()
        # 手写 P5 解析（不依赖 PIL）：magic / 宽 高 / maxval，其间可能有 '#' 注释
        fields, pos = [], 0
        while len(fields) < 4:
            while blob[pos:pos + 1].isspace():
                pos += 1
            if blob[pos:pos + 1] == b"#":
                while blob[pos:pos + 1] not in (b"\n", b""):
                    pos += 1
                continue
            start = pos
            while not blob[pos:pos + 1].isspace():
                pos += 1
            fields.append(blob[start:pos])
        if fields[0] not in (b"P5", b"P2"):
            raise ValueError("只支持 P5/P2 PGM: " + repr(fields[0]))
        self.w, self.h, maxval = int(fields[1]), int(fields[2]), int(fields[3])
        pos += 1
        if fields[0] == b"P5":
            import numpy as np
            self.pix = np.frombuffer(blob[pos:pos + self.w * self.h], dtype="u1")
        else:
            import numpy as np
            self.pix = np.array(blob[pos:].split()[:self.w * self.h], dtype="u1")
        self.pix = self.pix.reshape(self.h, self.w).astype("float32") / float(maxval)

    def occupied(self, x, y):
        col = int((x - self.ox) / self.res)
        row = self.h - 1 - int((y - self.oy) / self.res)     # 图像第 0 行是地图最上方
        if col < 0 or row < 0 or col >= self.w or row >= self.h:
            return True                                     # 界外当障碍
        p = self.pix[row, col]
        occ = p if self.negate else 1.0 - p
        return occ > self.occupied_thresh

    def free(self, x, y, clearance=0.0):
        """(x,y) 周围 clearance 半径内没有一个占用栅格 → 空闲。"""
        if clearance <= 0.0:
            return not self.occupied(x, y)
        step = max(self.res, 0.05)
        n = int(math.ceil(clearance / step))
        for i in range(-n, n + 1):
            for j in range(-n, n + 1):
                if math.hypot(i * step, j * step) > clearance + 1e-9:
                    continue
                if self.occupied(x + i * step, y + j * step):
                    return False
        return True


class GraspPhase:
    """抓取阶段：选目标 → 站位 → creep → 抓取。可被任何 Node 复用。"""

    def __init__(self, node, *, target_classes=None, min_confidence=MIN_CONFIDENCE,
                 nav_client=None, cmd_vel_topic=CMD_VEL_TOPIC):
        self.node = node
        self.log = node.get_logger()
        self.min_confidence = min_confidence
        # 想找哪些类别（空 = 不限，按 TARGET_TIERS 自己挑）
        self.target_classes = list(target_classes or [])
        self.tf_buffer = getattr(node, "_tf_buffer", None)
        if self.tf_buffer is None:
            # ★ 调用方可能没有 TF 缓冲（dining_grasp_task.py 就没有）→ 自己建一个。
            #   没有它就拿不到 map←base_footprint：观察位、支撑面校验、站位全部算不出来 ✗
            #   （实测：dining_grasp_task 直接报"拿不到 map←base_footprint，算不出观察位"）
            self.tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)
        self.vision_client = node.create_client(DetectGraspTarget, VISION_SERVICE)
        self.grasp_client = node.create_client(GraspFixedObject, GRASP_SERVICE)
        self.cmd_pub = node.create_publisher(Twist, cmd_vel_topic, 10)
        # 复用调用方的 Nav2 客户端（patrol_task 里已经有），没有就自己建
        self.nav_client = nav_client or ActionClient(node, NavigateToPose, NAV_ACTION)
        self.last_targets = []       # 最近一次检测结果（给上层/证据用）
        # ★ 观察那一趟定下的【桌沿法线】：近看位前进、站位解算全程复用它 ✓
        #   （绝不由"目标位置 → 机器人"重算 —— 幻影目标会把法线翻到桌子另一边 ✗）
        self.obs_normal = None
        # ★ 手指关节实测位置：判断"到底有没有夹到物体"的唯一直接证据
        #   （仿真里合爪是位置控制：夹到物体就会停在物体宽度处、到不了指令值 ✓）
        self._finger_q = None
        self._finger_q2 = None      # fr3_finger_joint2（第二指；见 _on_joint_state 的说明）
        self.object_sizes = load_object_sizes()
        self.close_squeeze = load_close_squeeze()
        # ★ 注意：GraspPhase 不是 Node（只是拿着调用方的 node）→ 必须用 self.node.* ✗
        self.node.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        # ★★ 桌腿基准（2026-09-19）：/scan 用来反解"车在 map 里的真实偏航"，
        #   再把偏差注入 grasp_node 的 object_yaw_map（合拢轴的 AMCL 误差补偿，见 _table_yaw_probe）
        self._scan = None
        self._last_yaw_inject = None
        self._table_legs_map = None
        self.node.create_subscription(LaserScan, SCAN_TOPIC, self._on_scan, 10)
        # 桌腿基准的 map 真值：直接用已解析好的支撑面位姿（见 load_support_surface ✓）
        self._table_legs_map = table_legs_map((TABLE_CENTER_XYZ[0], TABLE_CENTER_XYZ[1]), TABLE_YAW)
        self._track = {}             # 类别 → (时间, odom 坐标)：creep 的里程计跟踪基准
        # 能夹什么以 objects.yaml 为准（读不到才退回代码里的兜底名单）
        self.graspable = load_graspable()
        # ★ 默认请求的类别全集（objects.yaml 的全部 key）——见 fetch_targets 的说明 ✓
        #   ⚠ 必须放在 self.graspable 之后 ✗
        #   （2026-09-19 我把它写在了前面 ⇒ `GraspPhase' object has no attribute 'graspable'
        #     ⇒ 构造就抛异常、整个 Phase 2 直接不跑 —— 现场"改完之后没法运行正常任务"就是它；
        #     冒烟测试见 HANDOFF_harness/test_grasp_phase_ctor.py ✓）
        self.all_classes = sorted(self.graspable) if self.graspable else []
        if self.graspable:
            ok = [k for k, v in self.graspable.items() if v]
            self.log.info("objects.yaml 里可夹类别 {} 个：{}".format(len(ok), ", ".join(ok)))
        else:
            self.log.warn("读不到 objects.yaml 的 graspable 标志，退回内置名单")
        # 导航地图：用来判"观察位/站位是不是空闲空间"（见 NavMap 的说明）
        self.nav_map = None
        mp = find_nav_map()
        if mp:
            try:
                self.nav_map = NavMap(mp)
                self.log.info("已载入导航地图 {}（{}x{} @ {:.2f} m/格）→ 站位/观察位会查空闲空间"
                              .format(mp, self.nav_map.w, self.nav_map.h, self.nav_map.res))
            except Exception as e:                       # noqa: BLE001
                self.log.warn("解析地图失败（{}: {}）→ 站位只按餐桌脚印判".format(
                    type(e).__name__, e))
        else:
            self.log.warn("找不到导航地图 → 站位只按餐桌脚印判（不查全局空闲空间）")

    def _free(self, x, y, clearance):
        return True if self.nav_map is None else self.nav_map.free(x, y, clearance)

    def _on_joint_state(self, msg):
        """抓两个手指的实测位置（**两个都要**，2026-09-18 修 mimic 后才有意义）。

        ★ 为什么不能只看 joint1：Fortress 不支持 URDF mimic ⇒ 2026-09-18 之前
          joint2 是"被动关节、停在闭合端"，只有 joint1 在动；那时
          `两指真实间距`（TF 量）是**运动学算的假象**（2×joint1），
          照它判"夹到没有"会一直被骗 ✗。现在 joint2 已改成受控关节并由
          finger_mimic_relay 跟随 joint1，所以：
            · TF 量出来的间距 = 物理真值 ✓
            · (q1 + q2) 也应与它一致；不一致 ⇒ 中继/控制器没起作用 ✗（会告警）
        """
        for name, pos in zip(msg.name, msg.position):
            if name == "fr3_finger_joint1":
                self._finger_q = float(pos)
            elif name == "fr3_finger_joint2":
                self._finger_q2 = float(pos)

    def tcp_pose_base(self):
        """指尖平面 fr3_hand_tcp 在当前 base_footprint 系里的 (x, y, z)；取不到返回 None。

        ★ 这是"机械臂到底把指尖送到了哪"的唯一直接证据（TF = 机器人自身运动学，不是真值）：
          把它和视觉量到的物体顶面高度一比，就能立刻分辨
            · 指尖确实落在物体高度带里 → 问题在物体/模型尺寸
            · 指尖停在物体顶面之上   → 问题在机械臂/规划（下去不够深）✗
        """
        if self.tf_buffer is None:
            return None
        try:
            tr = self.tf_buffer.lookup_transform("base_footprint", "fr3_hand_tcp", Time())
            t = tr.transform.translation
            return (t.x, t.y, t.z)
        except Exception:
            return None

    def tcp_axes_base(self):
        """指尖坐标系的 z 轴（接近方向）与 y 轴（两指合拢方向）在 base 里的单位向量。

        为什么需要：现场观察到"夹爪角度有问题"（手指不是水平地夹住罐子）✗
        · 顶抓时 z 轴应竖直（与 base 的 z 夹角 ≈0° 或 180°）
        · 两指合拢方向 y 轴应水平（与水平面夹角 ≈0°）
        这两个角一旦偏，两指就会斜着夹（先碰罐口沿、再把它挤走）——
        位置对、角度错时，日志里的"位置核对"和"尺寸测距"都看不出来 ✗
        """
        if self.tf_buffer is None:
            return None
        try:
            tr = self.tf_buffer.lookup_transform("base_footprint", "fr3_hand_tcp", Time())
            q = tr.transform.rotation
            x, y, z, w = q.x, q.y, q.z, q.w
            z_ax = (2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y))
            y_ax = (2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w))
            return z_ax, y_ax
        except Exception:
            return None

    def finger_gap_mm(self):
        """两指【真实间距】(mm)：直接量 fr3_leftfinger 与 fr3_rightfinger 两个帧的距离。

        ★ 为什么不能只看 /joint_states：URDF 里 fr3_finger_joint2 是 `<mimic>`，
          而 **Fortress(gz-sim 6.18) 不支持 mimic**（libignition-gazebo6 里没有任何
          mimic 代码；mimic 是 SDF 1.10 / Garden 才有的）→ 右手手指在仿真里没有驱动 ✗
          这时 /joint_states 只报 joint1（左手）→ 看起来"合爪正常"，实际只有一根手指在动 ✗
          量两个手指帧的距离才能看到真实间隙 ✓（TF 是机器人自己的运动学，不是真值）
        """
        if self.tf_buffer is None:
            return None
        try:
            tr = self.tf_buffer.lookup_transform("fr3_leftfinger", "fr3_rightfinger", Time())
            t = tr.transform.translation
            return math.sqrt(t.x * t.x + t.y * t.y + t.z * t.z) * 1000.0
        except Exception:
            return None

    def fingers_mm(self):
        """两指内侧面间距（mm）= (q1 + q2) × 1000；只读到一指时退化为 2×q1；读不到返回 None。

        ★ 用两指之和才是**物理**间距（每根手指各自平移 q，两指内侧面分别在 ±q）✓
        """
        if self._finger_q is None:
            return None
        if self._finger_q2 is None:
            return self._finger_q * 2000.0
        return (self._finger_q + self._finger_q2) * 1000.0

    # ══════════════ 等待原语（绝不 spin ✗）══════════════
    def sleep(self, seconds):
        """等一会儿。调用方必须是多线程 executor，否则时钟/回调不会前进。"""
        time.sleep(max(0.0, seconds))

    def wait_future(self, fut, timeout):
        """轮询 future（不 spin）。"""
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            if fut.done():
                return True
            time.sleep(0.02)
        return fut.done()

    def wait_client(self, client, timeout=SERVICE_WAIT):
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            if client.service_is_ready():
                return True
            time.sleep(0.05)
        return False

    # ══════════════ ① 目标获取（只有视觉；真值一律不读 ✗）══════════════
    def fetch_targets(self):
        """返回 GraspTargetStamped 列表（按置信度降序）。拿不到就返回空列表。

        ★ 只调视觉服务 /detect_grasp_target。以前这里有一条"读不到就退回 gz 真值"的
          退路 —— 已按用户要求**整条删除**：真值 = 直接读仿真答案，裁判禁止（即使是
          调试也不许用，因为它会让"视觉到底行不行"这个结论失效 ✗）。
          视觉拿不到目标时，本阶段就老老实实失败并打日志。
        """
        if not self.wait_client(self.vision_client):
            self.log.error("视觉服务 {} 不可用（适配层没起来？）".format(VISION_SERVICE))
            return []
        req = DetectGraspTarget.Request()
        # ★ 2026-09-19：`--classes` 没给时，**不再发空列表** ✗
        #   发空列表 ⇒ 视觉先走队友默认 4 类词表（apple/coke can/bowl/banana），
        #   只有"那一帧里这 4 类一个都没认出来"才切闭集复核 ⇒ 桌上真正的物体
        #   （tomato_soup_can / potted_meat_can / cracker_box…）经常整帧拿不到 ✗
        #   实测（12:20 那一趟，用户没带 --classes）：扫视里拿到的是
        #   cracker_box×2 / tomato_soup_can / apple，其中 apple 还是**隔壁桌**的幻影；
        #   站位上重测又只剩 1.5 m 外的 phantom ⇒ 一路退化成"用陈旧点抓"⇒ 抓空 ✗✗
        #   ⇒ 默认就请求 objects.yaml 的**全部类别**（视觉侧直接走闭集复核，
        #     目标与障碍物都齐）✓ 想只抓某一类仍然用 --classes ✓
        req.class_ids = self.target_classes or self.all_classes
        t_v0 = time.time()
        fut = self.vision_client.call_async(req)
        if not self.wait_future(fut, VISION_CALL_TIMEOUT):
            self.log.error("视觉服务调用超时（>{:.0f}s）".format(VISION_CALL_TIMEOUT))
            return []
        res = fut.result()
        if res is None:
            self.log.error("视觉服务无响应")
            return []
        if not res.success or not res.targets:
            self.log.warn("视觉没有可用目标：{}".format(res.message))
            # ★ 2026-09-19：把"点了哪几类"打出来 —— 现场那次 8 连败（观察位 3 角 + 近看位 5 角
            #   全 "no class matched"）的真正原因是**视觉侧没切闭集复核**（默认 4 类词表里
            #   认出了 coke can/apple 就不切），而日志里根本看不出调用方点了什么 ⇒
            #   这条 + 视觉侧那条"第 1 帧认出…里面没有调用方要的…→ 抓取模式"要对着看 ✓
            if self.target_classes:
                self.log.warn("  本次请求的类别 = {}（空 = 不限类别；视觉侧先试 4 类默认词表，"
                              "认不到才会切闭集复核）".format(self.target_classes))
            else:
                self.log.warn("  本次请求**没点类别** ⇒ 视觉走 4 类默认词表"
                              "（apple/coke can/bowl/banana），桌上其它物体认不出来 ✗"
                              " ⇒ 用 --classes <类名,类名…> 指定要抓的类 ✓")
            return []
        tg = sorted(res.targets, key=lambda t: -t.confidence)
        self.log.info("视觉返回 {} 个目标（{}；耗时 {:.1f}s）".format(
            len(tg), res.message, time.time() - t_v0))
        return tg

    # ══════════════ ①b 支撑面桌沿法线（观察位与站位共用同一个朝向）══════════════
    def _face_ok(self, n, obj_map):
        """从桌沿法线 n 那一侧接近，观察位与站位是否都站得下人。

        观察位要求 clearance 0.32 m（开阔地）；站位只要求 0.16 m
        （贴桌沿停车是常态，0.26 m 的 EDGE_CLEARANCE 另外由 standoff_pose 保证）。
        """
        ox = TABLE_CENTER_XYZ[0] + n[0] * OBSERVATION_DIST
        oy = TABLE_CENTER_XYZ[1] + n[1] * OBSERVATION_DIST
        if not self._free(ox, oy, ROBOT_CLEAR_OBS):
            return False
        for g in (STANDOFF_DIST, STANDOFF_DIST + 0.1, STANDOFF_DIST + 0.2, STANDOFF_DIST + 0.3):
            if self._free(obj_map[0] + n[0] * g, obj_map[1] + n[1] * g, ROBOT_CLEAR_PARK):
                return True
        return False

    def approach_normal(self, obj_map, robot_map):
        """选一条"[物体] 朝 [机器人] 那一侧、而且真的站得下人"的桌沿外法线。

        ★ 为什么不能再用"机器人当前方位"当接近方向（原实现就是那么干的）：
          站位 yaw = 从站位指向物体的方位，而站位 = 物体 + g × (物体→机器人方向)。
          "物体→机器人"这个方向里含【AMCL 定位误差】和【观察位停位偏差】：
          实测机器人停在 (2.6, 2.93)（观察位是 (2.7, 3.2)），算出来
          yaw = −1.155 rad，而桌沿法线是 −1.571 rad → **机械臂斜 24° 伸到桌上** ✗
        ★ 也不能只看方位：机器人从起点 (-5.30,-0.50) 出发时"物体在西边"→ 选西侧桌沿，
          而西侧桌沿外是 dining_table_1 的桌面 ✗（实测观察位被算到 (1.55, 2.0) 桌子里）。
          所以顺序是：先按方位排序（少绕路），再拿地图逐条筛"站得下人" ✓
        """
        dx = robot_map[0] - obj_map[0]
        dy = robot_map[1] - obj_map[1]
        cands = sorted(TABLE_FACE_NORMALS, key=lambda n: -(n[0] * dx + n[1] * dy))
        for n in cands:
            if self._face_ok(n, obj_map):
                if n != cands[0]:
                    self.log.info("  桌沿法线：按方位首选 {} 站不下人 → 改用 {}".format(
                        tuple(cands[0]), tuple(n)))
                return n
        self.log.warn("四条桌沿都站不下人（地图没载入？）→ 退回按方位选 {}".format(tuple(cands[0])))
        return cands[0]

    def observation_pose_for(self, robot_map, dist=OBSERVATION_DIST):
        """观察位 = 桌心沿"机器人那一侧的法线"外推 dist，朝向 = 面对桌心。

        它与站位用的是**同一条法线** → 站在观察位上机械臂就已经对正桌子，
        到站位只是沿同一条直线靠近，不会中途"拧"过去。
        """
        n = self.approach_normal(TABLE_CENTER_XYZ, robot_map)
        px = TABLE_CENTER_XYZ[0] + n[0] * dist
        py = TABLE_CENTER_XYZ[1] + n[1] * dist
        yaw = math.atan2(TABLE_CENTER_XYZ[1] - py, TABLE_CENTER_XYZ[0] - px)
        return (px, py, yaw)

    @staticmethod
    def _normal_from_pose(pose):
        """从一个位姿反推它是从哪一条桌沿法线看过去的（取主轴分量，轴对齐矩形）。

        ★ 用途：把"这一趟从哪一侧接近"固定下来（见 run()），命令行手工给的观察位
          也能正确推出所属桌沿 —— 之后近看位前进/站位解算都用它，不会中途换边 ✓
        """
        dx = pose[0] - TABLE_CENTER_XYZ[0]
        dy = pose[1] - TABLE_CENTER_XYZ[1]
        if abs(dx) >= abs(dy):
            return (1.0 if dx >= 0 else -1.0, 0.0)
        return (0.0, 1.0 if dy >= 0 else -1.0)

    def _ensure_obs_normal(self):
        """拿到本趟的桌沿法线（没有就按"机器人现在在桌子哪一侧"补一个 ✓）。"""
        if self.obs_normal is None:
            rp = self._robot_map_pose()
            if rp is not None:
                self.obs_normal = self.approach_normal(TABLE_CENTER_XYZ[:2], rp[:2])
                self.log.info("  接近法线未在观察时定过 → 按当前所在侧补定 ({:+.2f}, {:+.2f})"
                              .format(self.obs_normal[0], self.obs_normal[1]))
            else:
                self.obs_normal = self._normal_from_pose(
                    (TABLE_CENTER_XYZ[0], TABLE_CENTER_XYZ[1] + OBSERVATION_DIST))
                self.log.warn("  拿不到 map←base_footprint → 接近法线兜底 {:+.2f},{:+.2f}"
                              .format(self.obs_normal[0], self.obs_normal[1]))
        return self.obs_normal

    # ══════════════ ② 选目标（硬过滤 + 档位 + 并列时排序）══════════════
    @staticmethod
    def _in_neighbor_table(p_map):
        """点是否落在【邻居餐桌自己的脚印】里（带 5 cm 收缩，吸收 AMCL/视觉误差）。

        邻居桌 = dinning_table_1 (1.5,2.0) / dinning_table_2 (2.7,1.5)（见 NEIGHBOR_TABLES）。
        落在里面 = "这是隔壁桌子上的物体"，几乎不可能是本桌目标 ✓
        """
        if not TABLE_IS_DINING:                   # 只在餐厅那三张并排桌子时启用 ✓
            return False
        for cx, cy in NEIGHBOR_TABLES:
            if (abs(p_map[0] - cx) <= NEIGHBOR_HALF[0] - 0.05
                    and abs(p_map[1] - cy) <= NEIGHBOR_HALF[1] - 0.05):
                return True
        return False

    @staticmethod
    def _on_support(p_map, margin=0.30):
        """目标是否落在支撑面（餐桌 dinning_table_3）足迹内（带容差，吸收 AMCL 误差）。

        ★ 必须有这道校验：站在观察位面朝餐桌时，相机水平视野 62°，会**同时看到左右
          两张桌子** ✗（实测：table_1 x0.9~2.1、table_2 y1.25~1.75，都在画面里，
          它们上面的物体比本桌的多）。曾经因此选中 table_2 上的东西 →
          站位被算到 table_2 脚印【里面】(2.849,1.460) → **车直接开上桌子** ✗
        ★ 光靠"足迹 + 容差"不够（容差要放大到 0.20 m 才吃得下 AMCL 误差，而 table_1
          东端 chips_can 在 (1.9, 2.0)，|dx| = 0.8 正好落进 0.6+0.20 的窗里 ✗）
          → 再显式排掉【邻居餐桌自己的脚印】里的点 ✓
        ★ 2026-09-18：本函数**只做判断，不再受 SUPPORT_GATE_ENFORCE 影响** ——
          那道门现在是"逐候选打分"（见 choose_target），不再"整轮硬拦/整轮放弃" ✗
        """
        if GraspPhase._in_neighbor_table(p_map):
            return False
        return (abs(p_map[0] - TABLE_CENTER_XYZ[0]) <= TABLE_HALF_X + margin
                and abs(p_map[1] - TABLE_CENTER_XYZ[1]) <= TABLE_HALF_Y + margin)

    @staticmethod
    def _support_rank(p_map):
        """候选"在本桌上"的可信度：0 = 在本桌脚印内（优先）；1 = 地点不明；2 = 在邻居桌脚印内。

        ★ 为什么用【打分】而不是"拦/不拦"（2026-09-18 定稿）：
          · 硬拦（SUPPORT_GATE_ENFORCE=True，e1acaf3 之前）会**误杀真目标**（map 侧位姿
            实测能在扫视/导航期间漂 0.5 m）⇒ 整轮抓取直接放弃 ✗
          · 完全不拦（当前）会让【邻桌幻影】劫持选目标 ⇒ 站位被解算到桌外/桌子另一边
            ⇒ 导航到餐桌另一侧、车头顶到桌子上 ✗（用户现场现象）
          · 打分兼顾两者：本桌候选永远优先；一个本桌候选都没有时，才退而用"地点不明"
            的候选（可能是漂了的真目标 ✓）；只有"明确落在邻桌脚印里"的候选排最后，
            且**永不因为校验失败放弃整轮** ✓
        """
        if GraspPhase._on_support(p_map):
            return 0
        if GraspPhase._in_neighbor_table(p_map):
            return 2
        return 1

    def choose_target(self, targets, exclude=(), check_reach=True, robot_pose=None,
                      on_support_only=True):
        """返回 (target, obstacles, reason)。exclude 里放已经试过且失败的类别。

        check_reach：**观察位必须传 False** ✗ —— 观察位离桌 1.15 m，桌上任何物体
        到臂基座都在 1.1~1.3 m，套 [0.10, 0.75] 会把所有候选全拒掉
        （实测：`跳过 sugar_box：距臂基座 1.289 m 不在 [0.10, 0.75]` → Phase 2 直接结束）
        可达性只在【站位上】才有意义 ✓
        """
        cands = []
        for i, t in enumerate(targets):
            cls = t.class_id
            if cls in exclude:
                continue
            # 能不能夹：优先看 objects.yaml 的 graspable，读不到才用兜底名单
            can_grasp = self.graspable.get(cls, cls not in NON_GRASPABLE)
            if not can_grasp:
                if cls not in getattr(self, "_logged_nongrasp", set()):
                    self.log.info("  跳过 {}：objects.yaml 标了 graspable=false（窄边 ≥ 开口）"
                                  .format(cls))
                    self._logged_nongrasp = getattr(self, "_logged_nongrasp", set()) | {cls}
                continue
            if t.confidence < self.min_confidence:
                self.log.info("  跳过 {}：置信度 {:.2f} < {:.2f}".format(
                    cls, t.confidence, self.min_confidence))
                continue
            # 支撑面打分（见 _support_rank）：本桌 0 / 地点不明 1 / 邻桌脚印内 2。
            # ★ 旧行为是"SUPPORT_GATE_ENFORCE 关掉就整条不判"⇒ 幻影劫持；现在是排序依据 ✓
            rank = 1
            if on_support_only and robot_pose is not None:
                m = self._to_map((t.point.x, t.point.y), robot_pose)
                rank = self._support_rank(m)
                if SUPPORT_GATE_ENFORCE and rank != 0:
                    self.log.info("  跳过 {}：不在支撑面（餐桌）上 map({:.3f},{:.3f}) ✗"
                                  "（SUPPORT_GATE_ENFORCE=True）".format(cls, m[0], m[1]))
                    continue
                if rank == 2:
                    self.log.warn("  ⚠ {} 报出的位置 map({:.3f},{:.3f}) 落在【邻居餐桌】脚印里"
                                  "（隔着本桌看到的那张桌子）→ 只作最后备选 ✗"
                                  .format(cls, m[0], m[1]))
                elif rank == 1:
                    self.log.info("  ? {} 的位置 map({:.3f},{:.3f}) 不在本桌脚印 ±{:.2f} m 内"
                                  "（map 侧位姿漂移？）→ 本桌候选优先，它排后面"
                                  .format(cls, m[0], m[1], 0.30))
            r = math.hypot(t.point.x - ARM_BASE_X, t.point.y)
            if check_reach and not (REACH_MIN <= r <= REACH_MAX):
                self.log.info("  跳过 {}：距臂基座 {:.3f} m 不在 [{:.2f}, {:.2f}]".format(
                    cls, r, REACH_MIN, REACH_MAX))
                continue
            tier = next((k for k, row in enumerate(TARGET_TIERS) if cls in row), len(TARGET_TIERS))
            # 并列时的排序键：离机器人近（x 小）→ 与邻居间隙大 → 置信度高
            gap = min([math.hypot(t.point.x - o.point.x, t.point.y - o.point.y)
                       for j, o in enumerate(targets) if j != i] or [9.9])
            # ★ 排序键第 0 位 = 支撑面档位（本桌优先），第 1 位才是类目档位（圆柱优先）：
            #   旧实现只有类目档位 ⇒ 邻桌上的"圆柱类幻影"（potted_meat_can 档 1）
            #   会压过本桌上真能夹的方盒（档 2）✗
            cands.append((rank, tier, t.point.x, -gap, -t.confidence, t, i))
        if not cands:
            return None, [], "没有可夹且可达的目标"
        cands.sort(key=lambda c: c[:5])

        rank, tier, _, neg_gap, _, best, bi = cands[0]
        obstacles = [t for j, t in enumerate(targets) if j != bi]
        reason = ("\"{}\" / 距臂基座 {:.3f} m / 与邻居最近 {:.3f} m / conf {:.2f} / "
                  "支撑面档 {}（0=本桌 1=不明 2=邻桌脚印内）").format(
            best.class_id, math.hypot(best.point.x - ARM_BASE_X, best.point.y),
            -neg_gap, best.confidence, rank)
        return best, obstacles, reason

    # ══════════════ ③ 站位解算（目标 map 位置 → 停车点）══════════════
    @staticmethod
    def _to_map(p_base, rp):
        """base_footprint 点 → map 点（rp = 当时的机器人 map 位姿）。"""
        rx, ry, yaw = rp
        c, s = math.cos(yaw), math.sin(yaw)
        return (rx + c * p_base[0] - s * p_base[1],
                ry + s * p_base[0] + c * p_base[1])

    @staticmethod
    def _to_base(p_map, rp):
        """map 点 → 当前 base_footprint 点。"""
        rx, ry, yaw = rp
        dx, dy = p_map[0] - rx, p_map[1] - ry
        c, s = math.cos(yaw), math.sin(yaw)
        return (c * dx + s * dy, -s * dx + c * dy)

    def _mk_target(self, cls, conf, xy_base, stamp=None):
        t = GraspTargetStamped()
        t.header.frame_id = "base_footprint"
        t.header.stamp = stamp if stamp is not None else self.node.get_clock().now().to_msg()
        t.class_id = cls
        t.confidence = float(conf)
        t.point.x, t.point.y, t.point.z = float(xy_base[0]), float(xy_base[1]), TABLE_TOP_Z
        return t

    # ══════════════ ★★ 桌腿基准：用激光反解"车在 map 里的真实偏航"（2026-09-19）══════════════
    def _on_scan(self, msg):
        """存最新一帧激光（只留 ranges 与角度参数，够反解桌腿就行 ✓）。"""
        self._scan = (time.time(), msg.angle_min, msg.angle_increment,
                      list(msg.ranges), msg.range_min, msg.range_max)

    def _scan_points_base(self, max_points=4000):
        """把最新一帧激光转成 base_footprint 下的 (x, y) 点列表；数据太旧/没有则回 None。

        ★ 帧：/scan 的 frame_id 是 base_scan（车体中心、z≈0.18 m）⇒ x/y 与
          base_footprint 只差一个平移，用 TF 查到就用，查不到就按"同轴"处理（误差 <1 cm ✓）
        """
        sc = getattr(self, "_scan", None)
        if not sc:
            return None
        ts, amin, ainc, ranges, rmin, rmax = sc
        if time.time() - ts > SCAN_MAX_AGE:
            return None
        ox = oy = 0.0
        try:
            tr = self.tf_buffer.lookup_transform("base_footprint", "base_scan", Time())
            ox, oy = tr.transform.translation.x, tr.transform.translation.y
        except Exception:                                  # noqa: BLE001
            pass
        pts = []
        for i, r in enumerate(ranges):
            if r is None or not math.isfinite(r) or r < rmin or r > rmax - 1e-3:
                continue
            a = amin + i * ainc
            pts.append((ox + r * math.cos(a), oy + r * math.sin(a)))
            if len(pts) >= max_points:
                break
        return pts or None

    def _table_yaw_probe(self, rp_believed, *, verbose=True):
        """用激光扫到的桌腿反解"车在 map 里的真实偏航" → (yaw_err, 说明)。

        yaw_err = TF(AMCL) 报的偏航 − 桌腿反解的偏航。
        为什么要有它：合拢轴 = object_yaw_map − robot_yaw_map(TF/AMCL)，
        AMCL 偏航一偏，两指就沿错误方向合拢；对 50×97 的罐子 φ>19.8° 时
        需要的开口超过 80 mm ⇒ 几何上夹不进去（现场"手停在物体顶上、合爪合空"）
        ⇒ 必须用【不依赖 AMCL 的基准】把这个偏差量出来并补偿掉 ✓
        返回 None 表示"这次量不出来"（腿没扫全/残差大/偏差离谱）⇒ 调用方保持原行为 ✓
        """
        legs = getattr(self, "_table_legs_map", None)
        pts = self._scan_points_base()
        if rp_believed is None:
            return None
        if not legs:
            if verbose:
                self.log.warn("  桌腿基准不可用（读不到餐桌位姿）⇒ 不修正朝向")
            return None
        if not pts:
            if verbose:
                self.log.warn("  桌腿基准不可用（拿不到 /scan 或数据太旧）⇒ 不修正朝向")
            return None
        got, idx, miss = legs_from_scan(pts, legs, rp_believed)
        if len(got) < 3:
            # 第二轮：用第一轮的解（哪怕只有 3 条腿）重预测再配一次 —— 半径匹配本来就
            # 不怕偏航误差，这一步只是把"位置差得较多"的情况也兜住 ✓
            if len(got) >= 1:
                f0 = fit_pose_from_landmarks(got, [legs[k] for k in idx])
                if f0 is not None:
                    got, idx, miss = legs_from_scan(pts, legs, f0[:3])
        if len(got) < 3:
            if verbose:
                self.log.warn("  桌腿基准：4 条腿只对上 {} 条（缺 {}）⇒ 不修正朝向 ✗"
                              .format(len(got), miss))
            return None
        ref = [legs[k] for k in idx]        # 命中腿的 map 真值（保序 ✓）
        fit = fit_pose_from_landmarks(got, ref)
        if fit is None:
            return None
        fx, fy, fyaw, rms = fit
        err = math.atan2(math.sin(rp_believed[2] - fyaw), math.cos(rp_believed[2] - fyaw))
        dxy = math.hypot(fx - rp_believed[0], fy - rp_believed[1])
        usable = (rms <= YAW_FIT_RMS_MAX and abs(err) <= YAW_ERR_USABLE and dxy <= 0.50)
        if verbose:
            self.log.info(
                "  ★ 桌腿基准: 命中 {}/4 条腿，反解车在 map 的位姿 ({:+.3f},{:+.3f},yaw={:+.3f} rad)"
                " 残差 {:.1f} mm；TF(AMCL) 报 yaw={:+.3f} rad ⇒ **偏航偏差 e={:+.1f}°**"
                "（位置差 {:.0f} mm）{}".format(
                    len(got), fx, fy, fyaw, rms * 1000, rp_believed[2], math.degrees(err), dxy * 1000,
                    "   ✓ 可用" if usable else
                    "   ✗ 不可信（残差/偏差/位置差超限）⇒ 不修正朝向"))
        return (err, rms, dxy) if usable else None

    def _inject_object_yaw(self, err):
        """把偏航偏差 e 写进 grasp_node 的 object_yaw_map（= 物体在 map 里的偏航按 e 修正）。

        推导：C++ 里 box_yaw = object_yaw_map − robot_yaw_map。
        真值应是 box_yaw = 0 − 真实偏航 = 0 − (robot_yaw_map − e) = e − robot_yaw_map
        ⇒ 只要令 object_yaw_map := e 就自动抵消 AMCL 的偏航误差 ✓（C++ 侧已支持运行时重读）
        """
        if not hasattr(self, "_param_client"):
            try:
                from rcl_interfaces.srv import SetParameters
                self._param_client = self.node.create_client(
                    SetParameters, "{}/set_parameters".format(GRASP_NODE_NAME))
            except Exception as e:                          # noqa: BLE001
                self.log.warn("  注入 object_yaw_map 失败（建不了参数客户端）: {}: {}"
                              .format(type(e).__name__, e))
                self._param_client = None
        if self._param_client is None:
            return False
        if not self._param_client.wait_for_service(timeout_sec=2.0):
            self.log.warn("  {}/set_parameters 不可用 ⇒ 偏航偏差没能注入（朝向仍按 AMCL 算 ✗）"
                          .format(GRASP_NODE_NAME))
            return False
        from rcl_interfaces.msg import Parameter as RosParameter, ParameterValue, ParameterType
        req = SetParameters.Request()
        p = RosParameter()
        p.name = "object_yaw_map"
        p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(err))
        req.parameters = [p]
        fut = self._param_client.call_async(req)
        self.wait_future(fut, 3.0)
        return True

    def _apply_table_yaw_fix(self, rp=None, verbose=True):
        """量一次桌腿基准偏差并注入 grasp_node；返回注入的 e（没量到则 None）。

        ★ 只要量得出来就**一定写参数**（包括写 0）：
          参数是节点级的、会跨"驱动进程"残留 —— 上一趟注入过 e、这一趟重跑时若因为
          "偏差在容差内"而跳过写入，就会拿旧 e 去算朝向 ✗（进程内的 _last_yaw_inject
          在新进程里是 None，根本发现不了残留）⇒ 一律显式写入 ✓
        """
        rp = rp if rp is not None else self._robot_map_pose()
        probe = self._table_yaw_probe(rp, verbose=verbose)
        if probe is None:
            if verbose:
                self.log.warn("  ⚠ 桌腿基准这次量不出来 ⇒ object_yaw_map 保持现值"
                              "（若本仿真会话里之前注入过 e，它仍在生效 ✗ 注意核对日志）")
            return None
        err = probe[0]
        if not self._inject_object_yaw(err):
            return None
        if abs(err) < YAW_ERR_APPLY:
            if verbose:
                self.log.info("  （偏差 {:.1f}° 在容差内 → object_yaw_map 显式写回 0 ✓）"
                              .format(math.degrees(err)))
            self._last_yaw_inject = None
            return None
        self._last_yaw_inject = err
        if verbose:
            self.log.warn("  ★ 已把偏航偏差 e={:+.1f}° 注入 grasp_node 的 object_yaw_map "
                          "⇒ box_yaw = (0+e) − robot_yaw_map，AMCL 的偏航误差被抵消 ✓"
                          "（grasp_node 日志会打『object_yaw_map 运行时被改写』= 生效 ✓；"
                          "没打 ⇒ 那个包没重建 ✗）".format(math.degrees(err)))
        return err

    def _robot_odom_pose(self):
        """odom←base_footprint 的 (x, y, yaw)。

        ★ 最终目标换算必须走 odom，不能走 map ✗：
          map→base_footprint 那一段含 AMCL 误差（实测 0.16 m），而视觉/真值给的是
          **相对量**。用 map 换算等于把 AMCL 误差直接搬进抓取点：
          实测日志里"按 map 算 0.53 m、真值实测 0.98 m"差了 0.45 m，其中 0.16 m 就是它 ✗
          odom 是连续的机器人内部量，短时间尺度上比 AMCL 准得多 ✓
        """
        return self._lookup_pose("odom")

    def _wait_map_pose_stable(self, timeout=30.0):
        """等 map←base_footprint 稳定后再用：最多等 timeout s，每 0.5 s 试一次，
        **连续两次成功且两次位置差 < 0.02 m** 才算稳定可用；超时返回 None。

        ★ 为什么要有它（实测）：启动后约 5 s 就查 TF 会撞上 AMCL 的竞态
          （initialpose 刚发、AMCL 还在收敛）⇒ map←base_footprint 取不到、或取到还在跳的
          值 ⇒ 原来"拿不到就直接失败"会让【整轮抓取】白跑 ✗
          稳定之前不信任任何 map 换算（观察位/站位都是从它算出来的）✓
        ★ 代价：成功路径多花一个 0.5 s 采样间隔（必须隔开才有"连续两次"可比），
          稳定后立刻返回，不多等 ✓
        """
        deadline = time.time() + timeout
        prev = None
        while True:
            rp = self._robot_map_pose()        # 第一次一定查（与原逻辑一致，不受 rclpy 状态影响）
            if rp is not None:
                if prev is not None and math.hypot(rp[0] - prev[0], rp[1] - prev[1]) < 0.05:
                    return rp
                prev = rp
            if not rclpy.ok() or time.time() >= deadline:
                break
            self.sleep(0.5)
        # ★ 2026-09-16 修正：原来这里 return None ⇒ 调用方直接判定整轮失败 ✗
        #   实测 AMCL 启动阶段会连续抖动 >20 mm ⇒ 把任务卡死（"拿不到 map←base_footprint，
        #   算不出观察位"）。**拿到过位姿就该用最新的那个**，抖动只是精度差一点，
        #   远好于整轮放弃 ✓；只有一次都没取到才返回 None（= 原来的失败路径 ✓）
        if prev is not None:
            self.log.warn("  map←base_footprint 在 {:.0f} s 内没稳定（AMCL 仍在收敛？）"
                          "→ 用最新位姿 ({:+.3f},{:+.3f}) 继续 ✓".format(timeout, prev[0], prev[1]))
            return prev
        self.log.error("  map←base_footprint 一次都没取到（AMCL 没起来？）".format())
        return None

    def _lookup_pose(self, parent_frame):
        if self.tf_buffer is None:
            return None
        for _ in range(TF_RETRY):
            try:
                tr = self.tf_buffer.lookup_transform(parent_frame, "base_footprint", Time())
                t, q = tr.transform.translation, tr.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
                return (t.x, t.y, yaw)
            except Exception:
                self.sleep(0.2)
        return None

    def _robot_map_pose(self):
        if self.tf_buffer is None:
            return None
        for _ in range(TF_RETRY):
            try:
                tr = self.tf_buffer.lookup_transform("map", "base_footprint", Time())
                t, q = tr.transform.translation, tr.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
                return (t.x, t.y, yaw)
            except Exception:
                self.sleep(0.2)
        return None

    def _table_exit_distance(self, obj_map, ax, ay):
        """从物体沿接近方向 (ax, ay) 走到【桌沿外】需要多少米（物体已在桌外则 0）。

        用数值步进而不是解析求交：桌子脚印是轴对齐矩形，方向随便都成立，够用且不会写错。
        """
        step = 0.01
        t = 0.0
        while t <= 1.2:
            x, y = obj_map[0] + ax * t, obj_map[1] + ay * t
            if (abs(x - TABLE_CENTER_XYZ[0]) > TABLE_HALF_X
                    or abs(y - TABLE_CENTER_XYZ[1]) > TABLE_HALF_Y):
                return t
            t += step
        return 1.2

    def creep_goal_for(self, target_map, approach=(0.0, 1.0)):
        """按物体"离桌沿多远"决定站位距离 g（默认 0.33 m）。

        为什么需要自适应：相机装在 z=0.80 m（桌面顶 0.795 m）、且在车心前方 0.076 m。
        物体贴桌沿时（纵深 0.07，旧工作区验证过的情形）0.33 m 站位没问题 ✓；
        但物体放在桌里侧 0.25 m 时，0.33 m 站位会让【相机伸进桌面里 7 mm】✗
        → 相机会撞桌板、creep 永远收敛不了。
        做法：让"相机位置"（车心 + 0.08 m）仍在桌沿之外，所以
            g ≥ 沿接近方向走到桌沿的距离 + 0.08 + 余量 0.05
        纵深 0.07 → g = max(0.33, 0.20) = 0.33 ✓ 旧几何一字不变
        纵深 0.25（sugar_box）→ g = 0.38 ✓
        ★ 接近方向一律传【桌沿法线】（见 approach_normal）：法线是轴对齐的，
          所以"走到桌沿的距离"就是物体到那条边的垂距，数值步进也只是保险。
          旧实现用"机器人当前方位"当方向，方向里的定位误差会让这个距离也跟着错 ✗
        """
        ax, ay = approach
        n = math.hypot(ax, ay)
        if n < 1e-6:
            ax, ay = 0.0, 1.0
        else:
            ax, ay = ax / n, ay / n
        t_exit = self._table_exit_distance(target_map, ax, ay)
        g = max(STANDOFF_DIST, t_exit + CAMERA_FWD + STANDOFF_EDGE_MARGIN)
        if g > STANDOFF_DIST + 1e-3:
            self.log.info("  物体离桌沿 {:.3f} m → 站位从 {:.3f} 拉到 {:.3f} m"
                          "（免得相机撞桌板；相机到桌沿余量 {:.3f} m）"
                          .format(t_exit, STANDOFF_DIST, g, g - t_exit - CAMERA_FWD))
        return g

    def standoff_pose(self, target, standoff=None, robot_map=None, normal=None):
        """由目标的 base_footprint 位置算出 map 里的停车点与朝向。

        ★ 接近方向 = 桌沿法线，**由调用方在观察位定好后传进来**（当前实现见 run()：
           normal = self.obs_normal）—— 绝不由"目标位置 → 机器人"重算 ✗：
          那样一旦视觉把邻桌的物体报成本桌目标，法线就会翻到桌子的另一侧，
          站位跟着跑到桌子另一边（用户现场："导航到餐桌的另一边"）✗
        ★ 站位 = 物体 + g × 法线，yaw = 法线反方向 → 车头垂直桌沿、物体在正前方 ✓
        ★ 外推下限 EDGE_CLEARANCE = 0.34 m（≥ Nav2 robot_radius 0.28）——
          否则目标点落在代价地图的 inscribed 区，DWB 判非法轨迹，车到不了还顶桌子 ✗
        """
        rp = robot_map if robot_map is not None else self._robot_map_pose()
        if rp is None:
            return None
        rx, ry, ryaw = rp
        cy, sy = math.cos(ryaw), math.sin(ryaw)
        bx, by = target.point.x, target.point.y
        ox, oy = rx + cy * bx - sy * by, ry + sy * bx + cy * by       # 目标在 map 里
        ax, ay = normal if normal is not None else self.approach_normal((ox, oy), (rx, ry))
        # 站位必须落在桌子脚印之外（否则目标点落在致命代价上，规划器会拒 ✗）。
        # 用"沿接近方向小步外推"的数值办法，方向随便都成立；
        # 容差取 1e-6：正好等于 EDGE_CLEARANCE 边界时算【外面】（那本来就是留的余量）✓
        hx = TABLE_HALF_X + EDGE_CLEARANCE
        hy = TABLE_HALF_Y + EDGE_CLEARANCE

        def inside_table(x, y):
            return (abs(x - TABLE_CENTER_XYZ[0]) < hx - 1e-6
                    and abs(y - TABLE_CENTER_XYZ[1]) < hy - 1e-6)

        g0 = standoff if standoff else STANDOFF_DIST
        # ★ 2026-09-18：下限必须先和"目标离桌沿多远"对齐 —— 只从 g0 起算会留洞 ✗
        #   反例（离线自检 check_dining_view_geometry.py 实测到的）：从东侧接近一个
        #   在桌里侧 (2.30,2.00) 的目标时，creep_goal_for 给出 g0 = 1.14 m，
        #   而外推循环的上限写死 1.0 m ⇒ **循环一次都不执行**，站位直接落在
        #   桌沿外 0.14 m 处（inscribed 区）→ 又是"到不了 + 顶桌子"✗
        #   现在：下限 = 沿法线走到桌沿的距离 + EDGE_CLEARANCE（与桌子几何严格对齐 ✓），
        #   再在它之上按"空地/障碍"继续外推（上限给足 +0.6 m）✓
        t_edge = self._table_exit_distance((ox, oy), ax, ay)
        t_min = t_edge + EDGE_CLEARANCE
        t_off = max(g0, t_min)
        if t_off > g0 + 1e-9:
            self.log.info("  目标离桌沿（沿法线）{:.3f} m → 站位距离至少 {:.3f} m"
                          "（= 桌沿 + {:.2f} m 余量）".format(t_edge, t_min, EDGE_CLEARANCE))
        t_max = t_min + 0.6
        while t_off < t_max:
            x, y = ox + ax * t_off, oy + ay * t_off
            if not inside_table(x, y) and self._free(x, y, ROBOT_CLEAR_PARK):
                break
            t_off += 0.01
        if t_off > max(g0, t_min) + 1e-9:
            self.log.info("  站位被桌子/障碍挡住 → 沿桌沿法线外推到 {:.3f} m".format(t_off))
        if t_off >= t_max - 1e-9:
            # ★ 别静默用一个"其实站不下"的点（旧实现就是闷头用最后一个值 ✗）
            self.log.warn("  ⚠ 沿法线外推到 {:.2f} m 仍不满足（周围 {:.2f} m 内有障碍）→ "
                          "仍用它，但导航可能失败/顶障碍，日志里留意导航重试 ✗"
                          .format(t_max, ROBOT_CLEAR_PARK))
        px, py = ox + ax * t_off, oy + ay * t_off
        yaw = math.atan2(oy - py, ox - px)                             # 面朝物体（= 沿法线指向桌内）
        # ★ 自检：站位必须还在【桌沿法线那一侧】（不能跑到桌子另一边去）
        if (px - ox) * ax + (py - oy) * ay <= 0.0:
            self.log.error("  ✗ 站位解算方向异常：({:.2f},{:.2f}) 相对目标 ({:.2f},{:.2f}) "
                           "不在法线 ({:+.2f},{:+.2f}) 一侧".format(px, py, ox, oy, ax, ay))
        return (px, py, yaw, ox, oy)

    # ══════════════ ④ 导航 / 对准 / creep / 抓取服务 ══════════════
    @staticmethod
    def _normalize_angle(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def align_and_approach(self, aim_map, stand_dist=1.15, yaw_tol=0.03,
                           dist_tol=0.15, timeout=25.0):
        """原地对准 map 里的一个点 + 前后微调距离（观察位专用的小闭环）。

        为什么必须有：Nav2 的 xy_goal_tolerance 放宽到 0.4 m（抓取站位靠 creep 兜底），
        观察位没有闭环 → **实测停位偏了 313 mm**，桌子就偏出画面 ✗
        （实测日志：目标观察位 (2.70, 3.20)，实际停在 (2.95, 3.01)）
        这里用 TF 读机器人 map 位姿做"对准 + 靠近"，不查视觉 → 快（几秒）且稳 ✓
        stand_dist 是"机器人到 aim 点"的距离目标；aim 用餐桌中心时 1.15 m 很安全
        （桌面半深 0.25 + 车半径 0.35 = 0.60 m 才会碰到桌沿）。
        """
        moved = False
        deadline = time.time() + timeout
        try:
            while rclpy.ok() and time.time() < deadline:
                rp = self._robot_map_pose()
                if rp is None:
                    self.log.warn("拿不到 map←base_footprint，跳过对准")
                    return False
                rx, ry, ryaw = rp
                dx, dy = aim_map[0] - rx, aim_map[1] - ry
                d = math.hypot(dx, dy)
                err = self._normalize_angle(math.atan2(dy, dx) - ryaw)
                self.log.info("  对准观察点: 距离 {:.3f} m（目标 {:.2f}）偏航 {:+.3f} rad"
                              .format(d, stand_dist, err))
                if abs(err) < yaw_tol and abs(d - stand_dist) < dist_tol:
                    self.log.info("  对准完成 ✓")
                    return True
                if abs(err) > 0.15:                      # 先转正，再进出
                    v, w = 0.0, clamp(1.5 * err, -CREEP_W_MAX, CREEP_W_MAX)
                else:
                    v = clamp(1.2 * (d - stand_dist), -CREEP_V_MAX, CREEP_V_MAX)
                    w = clamp(1.5 * err, -CREEP_W_MAX, CREEP_W_MAX)
                moved = True
                self._publish_cmd_vel(v, w)
                self.sleep(CREEP_DT)
            self.log.warn("对准观察点超时（{:.0f}s）→ 就用当前位姿继续".format(timeout))
            return False
        finally:
            for _ in range(3):
                self._publish_cmd_vel(0.0, 0.0)
                self.sleep(0.2)
            if moved:
                self.sleep(0.4)

    # ══════════ ③b 近看位：只沿【同一条桌沿法线】向前走，不交给 Nav2 ══════════
    @staticmethod
    def table_edge_dist(normal):
        """桌心沿法线方向到【桌沿】的距离（= 轴对齐长方形在该方向上的支撑距离）。

        餐桌脚印 1.2(map x) × 0.5(map y) ⇒ 法线为 ±x 时 0.6 m、±y 时 0.25 m。
        ★ 近看位的距离必须按它算，不能按"离桌心的固定距离"算（见 CLOSE_LOOK_EDGE ✗）
        """
        ax, ay = abs(normal[0]), abs(normal[1])
        n = math.hypot(ax, ay)
        if n < 1e-6:
            return TABLE_HALF_Y
        ax, ay = ax / n, ay / n
        return ax * TABLE_HALF_X + ay * TABLE_HALF_Y

    def close_look_pose(self, normal):
        """近看位（map） = 桌心 + 法线 ×（桌沿距离 + CLOSE_LOOK_EDGE）；朝向 = 面对桌心。

        四个方向到桌沿的距离都是 CLOSE_LOOK_EDGE ✓（旧实现按桌心算 ⇒ 东西侧只剩 0.08 m ✗）
        返回 (x, y, yaw, dist_to_center)。
        """
        dist = self.table_edge_dist(normal) + CLOSE_LOOK_EDGE
        cx = TABLE_CENTER_XYZ[0] + normal[0] * dist
        cy = TABLE_CENTER_XYZ[1] + normal[1] * dist
        yaw = math.atan2(TABLE_CENTER_XYZ[1] - cy, TABLE_CENTER_XYZ[0] - cx)
        return (cx, cy, yaw, dist)

    def _goto_close_look(self, normal, timeout=15.0):
        """从观察位沿【同一条法线】向前开到近看位（cmd_vel 闭环；**不调 Nav2** ✗）。

        ★ 为什么不用 Nav2（这是"导航到餐桌另一边并撞桌子"的直接修法）：
          ① 旧实现的近看位 ="桌心 + 0.68 m"，在东西侧只离桌沿 0.08 m ⇒ 目标点落在桌子/
             inscribed 区里，DWB 到不了还一直往桌前顶 ✗；
          ② 交给 Nav2 就要重新规划一条路径 —— 它会自己选"绕到哪一侧"，
             于是出现"第一次观察之后就开到餐桌另一边"✗；
          ③ 这里只要"往前走一段"，直线上没有任何障碍（起点是法线上的观察位 ✓）：
             自己用底盘速度闭环，方向/终点都可控，也顺便把朝向调正 ✓
        安全护栏：
          · 机器人偏离法线 > 0.35 m 时**拒绝前进**（说明停位偏得太多，直着走会蹭桌子边）✗
          · 目标点就在法线上 ⇒ 闭环只会"前后 + 原地转向"，不会横向绕行 ✓
          · 到桌心距离不得小于 桌沿距离 + CLOSE_LOOK_EDGE（硬下限，不会开进桌子）✓
        """
        rp = self._robot_map_pose()
        if rp is None:
            self.log.warn("  拿不到 map←base_footprint → 跳过近看位（就用观察位继续）")
            return False
        _, _, _, dist_center = self.close_look_pose(normal)
        edge = self.table_edge_dist(normal)
        # ① 横向偏离检查：机器人在法线上的投影点 vs 实际位置
        rx, ry = rp[0], rp[1]
        along = (rx - TABLE_CENTER_XYZ[0]) * normal[0] + (ry - TABLE_CENTER_XYZ[1]) * normal[1]
        lateral = (rx - TABLE_CENTER_XYZ[0]) * (-normal[1]) + (ry - TABLE_CENTER_XYZ[1]) * normal[0]
        if along <= edge:                      # 已经在桌沿内/桌沿上（不该发生）
            self.log.warn("  ⚠ 现在到桌心的距离 {:.3f} m 小于桌沿距离 {:.3f} m → 不前进（只扫视）"
                          .format(along, edge))
            return False
        if along < OBSERVATION_DIST - 0.25:
            # ★ 说明：近看前进只对"观察位那一趟"有意义。换目标重试时机器人已经在【站位】
            #   （比观察位更近），这时再"前进到近看位"等于往回退，还容易蹭桌角 ✗
            #   （2026-09-18 实跑日志里那条"偏离桌沿法线 0.428 m"的告警就是这个场景）
            self.log.info("  现在离桌心 {:.2f} m（比观察位 {:.2f} m 更近，已在站位一侧）"
                          "→ 不做近看前进，就地扫视即可 ✓".format(along, OBSERVATION_DIST))
            return False
        if abs(lateral) > 0.35:
            self.log.warn("  ⚠ 偏离桌沿法线 {:.3f} m > 0.35 m → 不直着开（怕蹭桌角），"
                          "就在原地扫视 ✓".format(lateral))
            return False
        self.log.info("  前进到近看位（沿同一条法线；到桌心 {:.3f} m→{:.3f} m，"
                      "到桌沿恒为 {:.3f} m；横向偏离 {:+.3f} m）".format(
                          along, dist_center, CLOSE_LOOK_EDGE, lateral))
        return self.align_and_approach((TABLE_CENTER_XYZ[0], TABLE_CENTER_XYZ[1]),
                                       stand_dist=dist_center, dist_tol=0.08,
                                       timeout=timeout)

    def navigate(self, x, y, yaw):
        for attempt in range(1, MAX_NAV_RETRIES + 1):
            self.log.info("[{}/{}] 导航到 ({:.2f}, {:.2f}, yaw={:.2f})".format(
                attempt, MAX_NAV_RETRIES, x, y, yaw))
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = self.node.get_clock().now().to_msg()
            goal.pose.pose.position.x = float(x)
            goal.pose.pose.position.y = float(y)
            _, _, qz, qw = yaw_to_quat(yaw)
            goal.pose.pose.orientation.z = qz
            goal.pose.pose.orientation.w = qw
            fut = self.nav_client.send_goal_async(goal)
            if not self.wait_future(fut, GOAL_ACCEPT_TIMEOUT):
                self.log.warn("  目标接受超时，重试")
                continue
            gh = fut.result()
            if gh is None or not gh.accepted:
                self.log.warn("  目标被拒（{}/{}），2s 后重试".format(attempt, MAX_NAV_RETRIES))
                self.sleep(2.0)
                continue
            rf = gh.get_result_async()
            while rclpy.ok() and not rf.done():
                time.sleep(0.1)
            if rf.done() and rf.result().status == 4:
                self.log.info("  ✓ 到达（Nav2 status=4），底盘停止")
                return True
            self.log.warn("  ✗ 导航结束，重试")
            self.sleep(2.0)
        self.log.error("多次导航失败，放弃")
        return False

    def measure(self, class_id, targets_hint=None, refresh_after=600.0):
        """量出某类别物体在 base_footprint 下的 (x, y)；读不到返回 None。

        ★ 快路径：物体是静止的，第一次量到后把它记成 **odom 坐标**，
          之后每轮只用里程计换算到当前底盘系（10 Hz 级，不调视觉）✓
          为什么必须这样：适配层一次调用要 12~15 s（默认+抓取模式+窄词表复核），
          creep 若每轮都调视觉，18 轮 × 13 s = 234 s ✗（预算只有 45 s）
          而 creep 只走 0.2~0.6 m，里程计在这个尺度上漂移几 mm ✓
          全程不碰 gz 真值 ✓（裁判禁止的正是真值）
        ★ refresh_after 原来是 30 s —— 太短了 ✗：现在一次视觉调用就要 25~30 s
          （多帧投票 + 窄词表复核），所以 creep 一开始跟踪基准就"过期" → 转去重新
          调视觉 → 那一帧又没认出这个类别 → creep 直接放弃微调 →
          物体停在 base 0.7 m 外，抓取侧回 BAD_TARGET（距臂基座 0.93 > 0.75）✗
          （实测 2026-09-14 连续 4 轮都卡在这里）。物体在抓取期间不会动，
          600 s 的里程计外推在 0.5 m 尺度上完全够用 ✓
        """
        tr = self._track.get(class_id)
        if tr is not None and (time.time() - tr[0]) <= refresh_after:
            rp_map = self._robot_map_pose()      # ★ 必须用 map 位姿（见下）
            # ★ AMCL 跳变护栏（2026-09-19）：跟踪基准是在【旧 map 位姿】下换算来的，
            #   若 AMCL 之后跳了，用它外推会把物体整体搬走 ✗ ⇒ 跳变就强制重测 ✓
            rp0 = getattr(self, "_track_rp", None)
            if (rp_map is not None and rp0 is not None
                    and math.hypot(rp_map[0] - rp0[0], rp_map[1] - rp0[1]) > AMCL_JUMP_TOL):
                self.log.warn("  ⚠ AMCL 位姿在跟踪期间跳变 {:.0f} mm（({:+.2f},{:+.2f})→({:+.2f},{:+.2f})）"
                              "⇒ 不再用里程计外推，重新调视觉测一次 ✓".format(
                                  math.hypot(rp_map[0] - rp0[0], rp_map[1] - rp0[1]) * 1000,
                                  rp0[0], rp0[1], rp_map[0], rp_map[1]))
                tr = None
            if tr is not None and rp_map is not None:
                xy = self._to_base(tr[1], rp_map)
                self.log.info("  （用里程计跟踪 {}：odom{} → base({:+.3f},{:+.3f})）"
                              .format(class_id, tuple(round(v, 3) for v in tr[1]), xy[0], xy[1]))
                return xy
        if targets_hint is not None:
            for t in targets_hint:
                if t.class_id == class_id:
                    return (t.point.x, t.point.y)
        tg = self.fetch_targets()
        for t in tg:
            if t.class_id == class_id:
                self._remember(class_id, t)
                return (t.point.x, t.point.y)
        # ★ 开集标签是**逐帧抖的**：同一个糖盒这一帧叫 mustard_bottle、下一帧叫
        #   tomato_soup_can（实测 0.35~0.46 的分数、标签一轮一变）→ 按类别名跟踪
        #   会"读不到目标"直接放弃微调 ✗（实测就卡在这里）。
        #   微调只需要【位置】，所以退一步：找"离上次跟踪位置最近的那个检测"，
        #   只要够近（0.25 m）就认它是同一个物体 ✓ 标签原样打日志，不偷偷改类别。
        tr = self._track.get(class_id)
        if tr is not None and tg:
            rp_map = self._robot_map_pose()      # ★ 修正：原来喂 odom 位姿 ⇒ map 位置整体偏移
            if rp_map is not None:
                best, best_d = None, LABEL_FLICKER_RADIUS
                for t in tg:
                    q = self._to_map((t.point.x, t.point.y), rp_map)
                    d = math.hypot(q[0] - tr[1][0], q[1] - tr[1][1])
                    if d < best_d:
                        best, best_d = t, d
                if best is not None:
                    self.log.warn("  这次没认出 \"{}\" → 用 {} 的位置代替（相距 {:.3f} m ≤ {:.2f} m，"
                                  "判定为**同一物体**标签抖动，位置才是微调要的量）"
                                  .format(class_id, best.class_id, best_d, LABEL_FLICKER_RADIUS))
                    self._remember(class_id, best)
                    return (best.point.x, best.point.y)
                self.log.warn("  这次没认出 \"{}\"，而且别的检测都在 {:.2f} m 以外 "
                              "⇒ 那是**另一个物体**，不许替代 ✗（12:20 现场：用 0.12 m 外的"
                              "可乐罐代替番茄罐 ⇒ 微调去对正可乐罐、抓取点又用陈旧值 ⇒ 抓空）"
                              .format(class_id, LABEL_FLICKER_RADIUS))
        return None

    def _remember(self, class_id, tgt):
        """把某类别的当前位置记成 **map 坐标**（creep 的跟踪基准）+ 记录当时的 map 位姿。"""
        rp_map = self._robot_map_pose()          # ★ 轨迹存 map 坐标，就得用 map 位姿
        if rp_map is None:
            return
        self._track[class_id] = (time.time(), self._to_map((tgt.point.x, tgt.point.y), rp_map))
        # ★ 2026-09-19：记下"建立跟踪时"的 map 位姿 —— AMCL 若在这之后跳变（实测能跳 ~0.16 m ✗），
        #   里程计外推出来的 base 坐标就会整体错位（"后面几次越来越不准"的元凶之一 ✗）
        #   ⇒ 跳变超阈值时不猜，强制重新调视觉测一次 ✓
        self._track_rp = rp_map

    def _publish_cmd_vel(self, v, w):
        m = Twist()
        m.linear.x = float(v)
        m.angular.z = float(w)
        self.cmd_pub.publish(m)

    def creep(self, class_id, goal_dist=None):
        """相对闭环微调：把物体开到 base_footprint(0.33, 0) 且方位→0。

        用【相对量】闭环，不查 map → 定位误差与 Nav2 容差都不进这个回路。
        ★ 容差见文件头：方位 0.013 rad ≈ ±4.3 mm，比原来的 0.05 rad 紧得多，
          并且要连续 CREEP_OK_ROUNDS 轮达标才收工。
        """
        goal_d = goal_dist if goal_dist else math.hypot(*CREEP_GOAL_XY)
        self.log.info("相对微调: {} → base_footprint({:.2f}, {:.2f}) ±{:.3f} m / ±{:.3f} rad"
                      .format(class_id, goal_d, CREEP_GOAL_XY[1],
                              CREEP_TOL, CREEP_BEARING_TOL))
        moved, ok_rounds, n = False, 0, 0
        best_err, best_n = None, 0
        deadline = time.time() + CREEP_TIMEOUT
        try:
            while rclpy.ok() and time.time() < deadline:
                xy = self.measure(class_id)
                if xy is None:
                    self.log.warn("微调中读不到目标 → 放弃微调，用当前停位抓取")
                    return False
                d = math.hypot(*xy)
                bearing = math.atan2(xy[1], xy[0])
                dist_err = d - goal_d
                n += 1
                self.log.info("  [{:2d}] 物体 base({:+.3f}, {:+.3f}) 距离 {:.3f} 方位 {:+.3f}"
                              .format(n, xy[0], xy[1], d, bearing))
                if abs(dist_err) < CREEP_TOL and abs(bearing) < CREEP_BEARING_TOL:
                    ok_rounds += 1
                    if ok_rounds >= CREEP_OK_ROUNDS:
                        self.log.info("微调完成（连续 {} 轮达标，共 {} 轮）".format(ok_rounds, n))
                        return True
                    self.sleep(CREEP_DT)
                    continue
                ok_rounds = 0
                # ★ 卡死早退：实测有 152 轮的案例（车被桌子/障碍顶住，命令发出去不动 ✗）
                #   连续 40 轮（≈4 s）距离误差没有改善就收工，别把 45 s 预算烧光
                cur_err = abs(dist_err) + abs(bearing) * 0.33
                if best_err is None or cur_err < best_err - 0.002:
                    best_err, best_n = cur_err, n
                elif n - best_n >= 40:
                    self.log.warn("连续 40 轮没有改善（误差 {:.3f}）→ 判定被顶住，停住底盘用当前停位抓取"
                                  .format(cur_err))
                    return False
                if abs(bearing) > CREEP_SPIN_BEARING:
                    v, w = 0.0, clamp(CREEP_KW * bearing, -CREEP_W_MAX, CREEP_W_MAX)
                else:
                    v = clamp(CREEP_KV * dist_err, -CREEP_V_MAX, CREEP_V_MAX)
                    w = clamp(CREEP_KW * bearing, -CREEP_W_MAX, CREEP_W_MAX)
                moved = True
                self._publish_cmd_vel(v, w)
                self.sleep(CREEP_DT)
            self.log.warn("微调超时（{:.0f}s）→ 停住底盘，用当前停位抓取".format(CREEP_TIMEOUT))
            return False
        finally:
            for _ in range(3):                 # 无论成败都要把速度归零
                self._publish_cmd_vel(0.0, 0.0)
                self.sleep(0.2)
            if moved:
                self.sleep(0.5)                # 等底盘停稳再量/再抓

    def _rotate_by(self, dyaw, tol=0.03, timeout=8.0):
        """原地转过 dyaw（弧度，正=左转）；用 TF 的 map 偏航闭环，不 spin ✓"""
        if abs(dyaw) < 1e-3:
            return True
        rp0 = self._robot_map_pose()
        if rp0 is None:
            return False
        target_yaw = rp0[2] + dyaw
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            rp = self._robot_map_pose()
            if rp is None:
                break
            err = self._normalize_angle(target_yaw - rp[2])
            if abs(err) < tol:
                break
            self._publish_cmd_vel(0.0, clamp(1.2 * err, -CREEP_W_MAX, CREEP_W_MAX))
            self.sleep(CREEP_DT)
        for _ in range(3):
            self._publish_cmd_vel(0.0, 0.0)
            self.sleep(0.2)
        rp = self._robot_map_pose()
        return rp is not None and abs(self._normalize_angle(target_yaw - rp[2])) < 0.08

    def _survey_stop_ok(self, best):
        """扫视"可以停了"的判据：已经有一个**真能用**的候选。

        ★ 为什么不是"检出任何目标就停"（那样等于没扫）：
          邻桌幻影、桌上的干扰物（bowl/beer…夹不住）都会让"有目标"成立，
          但停下来的结果是**目标选择被幻影劫持** ✗。判据必须跟 choose_target 的
          可用条件一致：能夹（objects.yaml graspable）+ 置信度够 + 不在邻桌脚印里 ✓
        """
        rp = self._robot_map_pose()
        for _conf, q, cls in best.values():
            if not self.graspable.get(cls, cls not in NON_GRASPABLE):
                continue
            if _conf < min(self.min_confidence, CLOSE_MIN_CONFIDENCE):
                continue
            if rp is not None and self._support_rank(q) == 2:
                continue          # 落在邻居餐桌脚印里 ⇒ 不算"扫到了"
            return True
        return False

    def survey_targets(self, yaws=SURVEY_YAWS, stop_early=None):
        """在原地【转几个角度】各检测一次，按 map 坐标合并去重 ✓（不移动底盘 ✗）

        为什么要它（实测）：水平视野只有 62°（半角 31°），装不下 1.2 m 长的餐桌 ✗；
        而小物体（番茄罐 66×101 mm）在 1 m 外只有 ~30 像素、检测器直接认不出 ✗
        → 在观察位/近看位原地转 ±25° 各拍一次，并集覆盖 ±56°（够 0.67 m 处 ±41.9°）✓
        ★ 2026-09-18：0° 排在最前（直拍最正、检测与测距质量最好），
          并且【扫到能用的候选就停】—— 原来固定跑满 3 个角度 = 3 次视觉调用（~1 分钟）✗
        ★ 它只转底盘，不做任何导航 ✓（旧实现在"没目标"时导航到近看位，
          那个点算错过 ⇒ 车开到桌子另一边撞桌子，见 CLOSE_LOOK_EDGE 的说明）
        合并规则：同类且 map 坐标相距 < SURVEY_MERGE_DIST ⇒ 同一个物体，保留置信度高的那次 ✓
        返回：换算到**当前**底盘系的 GraspTargetStamped 列表（可直接喂给 choose_target）✓
        """
        if stop_early is None:
            stop_early = not SURVEY_ALL_ANGLES
        best = {}       # (class, 约化后的 map 位置) → (conf, map_xy, class)
        rp_od = self._robot_map_pose()       # ★ 下面算的是 map 坐标 ⇒ 必须用 map 位姿
        rotated = 0.0                        # 当前已转到的偏航（回正用；没转过就不用回 ✓）
        for i, dyaw in enumerate(yaws):
            if i > 0:
                ok = self._rotate_by(dyaw - yaws[i - 1])
                rotated = dyaw
                self.log.info("  扫视: 转到 {:+.0f}°{}".format(math.degrees(dyaw),
                                                              "" if ok else "（没转到位，继续）"))
            rp_od = self._robot_map_pose() or rp_od
            if rp_od is None:
                break
            tg = self.fetch_targets()
            self.log.info("  扫视[{}/{}] {:+.0f}°: {} 个目标".format(
                i + 1, len(yaws), math.degrees(dyaw), len(tg)))
            for t in tg:
                q = self._to_map((t.point.x, t.point.y), rp_od)
                key = None
                for k, (conf_k, xy_k, cls_k) in best.items():
                    if cls_k == t.class_id and math.hypot(q[0] - xy_k[0], q[1] - xy_k[1]) < SURVEY_MERGE_DIST:
                        key = k
                        break
                if key is None:
                    best[(t.class_id, round(q[0], 2), round(q[1], 2))] = (
                        float(t.confidence), q, t.class_id)
                    self.log.info("    + {} conf={:.2f} map({:+.3f},{:+.3f})".format(
                        t.class_id, t.confidence, q[0], q[1]))
                elif float(t.confidence) > best[key][0]:
                    cc, _, cl = best[key]
                    best[key] = (float(t.confidence), q, cl)
                    self.log.info("    ↑ {} 更新为 conf={:.2f}".format(t.class_id, t.confidence))
            if stop_early and self._survey_stop_ok(best):
                if i == 0:
                    self.log.info("  直拍就有可用的目标 → 不再转角补扫 ✓（省 {} 次视觉调用）"
                                  .format(len(yaws) - 1))
                else:
                    self.log.info("  {:+.0f}° 补扫到可用的目标 → 停止继续扫 ✓"
                                  .format(math.degrees(dyaw)))
                break
            if not stop_early and i + 1 < len(yaws):
                # 跑满模式：把后面角度的用途说清楚（免得看日志的人以为白扫）✓
                self.log.info("  （跑满 {} 个角度以覆盖整张桌子：靠边的物体只在某个角度才落在"
                              "画面中心 ✓）".format(len(yaws)) if i == 0 else
                              "  （继续扫下一个角度 ✓）")
        # 转回原朝向（**只在真的转过去过时**；不然白转一次，还多花时间 ✗）
        if rotated != 0.0 and self._rotate_by(-rotated) is False:
            self.log.warn("  扫视: 转回原朝向失败（继续）")
        rp_od = self._robot_map_pose() or rp_od     # ★ 同上：q 是 map 坐标，回换算也要 map 位姿
        out = []
        for _conf, q, cls in best.values():
            xy = self._to_base(q, rp_od) if rp_od else None
            if xy is None:
                continue
            t = self._mk_target(cls, _conf, xy)
            out.append(t)
        out.sort(key=lambda t: -t.confidence)
        self.log.info("  扫视合并后: {} 个（去重前 {}）".format(len(out), len(best)))
        return out

    def _fresh_target(self, class_id, match_radius=LABEL_FLICKER_RADIUS):
        """在【抓取点上】重新量一次目标 → (target, obstacles, source)。

        ★ 为什么必须有这一步（用户 2026-09-14 现场观察：夹爪下去时碰到了盒子顶面）：
          原来最后发给抓取服务的点取自 self._final_map —— 那是【观察位那次】的估计
          （离物体 1 m 以外）再用里程计外推出来的 ✗。中间虽然在站位/爬到位后又拍过照，
          但那些结果只用来挑目标、更新跟踪，**没有更新最终那个点** ✗
          → 手指实际是朝着"1 m 外看出来的位置"下探的，差一两厘米就落到盒子顶面上了
            （盒子只有 38 mm 厚，两指之间的走廊容差只有 ±21 mm）。
        ★ 现在：抓取点上这一帧的检测结果（已经是当前 base_footprint 系、时间戳最新）
          直接就是抓取点 ✓ 不再走 map/odom 往返换算（那一步本身也会引入误差）。
          标签逐帧会抖 → 同名类别优先，找不到就退一步用"离上次跟踪位置最近"的那个。
        """
        tg = self.fetch_targets()
        self.last_targets = tg
        if not tg:
            return None, None, "none"
        same = [t for t in tg if t.class_id == class_id]
        tr = self._track.get(class_id)
        # ★ 同名但位置离谱的检测不要（远处同类别物体 / 误检）：与上次跟踪位置比一比，
        #   超过 match_radius 就当成"不是同一个物体"（爬行期间物体不可能移动 35 cm）
        if same and tr is not None:
            rp_od = self._robot_map_pose()       # ★ 修正：tr[1] 是 map 坐标，原来喂 odom ⇒ 比错
            if rp_od is not None:
                near = []
                for t in same:
                    q = self._to_map((t.point.x, t.point.y), rp_od)
                    if math.hypot(q[0] - tr[1][0], q[1] - tr[1][1]) <= match_radius:
                        near.append(t)
                if not near:
                    self.log.warn("  抓取点上认出的 {} 离跟踪位置超过 {:.2f} m → 不采信"
                                  .format(class_id, match_radius))
                same = near
        if not same and tr is not None:
            rp_od = self._robot_map_pose()       # ★ 同上：与 tr[1]（map 坐标）比距离
            if rp_od is not None:
                best, bd = None, match_radius
                for t in tg:
                    q = self._to_map((t.point.x, t.point.y), rp_od)
                    d = math.hypot(q[0] - tr[1][0], q[1] - tr[1][1])
                    if d < bd:
                        best, bd = t, d
                if best is not None:
                    self.log.warn("  抓取点上没认出 \"{}\" → 用 {} 的检测代替"
                                  "（相距 {:.3f} m ≤ {:.2f} m，判定为同一物体标签抖动 ✓）"
                                  .format(class_id, best.class_id, bd, match_radius))
                    same = [best]
                else:
                    self.log.warn("  抓取点上没认出 \"{}\"，且别的检测都在 {:.2f} m 以外 "
                                  "⇒ 那是另一个物体，不代替 ✗".format(class_id, match_radius))
        if not same:
            return None, None, "none"
        best = max(same, key=lambda t: t.confidence)
        return best, [t for t in tg if t is not best], "vision"

    def _nudge(self, class_id, goal_d, rounds=25):
        """按【里程计跟踪】把物体推到 base(goal_d, 0)：补偿刚测出来的残差。

        不再调视觉（一次 25~30 s 太贵），只走 measure() 的 odom 快路径 ✓
        """
        for _ in range(rounds):
            xy = self.measure(class_id)
            if xy is None:
                return False
            d = math.hypot(*xy)
            bearing = math.atan2(xy[1], xy[0])
            if abs(d - goal_d) < CREEP_TOL and abs(bearing) < CREEP_BEARING_TOL:
                return True
            if abs(bearing) > CREEP_SPIN_BEARING:
                v, w = 0.0, clamp(CREEP_KW * bearing, -CREEP_W_MAX, CREEP_W_MAX)
            else:
                v = clamp(CREEP_KV * (d - goal_d), -CREEP_V_MAX, CREEP_V_MAX)
                w = clamp(CREEP_KW * bearing, -CREEP_W_MAX, CREEP_W_MAX)
            self._publish_cmd_vel(v, w)
            self.sleep(CREEP_DT)
        for _ in range(3):
            self._publish_cmd_vel(0.0, 0.0)
            self.sleep(0.2)
        return False

    def _grasp_with_offsets(self, target, obstacles, standoff=None):
        """调抓取服务，并且**用"夹到没有"判成败**；空合就换偏置重抓。

        返回 (ok, stage, msg, tried_offsets)：
          ok=True  ⇔ 某一次实测到"手指被挡住"（=真夹到了）✓
          ok=False 时 stage 保留最后一次服务返回的 stage（0 表示"服务说成功但其实是空合"）

        ★ 为什么不能拿 success/stage 当成功判据（这是本项目最贵的坑）：
          MTC 的 attach object 只是**规划场景里的逻辑附着**（pick_and_place.cpp），
          物体没夹住也照样 success=true / stage=0；实测 sweeoffset 扫描里 9 档全部
          "success=True stage=0"，而两指间距全程 = 指令值（空合）✗
          ⇒ 唯一的物理判据是"手指有没有被挡住"（两指真实间距 vs 指令间距）✓

        ★ 偏置阶梯的方向依据（2026-09-16 sweep_offset 实验 + 2026-09-18 现场观察）：
          径向 ±40 mm 九档全空合 ⇒ 那次的径向不是主因；而合爪方向（base y）余量最窄
          （罐 = ±7 mm、糖盒 = ±21 mm）⇒ 侧向优先；再把径向 ±20/±45 mm 作为兜底 ✓
        ★ 2026-09-18 第二次实跑（用户）：**"第一次抓取完之后，番茄罐发生了位移，
          导致后几次基本是空抓"** ⇒ 每档重抓【之前】必须用视觉重测一次 ✓
          为什么必须：上一档的手指可能已经碰到并把物体推走，而阶梯是加在**旧点**上的 ——
          拿着过期点试 4 档等于白试 ✗（实测一次视觉调用只要 5~6 s，完全付得起 ✓）
          重测还能顺手回答"是不是我把它推走的"（日志里打出与上一档基准的位移）✓
        ★ 偏置加在【发给服务的抓取点】上（base_footprint 系）⇒ 等效于把机械臂
          相对物体平移，不用动车、不用重新导航（每次只是一个新的 MTC 周期）✓
        """
        offsets = GRASP_RETRY_OFFSETS[:max(1, int(MAX_GRASP_TRIES))]
        cls = target.class_id
        # ★★ 桌腿基准（2026-09-19）：抓之前先量"车在 map 里的真实偏航"与 TF 报的差多少，
        #   并把偏差注入 grasp_node 的 object_yaw_map ⇒ 合拢轴不再受 AMCL 偏航误差影响 ✓
        #   为什么必须在【抓之前】做：合拢轴错 φ 时两指会压在物体顶面把物体推走
        #   （50×97 的罐 φ>19.8° 就几何上夹不进去）⇒ 事后再修就已经把物体毁了 ✗
        self._apply_table_yaw_fix()
        ok, stage, msg, closure = False, 0, "", {}
        tried = []
        est = (target.point.x, target.point.y)   # 视觉估计（绝对值含系统偏差 ✗）
        bias = (0.0, 0.0)                        # ★ 相对视觉估计的修正偏置（跟随位移学到 ✓）
        prev_est = est
        # ★★ 2026-09-19：**跟随位移**（只在"被推走"时启用）—— 这是目前唯一可信的方向信息
        #   物理推导（现场日志已验证方向）：
        #     物体被某个手指/指根碰到时，是**被推向它自己所在的那一侧**（手指从轴线往外推）✓
        #     ⇒ 观察到的位移方向 **就是** 物体真实位置所在的方向 ✓
        #   为什么可信：位移是**差分**量 —— 视觉的绝对偏差（外参平移那类）在两次测量里相同，
        #     相减就抵消了 ✓✓（现场：5 次尝试每次都把罐子推向 −y，方向完全一致 ✓）
        #   下一档就按 "probe + 2×(观测位移)" 走过去（≈ 被推的距离 + 原来的偏差）✓
        following = False       # 一旦观测到"被推走"，就切到"跟随位移"模式（不再用固定阶梯）✓
        for i, (dx, dy) in enumerate(offsets, start=1):
            if i > 1:
                # ── 重测：物体可能被上一档推走了（现场实证）──────────────────
                fresh, obs_f, _src = self._fresh_target(cls)
                if fresh is not None:
                    new_est = (fresh.point.x, fresh.point.y)
                    # 位移 = 新测 − 上一档测到的位置（**差分** ⇒ 视觉的系统偏差抵消 ✓✓）
                    mv_vec = (new_est[0] - prev_est[0], new_est[1] - prev_est[1])
                    mv = math.hypot(*mv_vec)
                    if mv > FOLLOW_MIN_MM:
                        # 被推动了 ⇒ 位移方向 = 物体真实所在方向 ⇒ 把它累加进偏置 ✓
                        # ★ 用【偏置】而不是"绝对点"：这样后面的 creep/重测/重新对正
                        #   都不会把学到的东西冲掉 ✓（第一版就是被 re-creep 冲掉 ✗）
                        bias = (bias[0] + FOLLOW_GAIN * mv_vec[0],
                                bias[1] + FOLLOW_GAIN * mv_vec[1])
                        bn = math.hypot(*bias)
                        if bn > FOLLOW_BIAS_MAX:            # 幅值封顶（防一次大位移把点甩飞 ✗）
                            bias = (bias[0] * FOLLOW_BIAS_MAX / bn,
                                    bias[1] * FOLLOW_BIAS_MAX / bn)
                            self.log.warn("  修正偏置幅值 {:.0f} mm 超过上限 {:.0f} mm → 封顶 ✓"
                                          .format(bn * 1000, FOLLOW_BIAS_MAX * 1000))
                        following = True
                        self.log.warn(
                            "  重测[{}]: {} base({:+.3f},{:+.3f})，被推走 {:.0f} mm（Δx={:+.0f}, "
                            "Δy={:+.0f} mm）⇒ **物体在视觉估计的 {:+.0f}/{:+.0f} 那一侧** ⇒ 修正偏置"
                            "更新为 ({:+.0f}, {:+.0f}) mm，下一档按『跟随位移』走 ✓".format(
                                i, cls, new_est[0], new_est[1], mv * 1000,
                                mv_vec[0] * 1000, mv_vec[1] * 1000,
                                mv_vec[0] * 1000, mv_vec[1] * 1000,
                                bias[0] * 1000, bias[1] * 1000))
                    else:
                        self.log.info("  重测[{}]: {} base({:+.3f},{:+.3f})（仅移动 {:.0f} mm → 上一次没碰到它，"
                                      "沿用视觉估计 + 固定阶梯 ✓）".format(
                                          i, cls, new_est[0], new_est[1], mv * 1000))
                    est = new_est
                    obstacles = obs_f or obstacles
                    # ★ 必须刷新里程计跟踪基准：物体被推走后，creep/measure 的快路径
                    #   还指着旧位置 ⇒ 不刷新会把车开到旧点去对正 ✗
                    self._remember(cls, fresh)
                    # 物体挪得比较多时，先重新对正一次再抓（creep 走里程计闭环，几秒 ✓）
                    #   ★ 偏置 bias 不受影响：它本来就是"相对视觉估计"的修正量 ✓
                    if standoff and (abs(est[1] + bias[1]) > 0.030
                                     or abs(math.hypot(est[0] + bias[0], est[1] + bias[1])
                                            - standoff) > 0.060):
                        self.log.warn("  物体已被挪动（横向 {:+.3f} m / 距离 {:.3f} vs {:.3f}）"
                                      "→ 重新微调对正一次 ✓".format(
                                          est[1] + bias[1],
                                          math.hypot(est[0] + bias[0], est[1] + bias[1]), standoff))
                        self.creep(cls, standoff)
                        xy = self.measure(cls)
                        if xy is not None:
                            est = xy
                            self.log.info("  对正后 {} 在 base({:+.3f},{:+.3f})"
                                          "（修正偏置仍为 ({:+.0f}, {:+.0f}) mm ✓）".format(
                                              cls, est[0], est[1], bias[0] * 1000, bias[1] * 1000))
                        # ★ 重新对正 = 又转了一次车 ⇒ 桌腿基准要重量一次（转车不改偏差 e，
                        #   但重新量一遍可以把"AMCL 在这期间漂了"也一起修掉 ✓）
                        self._apply_table_yaw_fix(verbose=False)
                else:
                    self.log.warn("  重测[{}]: 这次没认出 {} → 沿用上一档基准 + 偏置（精度差 ✗）"
                                  .format(i, cls))
            prev_est = est
            if following:
                dx, dy = 0.0, 0.0       # 跟随模式下只用"跟过去"的点，不再叠固定偏置 ✓
            tgt = self._mk_target(cls, target.confidence,
                                  (est[0] + bias[0] + dx, est[1] + bias[1] + dy))
            self.log.warn("  重抓[{}/{}]：视觉估计 base({:+.3f},{:+.3f}) + 修正偏置 "
                          "({:+.0f}, {:+.0f}) mm + 阶梯 (Δx={:+.0f}, Δy={:+.0f} mm) → "
                          "base({:+.3f},{:+.3f})".format(
                              i, len(offsets), est[0], est[1],
                              bias[0] * 1000, bias[1] * 1000, dx * 1000, dy * 1000,
                              tgt.point.x, tgt.point.y))
            tried.append((dx, dy))
            obs_t = [self._mk_target(o.class_id, o.confidence,
                                     (o.point.x + dx, o.point.y + dy)) for o in obstacles]
            now = self.node.get_clock().now().to_msg()
            tgt.header.stamp = now
            for o in obs_t:
                o.header.stamp = now
            ok_r, stage, msg, closure = self.call_grasp(tgt, obs_t)
            if closure.get("verdict") == "contact":
                self.log.info("  ✓ 重抓成功：第 {} 档偏置 (Δx={:+.0f}, Δy={:+.0f} mm) 时手指被挡住"
                              " ⇒ 这一档就是物体的估位（可据此标定视觉的系统偏差 ✓）"
                              .format(i, dx * 1000, dy * 1000))
                return True, stage, msg, tried
            if ok_r and closure.get("verdict") != "contact":
                self.log.warn("  ⚠ 服务报 success（stage={}）但实测是【{}】⇒ 不算成功，"
                              "继续换偏置 ✗".format(stage, closure.get("verdict")))
            if closure.get("verdict") == "unclosed" and stage != 0:
                # 规划/执行失败（夹爪没动）不是"物体位置不对"⇒ 换偏置多半也没用，
                # 但仍然试下一档（有的档位解不出来、下一档可能就解出来了 ✓）
                self.log.warn("  这一档夹爪没动（unclosed，stage={}）→ 试下一档偏置".format(stage))
            if closure.get("verdict") == "wrong_object":
                # ★ 实测间距比目标窄边明显宽 ⇒ 夹住的是**另一个更宽的物体** ✗
                #   这不是"位置偏了"，换偏置毫无意义 ⇒ 直接停、交回上层（换目标/重测）✓
                #   现场教训（2026-09-19 10:42）：视觉把番茄罐标成了 potted_meat_can，
                #   这一档合到 61.3 mm（目标窄边 50）却被旧判据当成"夹到了" ⇒ 报成功 ✗✗
                self.log.error("  ✗ 夹住的物体比目标类别窄边宽 ⇒ 抓错对象了（多半是识别/标签问题，"
                               "不是位置问题）⇒ 停止偏置扫描，交回上层重新识别 ✗")
                return False, stage, msg, tried
            if stage != 0:
                # 规划/执行真失败（或目标被拒）→ 换偏置没用，交给调用方换目标 ✗
                self.log.warn("  服务返回 stage={}（{}）→ 停止偏置扫描".format(
                    stage, STAGE_TEXT.get(stage, "未知")))
                return False, stage, msg, tried
        self.log.error("  {} 档偏置都没夹到（最后一档实测 {}）⇒ 该点位的位姿估计不可信 ✗；"
                       "下一步建议：看视觉日志的『桌面剔除/地面测距 vs 掩码法』那几行，"
                       "偏差方向就是标定依据".format(len(tried), closure.get("why", "?")))
        return False, stage, msg, tried

    def call_grasp(self, target, obstacles):
        """调 /grasp_fixed_object 一次 → (success, stage, message, closure)。

        ★ closure = classify_closure(...) 的结果（dict）：
          "contact" = 手指被挡住（真夹到了）；"empty" = 两指之间是空的 ✗
          调用方必须按 closure 判成败 —— **不能只看 success/stage**（见常量区说明）✓
        """
        if not self.wait_client(self.grasp_client, 30.0):
            self.log.error("抓取服务 {} 不可用（grasp_service.launch.py 起了吗？）"
                           .format(GRASP_SERVICE))
            return 0, 1, "grasp service unavailable", {"verdict": "unknown"}
        req = GraspFixedObject.Request()
        req.target = target
        req.obstacles = obstacles
        self.log.info("调用 {} : target={} 轴心点({:.3f}, {:.3f}, {:.3f}) 障碍 {} 个".format(
            GRASP_SERVICE, target.class_id, target.point.x, target.point.y, target.point.z,
            len(obstacles)))
        q0 = self._finger_q
        q02 = self._finger_q2
        span, cmd_mm = None, None
        sz = self.object_sizes.get(target.class_id)
        if sz:
            span = min(sz[0], sz[1])
            squeeze = self.close_squeeze          # 与 pick_and_place.cpp 同源（都读配置）
            cmd = 0.5 * span - squeeze
            cmd_mm = cmd * 2000.0
            self.log.info("  预期合爪: 物体窄边 {:.4f} m − 干涉 {:.4f} → 关节 {:.4f}"
                          "（两指间距 {:.1f} mm）".format(span, squeeze, cmd, cmd_mm))
        gap0 = self.finger_gap_mm()
        tcp0 = self.tcp_pose_base()
        # ★ 记轨迹范围（min/max），不能只留"z 最小"的那一次 ✗：
        #   home 位姿的指尖 z(0.914) 比抓取位姿(0.9195) 还低 → 只留最小 z 会永远
        #   记成 home 位姿（实测踩过，害我误判"机械臂没到位" ✗）
        tcp_xs, tcp_ys, tcp_zs = [], [], []
        # ★ 合拢时刻判定（2026-09-15 现场）：原来时间戳/指尖 xyz/间距是分开的几组列表，
        #   判"合拢那一刻指尖在哪"必须三者【对齐】⇒ 另存一份 (t, x, y, z, gap_mm) 的对齐样本
        #   （见 analyze_closure / format_closure_report）。上面三组列表保持原样不动，
        #   免得动到"指尖轨迹/落点核对"那些已经在用的逻辑 ✗。
        #   时间用 monotonic：这里只做区间相减，不受系统时钟跳变影响 ✓
        samples = []
        d_best, ax_best = None, None
        if gap0 is not None:
            self.log.info("  合爪前两指真实间距 = {:.1f} mm；指尖平面 base({:+.3f},{:+.3f},{:.3f})"
                          .format(gap0, tcp0[0], tcp0[1], tcp0[2]) if tcp0 else
                          "  合爪前两指真实间距 = {:.1f} mm".format(gap0))
        fut = self.grasp_client.call_async(req)
        qmin, qmax = self._finger_q, self._finger_q          # 夹持过程中采样
        # ★ 2026-09-19 修：第二个手指也要**同时**采样 ✗
        #   之前那条"两指关节实测"拿 j1 的【历史最小值】比 j2 的【当前值】——
        #   合爪结束后 MTC 会把手重新张开 ⇒ 比出来凭空多 8.8 mm 的假警报 ✗
        q2min, q2max = getattr(self, "_finger_q2", None), getattr(self, "_finger_q2", None)
        gmin, gmax = gap0, gap0
        deadline = time.time() + GRASP_SERVICE_TIMEOUT
        while rclpy.ok() and not fut.done() and time.time() < deadline:
            q = self._finger_q
            if q is not None:
                qmin = q if qmin is None else min(qmin, q)
                qmax = q if qmax is None else max(qmax, q)
            q2 = getattr(self, "_finger_q2", None)
            if q2 is not None:
                q2min = q2 if q2min is None else min(q2min, q2)
                q2max = q2 if q2max is None else max(q2max, q2)
            g = self.finger_gap_mm()
            if g is not None:
                gmin = g if gmin is None else min(gmin, g)
                gmax = g if gmax is None else max(gmax, g)
            tp = self.tcp_pose_base()
            if tp is not None:
                tcp_xs.append(tp[0])
                tcp_ys.append(tp[1])
                tcp_zs.append(tp[2])
                samples.append((time.monotonic(), tp[0], tp[1], tp[2], g))
                d_now = math.hypot(tp[0] - target.point.x, tp[1] - target.point.y)
                if d_best is None or d_now < d_best:      # 最接近契约点那次的姿态
                    d_best = d_now
                    ax_best = self.tcp_axes_base()
            time.sleep(0.1)
        # ★ 合拢时刻日志：放在超时判断【之前】—— 就算这次调用超时/失败，
        #   "两指开始合拢那一刻指尖在哪"也照样要落盘（这条正是判别空合的关键 ✓）
        for ln in format_closure_report(analyze_closure(samples),
                                        target.point.x, target.point.y):
            self.log.info(ln)
        if not fut.done():
            self.log.error("抓取服务超时（>{:.0f}s）".format(GRASP_SERVICE_TIMEOUT))
            return 0, -1, "timeout", {"verdict": "unknown"}
        # ★ 手指关节实测行程 → 直接判定"夹到了没有"
        if qmin is not None and qmax is not None and abs(qmax - qmin) > 1e-4:
            gap = qmin * 2000.0
            self.log.info("  合爪实测: 关节 起始 {:.4f} → 最终 {:.4f}（两指间距 {:.1f} mm）".format(
                qmax, qmin, gap))
            if span:
                verdict, why = classify_closure(cmd_mm, gap, span * 1000.0,
                                                open_mm=(gmax if gmax is not None else gap0))
                (self.log.info if verdict == "contact" else self.log.warn)("  ★ " + why)
        else:
            self.log.warn("  合爪实测: 没采到手指关节变化（/joint_states 没起来？）")
        if tcp_xs:
            self.log.info("  指尖轨迹(本次调用): x {:.3f}→{:.3f}（最远 {:.3f}）  z {:.3f}→{:.3f}"
                          "  ← 最远 x 应≈抓取点的 x ✓"
                          .format(min(tcp_xs), max(tcp_xs), max(tcp_xs),
                                  min(tcp_zs), max(tcp_zs)))
        # ★ 两指真实间距：用来判断"两指是否对称合拢"以及"有没有夹到东西"
        #   两指对称时：期望间隙 = 2 × joint1（URDF 里 joint2 是 joint1 的镜像 mimic ✓）
        #   ← 这里原来写成 q + 40 mm（当成只有左手在动）→ 会误报 mimic 失效 ✗（实测踩过）
        if gmin is not None and gmax is not None and abs(gmax - gmin) > 0.5:
            self.log.info("  两指真实间距: 起始 {:.1f} mm → 最小 {:.1f} mm".format(gmax, gmin))
            # ★ 两指关节实测（2026-09-18 起 joint2 也是受控关节 ⇒ 这才有意义）
            if qmin is not None and q2min is not None:
                # 两指各自的历史最小值相加 = "合爪最紧那一刻"的真实开口 ✓（与 TF 的最小间距同义）
                gap_j = (qmin + q2min) * 1000.0
                self.log.info("  两指关节实测: j1最小={:.4f} j2最小={:.4f} → (q1+q2)×1000 = {:.1f} mm"
                              "（与 TF 量出的最小间距 {:.1f} mm 相差 {:+.1f} mm）{}".format(
                                  qmin, q2min, gap_j, gmin, gap_j - gmin,
                                  "  ✓ 两指对称 ✓" if abs(gap_j - gmin) <= 3.0 else ""))
                if abs(gap_j - gmin) > 3.0:
                    self.log.warn("  ✗ 两指不对称（关节算 {:.1f} mm vs TF 量 {:.1f} mm）⇒ 第二指没跟上 "
                                  "joint1（finger_mimic_relay / finger2_controller 没起作用）✗".format(
                                      gap_j, gmin))
            if qmin is not None:
                expect = 2.0 * qmin * 1000.0       # 两指对称 = 2×关节值
                if abs(gmin - expect) > 8.0:
                    self.log.warn("  ✗ 真实间隙 {:.1f} mm 与「两指对称」应有的 {:.1f} mm 不符 "
                                  "→ 两指不对称（mimic/装配有问题）".format(gmin, expect))
                else:
                    self.log.info("  ✓ 两指对称合拢（真实间隙 ≈ 2×关节值，mimic 正常）")
        res = fut.result()
        if res is None:
            self.log.error("抓取服务无响应")
            return 0, -1, "no response", {"verdict": "unknown"}
        self.log.info("抓取服务返回: success={} stage={} [{}] {}".format(
            res.success, res.stage, STAGE_TEXT.get(res.stage, "未知"), res.message))
        # ★ 落点核对（2026-09-15）：契约点 vs 抓手**实测**位置，两者都在 base_footprint 里。
        #   这是"偏差到底在契约点（视觉）这一侧，还是契约点→抓手（规划/执行）这一段"的
        #   唯一直接判据 —— 之前一直缺这一条，所以只能在两侧之间猜 ✗
        #   注意：服务返回时通常已经抬升过，z 会变 ⇒ 只比 x/y ✓
        # ★ 姿态核对（2026-09-15）：位置对但角度错时，前面所有对数都看不出来 ✗
        if ax_best is not None:
            zax, yax = ax_best
            zt = math.degrees(math.acos(max(-1.0, min(1.0, abs(zax[2])))))
            yt = math.degrees(math.asin(max(-1.0, min(1.0, abs(yax[2])))))
            self.log.info(
                "  ★ 姿态核对: 接近方向 z 轴 base({:+.2f},{:+.2f},{:+.2f}) 偏竖直 {:.1f}°；"
                "两指合拢方向 y 轴 base({:+.2f},{:+.2f},{:+.2f}) 偏水平 {:.1f}°{}".format(
                    zax[0], zax[1], zax[2], zt, yax[0], yax[1], yax[2], yt,
                    "   ✓ 顶抓姿态正常" if (zt < 8.0 and yt < 8.0) else
                    "   ✗ 角度偏了：两指不是水平合拢 ⇒ 会先碰罐口沿/挤走物体（现场观察吻合）"))
        # ★ 合拢轴核对（2026-09-19，potted_meat_can 那次"停在物体顶上、合爪合空"的根因候选）：
        #   姿态核对只证明"两指水平合拢"，**证明不了"合拢轴 = 物体窄边那一对"** ✗。
        #   而合拢轴在 C++ 侧是【算出来的】，不是量出来的：
        #       box_yaw = object_yaw_map(常数 0；本世界物体都按出厂姿态、与 map 轴对齐)
        #                 − robot_yaw_map(TF map←base_footprint)
        #   ⇒ 整车朝向链挂在这个 TF 偏航上。可是站位的 yaw 是驱动层按【桌沿法线】命令的
        #   （法线来自地图、轴对齐 ✓），于是
        #       φ = TF 报的 map 偏航 − 命令的站位 yaw
        #   就是"朝向链的不一致量"：要么车真没转到命令 yaw（Nav2 余差/微调旋转），
        #   要么 AMCL 的偏航错了 —— 两种情况下"合拢轴对齐物体窄边"这个前提都不再可靠 ✗。
        #   为什么要当成硬指标：长方体需要的开口 = d·|cosφ| + w·|sinφ|
        #   （φ = 合拢轴与窄边的夹角，d/w = 目录表的 depth/width）
        #     potted_meat_can 50×97 mm：φ=15° → 73.4 mm（贴上限、只能啃两个角）
        #                                φ=19.8° → 80.2 mm = 开口上限 ⇒ 再偏就**几何上夹不进去**
        #   那时两指必然压在物体顶面把它推走，然后"合到指令间距但两指之间是空的"
        #   ⇒ 日志里的 [empty]、现场看到的"手停在罐顶、夹爪合得宽"都能对上 ✓
        #   形状闸门（d/w 之比 < 1.15 视为圆柱/近方）：本目录表里比值 = 1.00 的那几个
        #   （coke can / tomato_soup_can / chips_can / master_chef_can / beer / bowl / apple）
        #   都是圆柱或球 ⇒ 合拢轴偏 φ 不改变需要的开口，本条降级为提示 ✓；
        #   而 gelatin_box(1.16) / mustard_bottle(1.64) / sugar_box(2.34) / cracker_box(2.63)
        #   都过闸门 ⇒ 一律按长方体算（保守）✓
        try:
            rp_ck = self._robot_map_pose()
            n_ck = self.obs_normal
            if rp_ck is not None and n_ck is not None:
                yaw_cmd = math.atan2(-n_ck[1], -n_ck[0])      # 站位朝桌子 → 车头沿 −法线
                # 归一化到 (−π, π]（不用 self._normalize_angle：本函数在离线自测里
                # 会被喂假 self，凡是 self.xxx 都得先在假对象里存在 ✗）
                dphi = math.atan2(math.sin(rp_ck[2] - yaw_cmd), math.cos(rp_ck[2] - yaw_cmd))
                sz_ck = self.object_sizes.get(target.class_id)
                # ★ 已注入的桌腿基准修正 e（见 _apply_table_yaw_fix）：C++ 侧 box_yaw 已经
                #   按它修正过 ⇒ 真正决定合拢轴的是 φ_eff = φ_raw − e ✓
                e_inj = getattr(self, "_last_yaw_inject", None) or 0.0
                dphi_eff = math.atan2(math.sin(dphi - e_inj), math.cos(dphi - e_inj))
                extra = ""
                if sz_ck:
                    d0, w0 = float(sz_ck[0]), float(sz_ck[1])
                    if min(d0, w0) > 0 and max(d0, w0) / min(d0, w0) < 1.15:
                        extra = ("（物体 {:.0f}×{:.0f} mm 近方/圆柱 ⇒ 合拢轴偏了不改变开口 ✓）".format(
                            d0 * 1000, w0 * 1000))
                    else:
                        span_phi = closing_span_mm(d0, w0, dphi_eff) / 1000.0
                        extra = ("；长方体 {:.0f}×{:.0f} mm 按【生效】的 φ={:+.1f}° 算需要的开口 = "
                                 "{:.1f} mm（上限 {:.0f} mm）{}".format(
                                     d0 * 1000, w0 * 1000, math.degrees(dphi_eff),
                                     span_phi * 1000, GRIPPER_OPEN_M * 1000,
                                     "   ✗ 超过开口 ⇒ 几何上夹不进去：两指会压在物体顶面把它推走，"
                                     "然后合到指令间距而两指之间是空的（现场「合得宽」就是它）"
                                     if span_phi > GRIPPER_OPEN_M else
                                     "   ✓ 还在开口内（但 φ=19.8° 就是上限）"))
                self.log.info("  ★ 合拢轴核对: 站位命令 yaw={:+.3f} rad，TF 报的 map←base_footprint "
                              "yaw={:+.3f} rad → 原始差 φ={:+.1f}°；桌腿基准修正 e={:+.1f}° "
                              "⇒ 生效 φ={:+.1f}°{}".format(
                                  yaw_cmd, rp_ck[2], math.degrees(dphi), math.degrees(e_inj),
                                  math.degrees(dphi_eff), extra))
                if abs(dphi_eff) > math.radians(5.0):
                    self.log.warn("  ⚠ 生效 φ>5° ⇒ 合拢轴仍与物体窄边不一致（桌腿基准没量到，"
                                  "或量到了但不够）⇒ 先查清再抓，别硬抓（会把物体推走 ✗）")
                elif abs(dphi) > math.radians(5.0):
                    self.log.info("  ✓ 原始 φ 偏了 {:.1f}°，但已被桌腿基准补偿到 {:.1f}°".format(
                        math.degrees(dphi), math.degrees(dphi_eff)))
        except Exception as e:                                   # noqa: BLE001
            self.log.warn("  合拢轴核对失败: {}: {}".format(type(e).__name__, e))
        # ★ 落点核对：必须用【轨迹里最接近契约点的那一次采样】，不能用服务返回后的采样 ✗
        #   实测踩坑：服务返回时已经后退 16 cm ⇒ 报出"差 -160 mm"的假警 ✗
        if tcp_xs:
            i = min(range(len(tcp_xs)),
                    key=lambda k: math.hypot(tcp_xs[k] - target.point.x,
                                             tcp_ys[k] - target.point.y))
            ddx = tcp_xs[i] - target.point.x
            ddy = tcp_ys[i] - target.point.y
            dd = math.hypot(ddx, ddy)
            self.log.info(
                "  ★ 落点核对: 契约点 base({:+.3f},{:+.3f}) vs 抓手最接近处 base({:+.3f},{:+.3f}) "
                "→ 差 ({:+.0f},{:+.0f}) mm（|d|={:.0f} mm）{}".format(
                    target.point.x, target.point.y, tcp_xs[i], tcp_ys[i],
                    ddx * 1000.0, ddy * 1000.0, dd * 1000.0,
                    "   ✓ 规划/执行确实把夹爪送到了契约点 ⇒ 剩下的是【契约点=视觉】这一侧"
                    if dd <= 0.015 else
                    "   ✗ 夹爪没到契约点 ⇒ 偏置在【契约点→夹爪】这一段（规划/执行）"))
        # ★ 唯一判据：两指真实间距 vs 指令间距（见 classify_closure 的说明）
        meas = gmin if gmin is not None else (qmin * 2000.0 if qmin is not None else None)
        open_mm = gmax if gmax is not None else gap0     # 合爪前的张开间距（识别"没合爪"用 ✓）
        verdict, why = classify_closure(cmd_mm, meas, None if span is None else span * 1000.0,
                                        open_mm=open_mm)
        self.log.info("  ★ 夹到没有: [{}] {}".format(verdict, why))
        closure = {"verdict": verdict, "why": why, "measured_mm": meas,
                   "commanded_mm": cmd_mm, "open_mm": open_mm,
                   "span_mm": (None if span is None else span * 1000.0)}
        return res.success, res.stage, res.message, closure

    # ══════════════ ⑤ 主流程 ══════════════
    def run(self, observation_pose=None, skip_nav=False, no_creep=False, park_override=None):
        """跑完整个抓取阶段。

        observation_pose = (x, y, yaw)，或 None = 由支撑面自己算（推荐）：
                             桌心沿"机器人这一侧的桌沿法线"外推 OBSERVATION_DIST，
                             朝向面对桌心 → 与站位同一条直线，机械臂全程对正桌子 ✓
        park_override    = (x, y, yaw) 或 None；给了就固定用这个站位（调试用，
                           例如旧工作区那套 (3.0, 2.51, -π/2)）
        """
        self.log.info("=" * 55)
        self.log.info("Phase 2 抓取阶段开始（只用视觉；不读 gz 真值 ✗）")
        # ★ 2026-09-19：开场把"要抓哪几类"说清楚。空列表会让视觉退回 4 类默认词表
        #   （apple/coke can/bowl/banana）⇒ 桌上其它物体**永远认不出来**，
        #   而现象只是"扫视全是 0 目标"，非常难查（现场一次 8 连败就是这么来的 ✗）
        if self.target_classes:
            self.log.info("目标类别（--classes）：{}".format(", ".join(self.target_classes)))
        else:
            self.log.info("没指定 --classes ⇒ 默认抓【本桌任意物体】：请求 objects.yaml 全部 {} 类 {}"
                          "（视觉侧走闭集复核 ✓）".format(
                              len(self.all_classes), self.all_classes if len(self.all_classes) <= 6 else "（见视觉日志）"))

        if not skip_nav:
            # ★ 算观察位前等 map←base_footprint 稳定（启动时 AMCL 竞态，见 _wait_map_pose_stable）；
            #   命令行直接给了 observation_pose 时不需要它（rp0 只用于日志/法线兜底）⇒ 保持单次查询
            rp0 = (self._wait_map_pose_stable() if observation_pose is None
                   else self._robot_map_pose())
            if observation_pose is None:
                if rp0 is None:
                    self.log.error("拿不到 map←base_footprint，算不出观察位")
                    return False
                observation_pose = self.observation_pose_for(rp0)
                self.log.info("① 观察位（由桌沿法线算出，与站位同一条法线）")
            else:
                self.log.info("① 先到观察位（能看到整张桌子，便于选目标 + 拿障碍物清单）")
            # ★ 接近法线在这里【定一次】，之后近看位/站位全用它 ✓
            #   绝不再由"目标位置 → 机器人"重算（那会在视觉把邻桌物体报成本桌目标时
            #   把法线翻到桌子另一边 ⇒ 车绕到餐桌另一侧 ✗，用户现场现象）
            self.obs_normal = self._normal_from_pose(observation_pose)
            self.log.info("   观察位 = ({:.3f}, {:.3f}, yaw={:.3f})；桌沿法线 = ({:+.2f}, {:+.2f})"
                          "（本趟全程复用）".format(observation_pose[0], observation_pose[1],
                                                  observation_pose[2],
                                                  self.obs_normal[0], self.obs_normal[1]))
            if not self.navigate(*observation_pose):
                self.log.warn("观察位不可达 → 直接用站位视觉继续")
            else:
                # ★ 到达后再做一次"对准餐桌中心 + 收到 ~1.15 m"的小闭环：
                #   Nav2 容差 0.4 m 允许停偏 0.3 m（实测偏了 313 mm），桌子会偏出画面 ✗
                self.align_and_approach((TABLE_CENTER_XYZ[0], TABLE_CENTER_XYZ[1]),
                                        stand_dist=OBSERVATION_DIST)
            self.sleep(0.5)

        tried = set()                  # 已判定"不可抓"的类别（规划型失败）
        pos_rounds = 0                 # "位置型失败"重试轮数（重测后重试同一目标 ✓）
        close_looked = False           # 近看位只去一次（别反复前进/后退）
        for attempt in range(1, MAX_TARGET_TRIES + MAX_POSITION_ROUNDS + 1):
            # ══════════ 观察：在【原地】转角度扫视；不够格才沿法线前进到近看位 ══════════
            # ★ 2026-09-18 改（用户现场："第一次观察之后就导航到餐桌另一边并撞桌子"）：
            #   旧实现是"观察位拍一张 → 没目标/没候选 → **导航**到近看位(离桌心 0.68 m)"，
            #   两个副作用：
            #     ① 近看位按"离桌心"算 ⇒ 在东西侧只离桌沿 0.08 m（桌心到桌沿 0.6 m）
            #        ⇒ 目标点落在桌子脚印/Nav2 的 inscribed 区里 ⇒ DWB 到不了、
            #        车头一直朝桌子顶 ⇒ 物理上撞桌子；
            #     ② 交给 Nav2 就由它自己挑绕行方向 ⇒ 出现"开到餐桌另一边"。
            #   现在：观察位【原地】扫视（0° 直拍优先，扫到可用的就停）；
            #   只有一个能用的候选都没有时，才沿【同一条法线向前】开一小段到近看位
            #   （cmd_vel 闭环，不经过 Nav2，见 _goto_close_look）✓
            if attempt == 1:
                if skip_nav:
                    # --skip-nav：机器人已经手工停好，只测服务链路 → 不要动底盘（不转扫视）✗
                    self.log.info("② --skip-nav：直拍一次（不转角度扫视，不动底盘）")
                    targets = self.fetch_targets()
                else:
                    self.log.info("② 观察位【原地】扫视（最多 {} 个角度，扫到可用目标即停；不移动底盘）"
                                  .format(len(SURVEY_YAWS)))
                    targets = self.survey_targets()
            else:
                targets = self.fetch_targets()          # 换目标重试：直拍一张就够
            self.last_targets = targets
            if not targets:
                self.log.warn("这一趟没返回任何目标（下一步会前进到近看位再扫一次）")
            else:
                self.log.info("候选目标 {} 个（来源 vision）：{}".format(
                    len(targets),
                    ["{}({:.2f})".format(t.class_id, t.confidence) for t in targets]))
            # 选目标前先拿机器人的 map 位姿：①支撑面打分 ②站位解算 都要用
            # ★ 算站位前等它稳定（启动/重定位时 AMCL 会让它跳，见 _wait_map_pose_stable）
            rp = self._wait_map_pose_stable()
            if rp is None:
                self.log.error("拿不到 map←base_footprint 变换，算不出站位")
                return False
            # 观察位【不查可达性】（离桌 1.15 m，查了会把所有候选拒光 ✗）
            target, obstacles, reason = self.choose_target(targets, exclude=tried,
                                                           check_reach=False, robot_pose=rp)
            # ★ 只选中"邻桌幻影"时也算"没找到真目标"（2026-09-18）：
            #   幻影几乎一定是"隔着本桌看到邻桌的物体" ⇒ 本桌的真目标只是【没认出来】，
            #   而照幻影去抓必然空合（差一张桌子）✗ ⇒ 先沿法线前进到近看位再确认一次 ✓
            if target is not None:
                m_chk = self._to_map((target.point.x, target.point.y), rp)
                if self._support_rank(m_chk) == 2:
                    self.log.warn("  选中的 {} 落在【邻桌】脚印里 map({:.2f},{:.2f}) ⇒ "
                                  "先前进到近看位再确认（不直接照幻影抓 ✗）"
                                  .format(target.class_id, m_chk[0], m_chk[1]))
                    reason += "（邻桌幻影 → 先近看）"
                    target = None
            # ★ 观察位太远 → 置信度全线掉到阈值以下（实测 0.30~0.47）→ 沿法线前进再看一次
            if (target is None and not skip_nav and not close_looked
                    and self.min_confidence > CLOSE_MIN_CONFIDENCE):
                close_looked = True
                self.log.warn("观察位（含扫视）没有够格的候选（{}）→ 沿同一条法线前进到近看位重看"
                              .format(reason))
                normal = self._ensure_obs_normal()
                if self._goto_close_look(normal):
                    self.sleep(0.5)
                    rp = self._robot_map_pose() or rp
                    prev = self.min_confidence
                    self.min_confidence = CLOSE_MIN_CONFIDENCE
                    try:
                        self.log.info("近看位【原地】扫视（最多 {} 个角度：0°/±25°/±45°，"
                                      "桌子两端才会落到画面中心 ✓）".format(len(SURVEY_YAWS_NEAR)))
                        targets = self.survey_targets(SURVEY_YAWS_NEAR)
                        self.last_targets = targets
                        self.log.info("近看候选 {} 个：{}".format(
                            len(targets),
                            ["{}({:.2f})".format(t.class_id, t.confidence) for t in targets]))
                        target, obstacles, reason = self.choose_target(
                            targets, exclude=tried, check_reach=False, robot_pose=rp)
                    finally:
                        self.min_confidence = prev
                else:
                    self.log.warn("没能前进到近看位（已就地扫视过了）→ 用现有候选继续")
            if target is None:
                self.log.error("没有可夹的目标：{}".format(reason))
                return False
            self.log.info("→ 选中 [{}]：{}".format(target.class_id, reason))

            cy, sy = math.cos(rp[2]), math.sin(rp[2])
            obj_map = (rp[0] + cy * target.point.x - sy * target.point.y,
                       rp[1] + sy * target.point.x + cy * target.point.y)
            # ★ 接近方向 = 【观察那一趟定下的】桌沿法线（不是"物体→机器人"那种含定位
            #   误差的方向，也不是"按目标位置重新算"——那会让邻桌幻影把法线翻到另一边 ✗）
            normal = self._ensure_obs_normal()
            standoff = self.creep_goal_for(obj_map, normal)
            self.log.info("  桌沿法线 = ({:+.3f}, {:+.3f}) → 机械臂正对桌子（yaw = {:.3f}）"
                          .format(normal[0], normal[1], math.atan2(-normal[1], -normal[0])))

            # ★ 目标与障碍一律记【map 坐标】：底盘之后还会动（导航+creep），
            #   观察位量到的 base 系点到了站位就是错的 ✗
            #   （实测：站位上视觉没认出 sugar_box → 沿用了观察位那次 base(1.184,0.249)
            #    的相对量 → 手被指到 1.18 m 外，根本不是物体在的地方）
            # 用 map 记账（不是 odom ✗）：_track/_final_map 全程存 map 坐标
            #   （见 _remember / measure），这里用 odom 会让后面 _to_base 换算错一个 AMCL 偏移
            rp_od = self._robot_map_pose() or rp
            self._track[target.class_id] = (time.time(),
                                            self._to_map((target.point.x, target.point.y), rp_od))
            self._final_map = {"class": target.class_id, "conf": target.confidence,
                               "xy": self._to_map((target.point.x, target.point.y), rp_od)}
            self._final_obs = [{"class": o.class_id, "conf": o.confidence,
                                "xy": self._to_map((o.point.x, o.point.y), rp_od)}
                               for o in obstacles]

            # 站位：优先用命令行覆盖（调试），否则由目标位置算
            if park_override is not None:
                px, py, pyaw = park_override
                _, _, _, ox, oy = (self.standoff_pose(target, standoff, rp, normal)
                                   or (px, py, pyaw, px, py))
                self.log.info("站位被命令行固定为 ({:.3f}, {:.3f}, yaw={:.3f})（调试）"
                              .format(px, py, pyaw))
            else:
                pose = self.standoff_pose(target, standoff, rp, normal)
                if pose is None:
                    self.log.error("拿不到 map←base_footprint 变换，算不出站位")
                    return False
                px, py, pyaw, ox, oy = pose
            self.log.info("② 抓取站位 = ({:.3f}, {:.3f}, yaw={:.3f})；目标在 map({:.3f}, {:.3f})"
                          .format(px, py, pyaw, ox, oy))
            if not skip_nav and not self.navigate(px, py, pyaw):
                self.log.error("站位不可达 → 结束")
                return False
            self.sleep(1.0)

            # 站位上重测（相机与桌面等高，站位上物体整只可见 ✓）
            self.sleep(0.5)
            targets = self.fetch_targets()
            self.last_targets = targets
            t2, obs2, _ = self.choose_target(targets, exclude=tried, check_reach=True,
                                             robot_pose=self._robot_map_pose())
            if t2 is not None and t2.class_id == target.class_id:
                target, obstacles = t2, obs2
            else:
                self.log.warn("站位上没重新检测到 {} → 沿用观察位那次的量".format(target.class_id))

            if not no_creep:
                self.creep(target.class_id, standoff)

            # ══════════ ④ 抓取点上的【最后一张照片】= 最终抓取点 ══════════
            # （用户在 Gazebo 里看到"夹爪下去碰到盒子顶面"就是这里原来用的点太旧 ✗）
            best, obs, src = self._fresh_target(target.class_id)
            if best is not None:
                # ★ 必须重打时间戳：适配层那条消息带的是【拍照时刻】的 stamp，
                #   而它一帧要 12~35 s（3 帧投票 + 窄词表复核）⇒ 直接透传会被抓取侧
                #   判"目标不新鲜 age=12.9s > 10s"拒收 ✗（实测 stage=1 NO_TARGET）
                #   位置还是那次测量的位置（刚测完，底盘没动），时间戳用"此刻" ✓
                now = self.node.get_clock().now().to_msg()
                best.header.stamp = now
                for o in obs:
                    o.header.stamp = now
                self._remember(target.class_id, best)      # 同时刷新里程计跟踪
                bx, by = best.point.x, best.point.y
                self.log.info("抓取点重测: {} base({:+.3f},{:+.3f}) conf={:.2f}（障碍 {} 个，来源 {}）"
                              .format(best.class_id, bx, by, best.confidence, len(obs), src))
                # 残差偏大 → 按同一套相对控制定量补一次（纯里程计），再拍一张确认
                if (abs(by) > 0.020 or abs(math.hypot(bx, by) - standoff) > 0.030):
                    self.log.warn("  残差偏大（横向 {:+.3f} m / 距离 {:.3f} vs {:.3f}）→ 定量补一次"
                                  .format(by, math.hypot(bx, by), standoff))
                    self._nudge(target.class_id, standoff)
                    self.sleep(0.5)
                    best2, obs2, src2 = self._fresh_target(target.class_id)
                    if best2 is not None:
                        # ★ 同样要重打时间戳（第一次忘了会导致第二次测量被"不新鲜"拒收 ✗）
                        now2 = self.node.get_clock().now().to_msg()
                        best2.header.stamp = now2
                        for o in obs2:
                            o.header.stamp = now2
                        self._remember(target.class_id, best2)
                        best, obs = best2, obs2
                        self.log.info("  补正后重测: {} base({:+.3f},{:+.3f}) conf={:.2f}"
                                      .format(best.class_id, best.point.x, best.point.y,
                                              best.confidence))
                target, obstacles = best, obs
            else:
                # 抓取点上认不出来 → 退回"观察位估计 + 里程计外推"，并明确告警
                self.log.warn("抓取点上认不出 {} → 退回观察位估计 + 里程计外推（精度差，可能碰物体）"
                              .format(target.class_id))
                # ★ fm["xy"] 是 map 坐标（_final_map / _track 的基准）⇒ 必须用 map 位姿换算；
                #   原来优先用 odom 位姿 ⇒ 会整体错一个 AMCL 偏移（宁可拿不到位姿就报错退出）
                rp2 = self._robot_map_pose()
                if rp2 is None:
                    self.log.error("拿不到任何底盘位姿，没法把目标换算到当前底盘系")
                    return False
                fm = getattr(self, "_final_map", None)
                if fm is None:
                    fm = {"class": target.class_id, "conf": target.confidence,
                          "xy": self._to_map((target.point.x, target.point.y), rp2)}
                target = self._mk_target(fm["class"], fm["conf"], self._to_base(fm["xy"], rp2))
                obstacles = [self._mk_target(o["class"], o["conf"], self._to_base(o["xy"], rp2))
                             for o in getattr(self, "_final_obs", [])]

            # ══════════ ④ 抓取：接触闭环（空合就重测 + 换偏置重抓）══════════
            ok, stage, msg, tried_offsets = self._grasp_with_offsets(target, obstacles, standoff)
            if ok:
                self.log.info("Phase 2 完成：抓取成功 ✓（判据 = 两指真实间距，见上面的"
                              "「★ 夹到没有」那一行 ✓）")
                # ★ 2026-09-19：顺手量一次"放回误差"（用户反馈"后面几次越来越不准"）
                #   MTC 会在同一次调用里【原地放下】⇒ 再测一次物体位置，与抓取点一比就知道
                #   放回准不准、物体有没有被挪动/转倒 ✓（下一次抓取精度的直接来源 ✓）
                if PLACE_CHECK:
                    self.sleep(0.5)
                    back, _o, _s = self._fresh_target(target.class_id, match_radius=0.60)
                    if back is not None:
                        dx = back.point.x - target.point.x
                        dy = back.point.y - target.point.y
                        d = math.hypot(dx, dy)
                        (self.log.warn if d > 0.020 else self.log.info)(
                            "  ★ 放回核对: 抓取点 base({:+.3f},{:+.3f}) → 放回后测得 "
                            "base({:+.3f},{:+.3f}) ⇒ 位移 (Δx={:+.0f}, Δy={:+.0f}) mm（|d|={:.0f} mm）{}"
                            .format(target.point.x, target.point.y, back.point.x, back.point.y,
                                    dx * 1000, dy * 1000, d * 1000,
                                    "   ⚠ 放回偏了（>20mm）⇒ 再抓同一个物体前先重新观察 ✓"
                                    if d > 0.020 else "   ✓ 放回基本原位"))
                    else:
                        self.log.info("  ★ 放回核对: 这次没认出 {}（可能被夹爪挡住）→ 跳过".format(
                            target.class_id))
                return True
            if stage in (2, 5):          # BAD_TARGET / NO_SOLUTION → 换一个目标再试
                tried.add(target.class_id)
                self.log.warn("目标 {} 失败（stage={}），换下一个目标重试（已试 {}）"
                              .format(target.class_id, STAGE_TEXT.get(stage, stage), tried))
                continue
            if stage == 0:
                # ★ stage=0 但每一档都没夹到 ⇒ 位置估计不可信。
                #   但**不能就此放弃这个类别**：现场实证"第一次抓取之后物体发生了位移"，
                #   重测一轮（重新观察 + 重新站位 + 重新测点）往往就能成 ✓
                #   （只有重试用完、或失败是规划型（stage 2/5）才把它列入 tried ✗）
                pos_rounds += 1
                if pos_rounds <= MAX_POSITION_ROUNDS:
                    self.log.warn("目标 {} 的 {} 档偏置都没夹到 ⇒ 重测一轮（第 {}/{} 轮）后"
                                  "再试同一个目标（物体可能被上一次推走了 ✓）".format(
                                      target.class_id, len(tried_offsets), pos_rounds,
                                      MAX_POSITION_ROUNDS))
                    continue
                tried.add(target.class_id)
                self.log.error("目标 {} 位置型重试用完（{} 轮 × {} 档）⇒ 换目标".format(
                    target.class_id, MAX_POSITION_ROUNDS, len(tried_offsets)))
                continue
            self.log.error("抓取失败（stage={} [{}]）：不再重试".format(
                stage, STAGE_TEXT.get(stage, "未知")))
            return False
        self.log.error("换目标/重测重试 {:.0f} 轮仍未成功 → 结束抓取阶段（已试 {}）".format(
            MAX_TARGET_TRIES + MAX_POSITION_ROUNDS, tried))
        return False
