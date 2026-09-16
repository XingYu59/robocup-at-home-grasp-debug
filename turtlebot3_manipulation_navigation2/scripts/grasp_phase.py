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
from sensor_msgs.msg import JointState
from nav2_msgs.action import NavigateToPose

from turtlebot3_manipulation_grasp.msg import GraspTargetStamped
from turtlebot3_manipulation_grasp.srv import DetectGraspTarget, GraspFixedObject

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
CLOSE_LOOK_DIST = 0.68      # m，近看位到桌心的距离（≈ 桌沿外 0.43 m、离物体 0.7~0.9 m）
# ★ 近看位上的【多角度扫视】：单个视场 62° 覆盖不了 1.2 m 长的桌子 ✗
#   （实测：观察位处桌子两端 x=2.10/3.30 落在视野外；而罐子在 1.14 m 只有 29×45 像素、
#     检测器完全认不出 ✗）→ 走到离桌心 ~0.68 m 后原地转几个角度，每个角度拍一次，
#   再把结果按 map 坐标合并（同一物体去重）✓
SURVEY_YAWS = (-0.44, 0.0, 0.44)     # rad，±25°；转完并集覆盖约 ±56° ✓
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
EDGE_CLEARANCE = 0.26       # m，车心到桌沿的最小距离（车半径 + 余量）
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
MAX_TARGET_TRIES = 2            # 抓失败换目标的次数（规则书：四个里任选一个）


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
ROBOT_CLEAR_PARK = 0.10     # m，站位只要求"别压在桌子上/墙里"（贴桌沿停车是常态）


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
        # ★ 手指关节实测位置：判断"到底有没有夹到物体"的唯一直接证据
        #   （仿真里合爪是位置控制：夹到物体就会停在物体宽度处、到不了指令值 ✓）
        self._finger_q = None
        self.object_sizes = load_object_sizes()
        self.close_squeeze = load_close_squeeze()
        # ★ 注意：GraspPhase 不是 Node（只是拿着调用方的 node）→ 必须用 self.node.* ✗
        self.node.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self._track = {}             # 类别 → (时间, odom 坐标)：creep 的里程计跟踪基准
        # 能夹什么以 objects.yaml 为准（读不到才退回代码里的兜底名单）
        self.graspable = load_graspable()
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
        for name, pos in zip(msg.name, msg.position):
            if name == "fr3_finger_joint1":
                self._finger_q = float(pos)
                break

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
        """两指间距（mm）= 2 × 关节值；读不到返回 None。"""
        return None if self._finger_q is None else self._finger_q * 2000.0

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
        req.class_ids = self.target_classes or []
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

    # ══════════════ ② 选目标（硬过滤 + 档位 + 并列时排序）══════════════
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
        """
        if TABLE_IS_DINING:                       # 只在餐厅那三张并排桌子时启用 ✓
            for cx, cy in NEIGHBOR_TABLES:        # 邻居桌子脚印内 → 不是本桌的目标
                if (abs(p_map[0] - cx) <= NEIGHBOR_HALF[0] - 0.05
                        and abs(p_map[1] - cy) <= NEIGHBOR_HALF[1] - 0.05):
                    return False
        return (abs(p_map[0] - TABLE_CENTER_XYZ[0]) <= TABLE_HALF_X + margin
                and abs(p_map[1] - TABLE_CENTER_XYZ[1]) <= TABLE_HALF_Y + margin)

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
            # 必须在支撑面（餐桌）上：见 _on_support 的说明
            if on_support_only and robot_pose is not None:
                m = self._to_map((t.point.x, t.point.y), robot_pose)
                if not self._on_support(m):
                    self.log.info("  跳过 {}：不在支撑面（餐桌）上 map({:.3f},{:.3f}) ✗"
                                  .format(cls, m[0], m[1]))
                    continue
            r = math.hypot(t.point.x - ARM_BASE_X, t.point.y)
            if check_reach and not (REACH_MIN <= r <= REACH_MAX):
                self.log.info("  跳过 {}：距臂基座 {:.3f} m 不在 [{:.2f}, {:.2f}]".format(
                    cls, r, REACH_MIN, REACH_MAX))
                continue
            tier = next((k for k, row in enumerate(TARGET_TIERS) if cls in row), len(TARGET_TIERS))
            # 并列时的排序键：离机器人近（x 小）→ 与邻居间隙大 → 置信度高
            gap = min([math.hypot(t.point.x - o.point.x, t.point.y - o.point.y)
                       for j, o in enumerate(targets) if j != i] or [9.9])
            cands.append((tier, t.point.x, -gap, -t.confidence, t, i))
        if not cands:
            return None, [], "没有可夹且可达的目标"
        cands.sort(key=lambda c: c[:4])
        _, _, neg_gap, _, best, bi = cands[0]
        obstacles = [t for j, t in enumerate(targets) if j != bi]
        reason = "\"{}\" / 距臂基座 {:.3f} m / 与邻居最近 {:.3f} m / conf {:.2f}".format(
            best.class_id, math.hypot(best.point.x - ARM_BASE_X, best.point.y),
            -neg_gap, best.confidence)
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

        ★ 接近方向 = 桌沿法线（approach_normal），不是"机器人当前方位"：
          站位 = 物体 + g × 法线，yaw = 法线反方向 → 车头垂直桌沿、物体在正前方 ✓
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
        # 容差取 1e-6：正好等于 EDGE_CLEARANCE 边界时算【外面】（那本来就是留的余量），
        # 这样旧工作区验证过的 (3.0, 2.51) 不会被推到 2.7 ✗
        hx = TABLE_HALF_X + EDGE_CLEARANCE
        hy = TABLE_HALF_Y + EDGE_CLEARANCE

        def inside_table(x, y):
            return (abs(x - TABLE_CENTER_XYZ[0]) < hx - 1e-6
                    and abs(y - TABLE_CENTER_XYZ[1]) < hy - 1e-6)

        t_off = standoff if standoff else STANDOFF_DIST
        while t_off < 1.0:
            x, y = ox + ax * t_off, oy + ay * t_off
            if not inside_table(x, y) and self._free(x, y, ROBOT_CLEAR_PARK):
                break
            t_off += 0.01
        if t_off > (standoff if standoff else STANDOFF_DIST):
            self.log.info("  站位被桌子/障碍挡住 → 沿桌沿法线外推到 {:.3f} m".format(t_off))
        px, py = ox + ax * t_off, oy + ay * t_off
        yaw = math.atan2(oy - py, ox - px)                             # 面朝物体（= 沿法线指向桌内）
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
            if rp_map is not None:
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
                best, best_d = None, 0.25
                for t in tg:
                    q = self._to_map((t.point.x, t.point.y), rp_map)
                    d = math.hypot(q[0] - tr[1][0], q[1] - tr[1][1])
                    if d < best_d:
                        best, best_d = t, d
                if best is not None:
                    self.log.warn("  这次没认出 \"{}\" → 用 {} 的位置代替（相距 {:.3f} m，"
                                  "开集标签会逐帧变，位置才是微调要的量）"
                                  .format(class_id, best.class_id, best_d))
                    self._remember(class_id, best)
                    return (best.point.x, best.point.y)
        return None

    def _remember(self, class_id, tgt):
        """把某类别的当前位置记成 odom 坐标（creep 的跟踪基准）。"""
        rp_map = self._robot_map_pose()          # ★ 同上：轨迹存 map 坐标，就得用 map 位姿
        if rp_map is None:
            return
        self._track[class_id] = (time.time(), self._to_map((tgt.point.x, tgt.point.y), rp_map))

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

    def survey_targets(self, yaws=SURVEY_YAWS):
        """在当前站位【转几个角度】各检测一次，按 map 坐标合并去重 ✓

        为什么要它（实测）：观察位离桌心 1.15 m 时，水平视野 31°（半角）覆盖不到桌子两端 ✗；
        而小物体（番茄罐 66×101 mm）在 1 m 外只有 ~30 像素、检测器直接认不出 ✗
        → 走到近看位（离桌心 0.68 m）再转 ±25° 扫三遍，并集覆盖 ±56°、物体 0.6~0.8 m ✓
        合并规则：同类且 map 坐标相距 < SURVEY_MERGE_DIST ⇒ 同一个物体，保留置信度高的那次 ✓
        返回：换算到**当前**底盘系的 GraspTargetStamped 列表（可直接喂给 choose_target）✓
        """
        best = {}       # (class, 约化后的 map 位置) → (conf, map_xy, class)
        rp_od = self._robot_map_pose()       # ★ 下面算的是 map 坐标 ⇒ 必须用 map 位姿
        for i, dyaw in enumerate(yaws):
            if i > 0:
                ok = self._rotate_by(dyaw - yaws[i - 1])
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
        # 转回原朝向（对齐前先归位，免得站位解算用到歪掉的朝向）
        if self._rotate_by(-yaws[-1]) is False:
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

    def _fresh_target(self, class_id, match_radius=0.35):
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
                    self.log.warn("  抓取点上没认出 \"{}\" → 用 {} 的检测代替（相距 {:.3f} m）"
                                  .format(class_id, best.class_id, bd))
                    same = [best]
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

    def call_grasp(self, target, obstacles):
        if not self.wait_client(self.grasp_client, 30.0):
            self.log.error("抓取服务 {} 不可用（grasp_service.launch.py 起了吗？）"
                           .format(GRASP_SERVICE))
            return 0, 1, "grasp service unavailable"
        req = GraspFixedObject.Request()
        req.target = target
        req.obstacles = obstacles
        self.log.info("调用 {} : target={} 轴心点({:.3f}, {:.3f}, {:.3f}) 障碍 {} 个".format(
            GRASP_SERVICE, target.class_id, target.point.x, target.point.y, target.point.z,
            len(obstacles)))
        q0 = self._finger_q
        span = None
        sz = self.object_sizes.get(target.class_id)
        if sz:
            span = min(sz[0], sz[1])
            squeeze = self.close_squeeze          # 与 pick_and_place.cpp 同源（都读配置）
            cmd = 0.5 * span - squeeze
            self.log.info("  预期合爪: 物体窄边 {:.4f} m − 干涉 {:.4f} → 关节 {:.4f}"
                          "（两指间距 {:.1f} mm）".format(span, squeeze, cmd, cmd * 2000))
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
        gmin, gmax = gap0, gap0
        deadline = time.time() + GRASP_SERVICE_TIMEOUT
        while rclpy.ok() and not fut.done() and time.time() < deadline:
            q = self._finger_q
            if q is not None:
                qmin = q if qmin is None else min(qmin, q)
                qmax = q if qmax is None else max(qmax, q)
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
            return 0, -1, "timeout"
        # ★ 手指关节实测行程 → 直接判定"夹到了没有"
        if qmin is not None and qmax is not None and abs(qmax - qmin) > 1e-4:
            gap = qmin * 2000.0
            self.log.info("  合爪实测: 关节 起始 {:.4f} → 最终 {:.4f}（两指间距 {:.1f} mm）".format(
                qmax, qmin, gap))
            if span:
                if qmin * 2.0 > span + 0.004:
                    self.log.warn("  ✗ 手指停在 {:.1f} mm —— 比物体窄边 {:.1f} mm 还宽 "
                                  "→ 没夹到物体（或夹到了别的东西）"
                                  .format(gap, span * 1000))
                elif qmin * 2.0 > span - 0.002:
                    self.log.info("  ✓ 手指停在 {:.1f} mm ≈ 物体窄边 {:.1f} mm "
                                  "→ 夹到了物体（接触即停）".format(gap, span * 1000))
                else:
                    self.log.warn("  ? 手指合到 {:.1f} mm，比物体窄边 {:.1f} mm 还小 "
                                  "→ 物体被挤走 / 没在两指之间".format(gap, span * 1000))
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
            return 0, -1, "no response"
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
        return res.success, res.stage, res.message

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
            self.log.info("   观察位 = ({:.3f}, {:.3f}, yaw={:.3f})；桌沿法线 = {}".format(
                observation_pose[0], observation_pose[1], observation_pose[2],
                self.approach_normal(TABLE_CENTER_XYZ[:2],
                                     rp0[:2] if rp0 else (observation_pose[0], observation_pose[1]))))
            if not self.navigate(*observation_pose):
                self.log.warn("观察位不可达 → 直接用站位视觉继续")
            else:
                # ★ 到达后再做一次"对准餐桌中心 + 收到 ~1.15 m"的小闭环：
                #   Nav2 容差 0.4 m 允许停偏 0.3 m（实测偏了 313 mm），桌子会偏出画面 ✗
                self.align_and_approach((TABLE_CENTER_XYZ[0], TABLE_CENTER_XYZ[1]),
                                        stand_dist=OBSERVATION_DIST)
            self.sleep(0.5)

        tried = set()
        for attempt in range(1, MAX_TARGET_TRIES + 1):
            targets = self.fetch_targets()
            self.last_targets = targets
            # ★ 一个都没返回时，也要试一次"近看"（实测踩过）：
            #   小物体（汤罐 66×101 mm）在观察位 1.15 m 处视觉完全认不出来 ✗，
            #   而原来的近看回退只在"有候选但都不合格"时才触发 → 直接结束 ✗
            if not targets and not skip_nav and not tried:
                self.log.warn("观察位没返回任何目标 → 靠近到近看位再试一次")
                rp_c = self._robot_map_pose()
                if rp_c is not None:
                    n = self.approach_normal(TABLE_CENTER_XYZ[:2], rp_c[:2])
                    cx = TABLE_CENTER_XYZ[0] + n[0] * CLOSE_LOOK_DIST
                    cy = TABLE_CENTER_XYZ[1] + n[1] * CLOSE_LOOK_DIST
                    cyaw = math.atan2(TABLE_CENTER_XYZ[1] - cy, TABLE_CENTER_XYZ[0] - cx)
                    if self.navigate(cx, cy, cyaw):
                        self.align_and_approach((TABLE_CENTER_XYZ[0], TABLE_CENTER_XYZ[1]),
                                                stand_dist=CLOSE_LOOK_DIST, timeout=15.0)
                        self.sleep(0.5)
                        self.log.info("近看位扫视（转 {} 个角度，单视场覆盖不到整桌 ✓）"
                                      .format(len(SURVEY_YAWS)))
                        targets = self.survey_targets()
                        self.last_targets = targets
            if not targets:
                self.log.error("视觉没给出任何目标（观察位 + 近看位都没有）→ 结束抓取阶段（不会退真值 ✗）")
                return False
            self.log.info("候选目标 {} 个（来源 vision）：{}".format(
                len(targets),
                ["{}({:.2f})".format(t.class_id, t.confidence) for t in targets]))
            # 选目标前先拿机器人的 map 位姿：①支撑面校验 ②站位解算 都要用
            # ★ 算站位前等它稳定（启动/重定位时 AMCL 会让它跳，见 _wait_map_pose_stable）
            rp = self._wait_map_pose_stable()
            if rp is None:
                self.log.error("拿不到 map←base_footprint 变换，算不出站位")
                return False
            # 观察位【不查可达性】（离桌 1.15 m，查了会把所有候选拒光 ✗）
            target, obstacles, reason = self.choose_target(targets, exclude=tried,
                                                           check_reach=False, robot_pose=rp)
            # ★ 观察位太远 → 置信度全线掉到阈值以下（实测 0.30~0.47）→ 再靠近看一次
            if (target is None and not skip_nav and not tried
                    and self.min_confidence > CLOSE_MIN_CONFIDENCE):
                self.log.warn("观察位没有够格的候选（{}）→ 靠近到近看位重看一次".format(reason))
                n = self.approach_normal(TABLE_CENTER_XYZ[:2], rp[:2])
                cx = TABLE_CENTER_XYZ[0] + n[0] * CLOSE_LOOK_DIST
                cy = TABLE_CENTER_XYZ[1] + n[1] * CLOSE_LOOK_DIST
                cyaw = math.atan2(TABLE_CENTER_XYZ[1] - cy, TABLE_CENTER_XYZ[0] - cx)
                if self.navigate(cx, cy, cyaw):
                    self.align_and_approach((TABLE_CENTER_XYZ[0], TABLE_CENTER_XYZ[1]),
                                            stand_dist=CLOSE_LOOK_DIST, timeout=15.0)
                    self.sleep(0.5)
                    rp = self._robot_map_pose() or rp
                    prev = self.min_confidence
                    self.min_confidence = CLOSE_MIN_CONFIDENCE
                    try:
                        self.log.info("近看位扫视（转 {} 个角度）".format(len(SURVEY_YAWS)))
                        targets = self.survey_targets()
                        self.last_targets = targets
                        self.log.info("近看候选 {} 个：{}".format(
                            len(targets),
                            ["{}({:.2f})".format(t.class_id, t.confidence) for t in targets]))
                        target, obstacles, reason = self.choose_target(
                            targets, exclude=tried, check_reach=False, robot_pose=rp)
                    finally:
                        self.min_confidence = prev
                else:
                    self.log.warn("近看位不可达")
            if target is None:
                self.log.error("没有可夹的目标：{}".format(reason))
                return False
            self.log.info("→ 选中 [{}]：{}".format(target.class_id, reason))

            cy, sy = math.cos(rp[2]), math.sin(rp[2])
            obj_map = (rp[0] + cy * target.point.x - sy * target.point.y,
                       rp[1] + sy * target.point.x + cy * target.point.y)
            # ★ 接近方向 = 桌沿法线（不是"物体→机器人"那种含定位误差的方向）
            normal = self.approach_normal(obj_map, rp[:2])
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

            ok, stage, msg = self.call_grasp(target, obstacles)
            if ok:
                self.log.info("Phase 2 完成：抓取成功 ✓")
                return True
            if stage in (2, 5):          # BAD_TARGET / NO_SOLUTION → 换一个目标再试
                tried.add(target.class_id)
                self.log.warn("目标 {} 失败（stage={}），换下一个目标重试（已试 {}）"
                              .format(target.class_id, STAGE_TEXT.get(stage, stage), tried))
                continue
            self.log.error("抓取失败（stage={} [{}]）：不再重试".format(
                stage, STAGE_TEXT.get(stage, "未知")))
            return False
        self.log.error("换目标重试 {:.0f} 次仍未成功 → 结束抓取阶段".format(MAX_TARGET_TRIES))
        return False
