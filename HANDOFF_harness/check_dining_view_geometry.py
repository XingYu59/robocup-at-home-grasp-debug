#!/usr/bin/env python3
"""观察位 / 近看位 / 站位几何自检（**离线，不用起仿真**）—— 2026-09-18。

为什么要有它（用户现场现象）：
    "导航到餐桌观察点之后，**总是在第一次观察之后导航到餐桌的另一边并且撞桌子**"
本脚本把这条现象拆成可以用真实导航地图直接验算的几个判据，
改 grasp_phase.py 的几何常量后**先跑它**，通过了再去起仿真 ✓（省一轮 5 分钟的复现）

它检查的 6 件事（全部对照 `turtlebot3_manipulation_navigation2/map/map.pgm` 与
`param/turtlebot3.yaml: robot_radius` 的真实数值，不猜）：
    ① 观察位：四条桌沿都站得下、且离障碍 ≥ robot_radius（Nav2 能合法到达）
    ② 近看位：**四个方向到桌沿的距离一致**（旧实现按"离桌心"算 ⇒ 东西侧只剩 0.08 m ✗）
       且近看位本身也在自由区（离障碍 ≥ robot_radius）
    ③ 站位（导航目标）：对桌上若干目标位置解算出来的停车点，全部满足
       free(robot_radius) —— 否则 DWB 判 inscribed(253) = 非法轨迹 ⇒ 车到不了还顶桌子 ✗
    ④ 站位永远在【观察那一侧】（不会因为目标位置换边 ⇒ 不会绕到桌子另一边）
    ⑤ 目标选择：邻桌幻影（table_2 上的 gelatin_box 被误标成 potable 类）**不能**压过
       本桌上真能夹的目标（_support_rank 打分）
    ⑥ 视野覆盖账：观察位 / 近看位 + SURVEY_YAWS 转角的并集，是否覆盖整条桌宽

用法：
    cd ~/Robocup@home_ws && source install/setup.bash
    python3 src/HANDOFF_harness/check_dining_view_geometry.py
退出码 0 = 全过；1 = 有 ✗（逐条打印原因）
"""

import math
import sys
import types

sys.path.insert(0, "/home/xing/Robocup@home_ws/src/turtlebot3_manipulation_navigation2/scripts")

import grasp_phase as GP          # noqa: E402
from grasp_phase import GraspPhase  # noqa: E402

FAIL = []


def check(name, ok, detail=""):
    print("  {} {}{}".format("✓" if ok else "✗", name, ("  ← " + detail) if detail else ""))
    if not ok:
        FAIL.append(name)


class _Log:
    def info(self, m): pass
    def warn(self, m): print("      [warn] " + m)
    def error(self, m): print("      [err ] " + m)


class Stub:
    """只借用 GraspPhase 的纯函数（不建 ROS 节点、不连服务）✓"""
    _to_map = staticmethod(GraspPhase._to_map)
    _to_base = staticmethod(GraspPhase._to_base)
    _support_rank = staticmethod(GraspPhase._support_rank)
    _on_support = staticmethod(GraspPhase._on_support)
    _in_neighbor_table = staticmethod(GraspPhase._in_neighbor_table)
    table_edge_dist = staticmethod(GraspPhase.table_edge_dist)
    _table_exit_distance = GraspPhase._table_exit_distance
    _face_ok = GraspPhase._face_ok
    approach_normal = GraspPhase.approach_normal
    choose_target = GraspPhase.choose_target
    standoff_pose = GraspPhase.standoff_pose
    creep_goal_for = GraspPhase.creep_goal_for
    _free = GraspPhase._free
    close_look_pose = GraspPhase.close_look_pose

    def __init__(self, nav_map, graspable):
        self.log = _Log()
        self.nav_map = nav_map
        self.graspable = graspable
        self.min_confidence = GP.MIN_CONFIDENCE
        self._robot_map_pose = lambda: self.rp
        self.rp = (0.0, 0.0, 0.0)


def mk_target(cls, conf, xy_base):
    t = GP.GraspTargetStamped()
    t.header.frame_id = "base_footprint"
    t.class_id = cls
    t.confidence = float(conf)
    t.point.x, t.point.y, t.point.z = float(xy_base[0]), float(xy_base[1]), GP.TABLE_TOP_Z
    return t


def main():
    nav = GP.NavMap(GP.find_nav_map())
    R = GP.NAV_ROBOT_RADIUS
    graspable = GP.load_graspable()
    gp = Stub(nav, graspable)
    cx, cy = GP.TABLE_CENTER_XYZ[0], GP.TABLE_CENTER_XYZ[1]
    print("地图 {}  机器人半径(Nav2) {:.2f} m  站位外推余量 EDGE_CLEARANCE {:.2f} m".format(
        GP.find_nav_map(), R, GP.EDGE_CLEARANCE))
    print("餐桌3 桌心 ({:.2f},{:.2f})  脚印 x±{:.2f} / y±{:.2f}  → x∈[{:.2f},{:.2f}] y∈[{:.2f},{:.2f}]"
          .format(cx, cy, GP.TABLE_HALF_X, GP.TABLE_HALF_Y,
                  cx - GP.TABLE_HALF_X, cx + GP.TABLE_HALF_X,
                  cy - GP.TABLE_HALF_Y, cy + GP.TABLE_HALF_Y))

    # ── ① / ② 观察位与近看位：四个方向 ───────────────────────────────
    # ★ 注意：南/西两侧本来就被邻桌占住（table_2 在南、table_1 在西），
    #   所以这里不是"四向都必须空"，而是：
    #     · 能站人的那一侧（北/东，= approach_normal 会选的那侧）必须真的站得下、且近看位安全；
    #     · 站不下人的那一侧必须被 _face_ok 拒掉（绝不能选它 → 否则就是"绕到桌子另一边"✗）
    free_sides, blocked_sides = [], []
    for name, n in (("北(0,+1)", (0, 1)), ("东(+1,0)", (1, 0)),
                    ("南(0,-1)", (0, -1)), ("西(-1,0)", (-1, 0))):
        px = cx + n[0] * GP.OBSERVATION_DIST
        py = cy + n[1] * GP.OBSERVATION_DIST
        (free_sides if gp._face_ok(n, (cx, cy)) else blocked_sides).append((name, n, px, py))

    print("\n① 观察位（离桌心 {:.2f} m）：能站人的一侧 = {}".format(
        GP.OBSERVATION_DIST, "、".join(s[0] for s in free_sides)))
    check("正好有 1~2 侧能站人（北/东；南/西被邻桌占住）",
          1 <= len(free_sides) <= 2, "free={} blocked={}".format(
              [s[0] for s in free_sides], [s[0] for s in blocked_sides]))
    for name, n, px, py in free_sides:
        check("{} 观察位 ({:.2f},{:.2f}) 离障碍 ≥ robot_radius".format(name, px, py),
              nav.free(px, py, R), "free({:.2f})={}".format(R, nav.free(px, py, R)))
    for name, n, px, py in blocked_sides:
        check("{} 被 _face_ok 正确拒掉（不会被选成接近侧）".format(name), True)

    print("\n② 近看位：到【桌沿】的距离四个方向必须一致（= {:.2f} m）".format(GP.CLOSE_LOOK_EDGE))
    for name, n in (("北(0,+1)", (0, 1)), ("东(+1,0)", (1, 0)),
                    ("南(0,-1)", (0, -1)), ("西(-1,0)", (-1, 0))):
        lx, ly, lyaw, dist = gp.close_look_pose(n)
        edge_gap = dist - gp.table_edge_dist(n)
        need_free = any(s[1] == n for s in free_sides)
        ok = abs(edge_gap - GP.CLOSE_LOOK_EDGE) < 1e-9
        if need_free:
            ok = ok and nav.free(lx, ly, R)
        check("{} ({:.2f},{:.2f}) 桌沿外 {:.3f} m / 离桌心 {:.3f} m{}".format(
            name, lx, ly, edge_gap, dist, "（且自由区 ✓）" if need_free else "（该侧不可用，只验算式）"),
            ok, "free({:.2f})={}".format(R, nav.free(lx, ly, R)))
    # 旧实现的对照（写死，作为"这个坑真实存在"的回归证据）
    old_bad = []
    for name, n in (("东", (1, 0)), ("西", (-1, 0))):
        ox = cx + n[0] * 0.68
        oy = cy + n[1] * 0.68
        gap = 0.68 - (abs(n[0]) * GP.TABLE_HALF_X + abs(n[1]) * GP.TABLE_HALF_Y)
        if gap < R:
            old_bad.append("{}侧只离桌沿 {:.3f} m".format(name, gap))
    print("   · 回归对照：旧实现「离桌心 0.68 m」→ " + ("；".join(old_bad) if old_bad
                                                        else "四向都安全"))
    check("旧实现确实在东西侧不安全（说明本修复有必要）", bool(old_bad))

    # ── ③ / ④ 站位（导航目标点）────────────────────────────────────
    print("\n③ 站位（导航目标）必须满足 free(robot_radius) —— 否则 DWB 判非法轨迹 ✗")
    # 桌面上的典型目标（世界文件里的真实位置）+ 一个"桌里侧"的目标
    objs = [("tomato_soup_can", 2.30, 2.00), ("bowl", 2.6716, 2.015),
            ("sugar_box", 3.10, 2.00), ("桌里侧目标", 2.70, 2.00)]
    for normal_name, n in (("北", (0, 1)), ("东", (1, 0))):
        for cls, ox, oy in objs:
            # 机器人先站在该法线上的观察位 → 目标与观察位同侧才可能被选中
            gp.rp = (cx + n[0] * GP.OBSERVATION_DIST, cy + n[1] * GP.OBSERVATION_DIST, 0.0)
            # 把 map 目标点换算成"当前底盘系"下的点（standoff_pose 的输入形式）
            gx, gy = ox - gp.rp[0], oy - gp.rp[1]
            t = mk_target(cls, 0.8, (gx, gy))
            st = gp.creep_goal_for((ox, oy), n)
            pose = gp.standoff_pose(t, st, gp.rp, n)
            if pose is None:
                check("{}/{} 站位解算".format(normal_name, cls), False, "None")
                continue
            px, py, pyaw, tox, toy = pose
            d_edge = max(abs(px - cx) - GP.TABLE_HALF_X, abs(py - cy) - GP.TABLE_HALF_Y)
            ok_free = nav.free(px, py, R)
            on_side = ((px - ox) * n[0] + (py - oy) * n[1]) > 0
            check("{}侧 {} → 站位({:.2f},{:.2f}) 桌沿外 {:.3f} m free({:.2f})={} 同侧={}".format(
                normal_name, cls, px, py, d_edge, R, ok_free, on_side),
                ok_free and on_side and d_edge >= GP.EDGE_CLEARANCE - 0.02)

    # ── ③b 对照：旧余量 0.26 会落在 inscribed 区 ────────────────────
    old_goal = (cx, cy + GP.TABLE_HALF_Y + 0.26)
    print("\n   · 回归对照：旧 EDGE_CLEARANCE=0.26 的站位 ({:.2f},{:.2f}) free({:.2f})={}"
          .format(old_goal[0], old_goal[1], R, nav.free(old_goal[0], old_goal[1], R)))
    check("旧站位确实不满足 Nav2 可达性（说明本修复有必要）",
          not nav.free(old_goal[0], old_goal[1], R))

    # ── ⑤ 幻影目标不能劫持选目标 ────────────────────────────────────
    print("\n⑤ 选目标：邻桌幻影 vs 本桌真目标（_support_rank 打分）")
    gp.rp = (cx, cy + GP.OBSERVATION_DIST, 0.0)
    real = mk_target("tomato_soup_can", 0.36, (2.30 - gp.rp[0], 2.00 - gp.rp[1]))
    # L1/L3 现场实例：gelatin_box(2.7,1.5) 被闭集复核误标成 potted_meat_can，map(2.951,1.569)
    phantom = mk_target("potted_meat_can", 0.55, (2.951 - gp.rp[0], 1.569 - gp.rp[1]))
    best, obs, why = gp.choose_target([phantom, real], check_reach=False, robot_pose=gp.rp)
    check("本桌真目标（conf 0.36）胜过邻桌幻影（conf 0.55）",
          best is not None and best.class_id == "tomato_soup_can", why)
    check("幻影被标成「邻桌脚印内」（档 2）", GraspPhase._support_rank((2.951, 1.569)) == 2,
          "rank={}".format(GraspPhase._support_rank((2.951, 1.569))))
    check("本桌目标被标成「本桌」（档 0）", GraspPhase._support_rank((2.30, 2.00)) == 0)
    best2, _, why2 = gp.choose_target([phantom], check_reach=False, robot_pose=gp.rp)
    check("只剩幻影时**不放弃整轮**（仍返回候选，只打 WARN）", best2 is not None, why2)

    # ── ⑥ 视野覆盖账 ────────────────────────────────────────────────
    print("\n⑥ 视野覆盖：单视场 62°（fx=530.47/cx=320 → 半角 31.1°）+ SURVEY_YAWS 并集")
    half_fov = math.degrees(math.atan2(320.0, 530.47))
    print("     半角 {:.1f}°；SURVEY_YAWS = {}（{:+.0f}° … {:+.0f}°）"
          .format(half_fov, GP.SURVEY_YAWS, math.degrees(min(GP.SURVEY_YAWS)),
                  math.degrees(max(GP.SURVEY_YAWS))))
    for tag, dist_center in (("观察位", GP.OBSERVATION_DIST),
                             ("近看位", GP.TABLE_HALF_Y + GP.CLOSE_LOOK_EDGE)):
        # 桌上物体在 y=2.0 那一排（x 从 2.1 到 3.3），相机站在北侧法线上
        cam_y = cy + dist_center
        need = max(abs(math.degrees(math.atan2(x - cx, cam_y - 2.0))) for x in (2.10, 3.30))
        cover = half_fov + math.degrees(max(abs(y) for y in GP.SURVEY_YAWS))
        check("{}（离桌心 {:.2f} m）：桌子两端需 ±{:.1f}°，并集覆盖 ±{:.1f}°".format(
            tag, dist_center, need, cover), cover >= need)

    # ── ⑦ 桌子上的物体覆盖账（2026-09-19 新增：用户报"最边上那个看不到"）──────
    print("\n⑦ 桌上每个物体在【观察位 / 近看位 × 扫视角度】下的偏轴角"
          "（>半角 {:.1f}° 就看不见；越接近 0° 检测/分割越好）".format(half_fov))
    # 餐桌 3 当前 4 个测试物体（world: example.world，一排 y=2.15）
    # ★ 2026-09-19 第二版：potted_meat_can 挪到**桌子正中 2.70**（原来在最西端 2.30 ⇒
    #   观察位偏轴 21°、近看位 38°，只有扫到 ±25°/±45° 才进画面中心，而它只有 40~70 px
    #   ⇒ 检出/命名逐帧碰运气 ✗）。间距 0.30 → 0.25 m 容纳这个位置 ✓
    objects = [("tomato_soup_can", 2.20, 2.15), ("coke can", 2.45, 2.15),
               ("potted_meat_can", 2.70, 2.15), ("cracker_box", 2.95, 2.15)]
    n = (0.0, 1.0)                      # 北侧观察（本场景唯一站得下人的一侧）
    for tag, dist_center, yaw_set in (
            ("观察位", GP.OBSERVATION_DIST, GP.SURVEY_YAWS),
            ("近看位", GP.TABLE_HALF_Y + GP.CLOSE_LOOK_EDGE,
             getattr(GP, "SURVEY_YAWS_NEAR", GP.SURVEY_YAWS))):
        yaws = [math.degrees(a) for a in yaw_set]
        cam = (cx + n[0] * dist_center, cy + n[1] * dist_center)
        rows = []
        for name, ox, oy in objects:
            # 机器人朝桌心（yaw 使 base x 指桌内）；目标相对相机轴的偏轴角（度）
            base_ang = math.degrees(math.atan2(ox - cam[0], cam[1] - oy))
            best = min(abs(base_ang - y) for y in yaws)
            vis = min(abs(base_ang - y) for y in yaws) <= half_fov
            rows.append((name, base_ang, best, vis))
        for name, a, b, vis in rows:
            check("{} {} 偏轴 {:+.1f}° → 扫视后最佳 {:.1f}°（{}）".format(
                tag, name, a, b, "看得见 ✓" if vis else "看不见 ✗"), vis)
        worst = max(r[2] for r in rows)
        check("{}：4 个物体在扫视后都落在画面中心 ±15° 内（最差 {:.1f}°）".format(tag, worst),
              worst <= 15.0,
              "最差 {:.1f}°；只拍 0° 时最差 {:.1f}°".format(
                  worst, max(abs(r[1]) for r in rows)))
    check("默认扫满所有角度（SURVEY_ALL_ANGLES）—— 否则靠边物体只在某个角度才居中被漏掉 ✗",
          bool(getattr(GP, "SURVEY_ALL_ANGLES", False)))
    # ★ 2026-09-19：本场景的【待验证目标】= potted_meat_can，它必须在 0° 就基本居中
    #   （挪到桌子正中 2.70 的目的就是这个：不再靠扫视角度"碰运气"把它扫进画面中心）
    cam0 = (cx, cy + GP.OBSERVATION_DIST)
    a0 = abs(math.degrees(math.atan2(2.70 - cam0[0], cam0[1] - 2.15)))
    check("待验证目标 potted_meat_can 在观察位 0° 就居中（偏轴 {:.1f}° ≤ 5°）".format(a0),
          a0 <= 5.0, a0)
    cam1 = (cx, cy + GP.TABLE_HALF_Y + GP.CLOSE_LOOK_EDGE)
    a1 = abs(math.degrees(math.atan2(2.70 - cam1[0], cam1[1] - 2.15)))
    check("近看位 0° 也居中（偏轴 {:.1f}° ≤ 5°）".format(a1), a1 <= 5.0, a1)

    print("\n" + ("全部通过 ✓" if not FAIL else "有 {} 条不过 ✗：{}".format(len(FAIL), FAIL)))
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
