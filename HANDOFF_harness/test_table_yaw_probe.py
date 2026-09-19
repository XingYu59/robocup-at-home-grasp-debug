#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线自测：桌腿基准（table_legs_map / legs_from_scan / fit_pose_from_landmarks）
与偏航补偿的注入值（GraspPhase._apply_table_yaw_fix）。

背景（2026-09-19 现场：potted_meat_can"手停在物体顶上、合爪合到指令间距却空合"）：
    C++ 侧合拢轴 = box_yaw = object_yaw_map(常数 0) − robot_yaw_map(TF/AMCL)
    ⇒ 整条朝向链挂在 AMCL 偏航上；AMCL 的偏航误差 e 是**常量偏置**，动车站消不掉
    （车一转，belief 跟着转 ⇒ e 不变）⇒ 必须用**不依赖 AMCL 的基准**量出 e 再补偿。
    本工程选的基准 = 餐桌的 4 条腿（世界文件真值：桌子局部 ±0.235/±0.585，截面 30 mm；
    激光在 0.18 m 高、从桌沿外看得见 ✓）。

本测试用合成激光验证三件事：
  ① 桌腿真值算得对（table_legs_map 与世界文件 dinning_table_3 逐条对上）；
  ② 给定"真位姿 / AMCL belief 有 e 偏置"的合成扫描，反解能还原真位姿（偏航误差 < 0.5°）；
  ③ 注入值必须是 e = belief − truth（符号错了会**反向**把朝向搞坏 ✗），且没量到/太小
     时不能乱写参数（保持原行为）。

用法（不用起仿真、不用 source）：
    python3 HANDOFF_harness/test_table_yaw_probe.py
"""
import importlib.util
import math
import os
import random
import sys
import types

SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                                      "turtlebot3_manipulation_navigation2", "scripts"))
_STUBS = ("rclpy", "rclpy.action", "rclpy.time", "tf2_ros",
          "geometry_msgs", "geometry_msgs.msg", "sensor_msgs", "sensor_msgs.msg",
          "nav2_msgs", "nav2_msgs.action",
          "turtlebot3_manipulation_grasp", "turtlebot3_manipulation_grasp.msg",
          "turtlebot3_manipulation_grasp.srv")


class _Any(types.ModuleType):
    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        child = _Any("{}.{}".format(self.__name__, name))
        setattr(self, name, child)
        return child

    def __call__(self, *a, **k):
        return _Any("stub")


for _n in _STUBS:
    if _n not in sys.modules:
        try:
            __import__(_n)
        except Exception:
            sys.modules[_n] = _Any(_n)

if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)
import grasp_phase as G  # noqa: E402

FAILED = []
NCHECK = [0]


def check(name, cond, detail=""):
    NCHECK[0] += 1
    print("  {} {}".format("✓" if cond else "✗", name) + ("" if cond else "   ← " + str(detail)))
    if not cond:
        FAILED.append(name)


# ══════════════════ 合成世界：dinning_table_3 + 一个"真位姿"的车 ══════════════════
TABLE_C = (2.7, 2.0)
TABLE_YAW = math.pi / 2.0
LEGS = G.table_legs_map(TABLE_C, TABLE_YAW)


def rot(a):
    c, s = math.cos(a), math.sin(a)
    return lambda x, y: (c * x - s * y, s * x + c * y)


def synth_scan(true_pose, *, sigma=0.01, seed=7, drop=(), face=G.LEG_FACE_OFFSET):
    """按"车真位姿"合成一帧激光：每条腿给出"近侧面"上的若干点（含噪声）。

    真实激光打到的是腿的**近侧面**（比腿轴靠近车 ~半个截面 15 mm）⇒ 这里先算腿轴在
    base 里的位置，再沿"车→腿"方向拉近 face 米，并撒 sigma 的径向噪声 ✓
    """
    rnd = random.Random(seed)
    tx, ty, tt = true_pose
    pts = []
    for k, (lx, ly) in enumerate(LEGS):
        if k in drop:
            continue
        dx, dy = lx - tx, ly - ty
        bx, by = rot(-tt)(dx, dy)                 # map→base
        r = math.hypot(bx, by)
        ux, uy = (bx / r, by / r) if r > 1e-6 else (1.0, 0.0)
        for _ in range(4):                        # 一腿 ~4 个采样点（1° 一格、腿张角 ~5°）
            rr = r - face + rnd.gauss(0.0, sigma)
            ang = math.atan2(uy, ux) + rnd.gauss(0.0, math.radians(0.6))
            pts.append((rr * math.cos(ang), rr * math.sin(ang)))
    return pts


print("=" * 72)
print("用例① 桌腿真值（世界文件 dinning_table_3：桌面 0.5×1.2、桌子(2.7,2.0) yaw=90°）")
exp = [(2.115, 1.765), (2.115, 2.235), (3.285, 1.765), (3.285, 2.235)]
for (gx, gy), (ex, ey) in zip(sorted(LEGS), sorted(exp)):
    check("桌腿 ({:+.3f},{:+.3f}) ≈ 期望 ({:+.3f},{:+.3f})".format(gx, gy, ex, ey),
          abs(gx - ex) < 2e-3 and abs(gy - ey) < 2e-3, (gx, gy))
check("桌腿在桌面足迹内（x∈[2.1,3.3], y∈[1.75,2.25]）",
      all(2.10 <= x <= 3.30 and 1.75 <= y <= 2.25 for (x, y) in LEGS), LEGS)

print("\n" + "=" * 72)
print("用例② 合成扫描 → 反解真位姿（AMCL belief 偏航带 e，反解必须还原真值）")
for e_deg, truth_yaw_deg in ((25.7, -90.0), (-12.0, -95.0), (0.0, -90.0)):
    e = math.radians(e_deg)
    truth = (2.19, 2.61, math.radians(truth_yaw_deg))
    belief = (truth[0] + 0.03, truth[1] - 0.02, truth[2] + e)   # AMCL：位置也带 3 cm
    pts = synth_scan(truth)
    got, idx, miss = G.legs_from_scan(pts, LEGS, belief)
    fit = G.fit_pose_from_landmarks(got, [LEGS[k] for k in idx])
    check("e={:+.1f}°：4 条腿都命中".format(e_deg), len(got) == 4 and not miss, (len(got), miss))
    check("e={:+.1f}°：反解位姿有效".format(e_deg), fit is not None, fit)
    if fit:
        fx, fy, fyaw, rms = fit
        dyaw = math.degrees(abs(math.atan2(math.sin(fyaw - truth[2]), math.cos(fyaw - truth[2]))))
        dxy = math.hypot(fx - truth[0], fy - truth[1])
        check("e={:+.1f}°：偏航还原误差 {:.2f}° < 0.5°".format(e_deg, dyaw), dyaw < 0.5, dyaw)
        check("e={:+.1f}°：位置还原误差 {:.0f} mm < 25 mm".format(e_deg, dxy * 1000),
              dxy < 0.025, dxy * 1000)
        check("e={:+.1f}°：拟合残差 {:.1f} mm ≤ 阈值".format(e_deg, rms * 1000),
              rms <= G.YAW_FIT_RMS_MAX, rms * 1000)
        # 真正要的产物：e = belief − truth_recovered
        e_meas = math.atan2(math.sin(belief[2] - fyaw), math.cos(belief[2] - fyaw))
        check("e={:+.1f}°：量出的偏差 e={:+.2f}° ≈ 设定值（±0.5°）".format(
            e_deg, math.degrees(e_meas)), abs(math.degrees(e_meas) - e_deg) < 0.5,
            math.degrees(e_meas))

print("\n" + "=" * 72)
print("用例③ 护栏：腿少/点乱/偏差离谱时不许乱用")
truth = (2.19, 2.61, math.radians(-90.0))
belief = (2.19, 2.61, math.radians(-90.0))
pts3 = synth_scan(truth, drop=(0,))
got3, idx3, miss3 = G.legs_from_scan(pts3, LEGS, belief)
check("只扫到 3 条腿时仍能命中 3 条（>=3 可用 ✓）", len(got3) == 3 and len(miss3) == 1,
      (len(got3), miss3))
fit3 = G.fit_pose_from_landmarks(got3, [LEGS[k] for k in idx3])
check("3 条腿也解得出来", fit3 is not None, fit3)
pts2 = synth_scan(truth, drop=(0, 1))
got2, idx2, miss2 = G.legs_from_scan(pts2, LEGS, belief)
check("只扫到 2 条腿 ⇒ 命中 2 条（调用方 <3 即放弃 ✓）", len(got2) == 2, len(got2))
check("2 点不足以拟合 ⇒ fit_pose_from_landmarks 返回 None",
      G.fit_pose_from_landmarks(got2, [LEGS[k] for k in idx2]) is None, None)
# 预测位置错得离谱（belief 偏 1 m）⇒ 一条腿都对不上
far, _, missfar = G.legs_from_scan(synth_scan(truth), LEGS, (3.5, 3.5, belief[2]))
check("belief 位置错 1 m 时对不上腿（命中 {} 条）⇒ 不会给出错解 ✓".format(len(far)),
      len(far) <= 1, len(far))

print("\n" + "=" * 72)
print("用例④ 注入值：GraspPhase._apply_table_yaw_fix 必须写 e = belief − truth（符号不能反 ✗）")


class _FakeLog:
    def __init__(self):
        self.lines = []

    def _rec(self, tag, m):
        self.lines.append("{} {}".format(tag, m))

    def info(self, m):
        self._rec("INFO", m)

    def warn(self, m):
        self._rec("WARN", m)

    def error(self, m):
        self._rec("ERROR", m)


class _FakeSelf:
    """只提供 _apply_table_yaw_fix / _table_yaw_probe 用到的成员。"""

    def __init__(self, belief, pts):
        self.log = _FakeLog()
        self._table_legs_map = LEGS
        self._scan = (0.0, 0.0, 1.0, [], 0.12, 3.5)
        self._pts = pts
        self._belief = belief
        self.tf_buffer = types.SimpleNamespace(lookup_transform=lambda *a, **k: None)
        self._last_yaw_inject = None
        self.injected = []

    def _scan_points_base(self, max_points=4000):
        return self._pts

    def _robot_map_pose(self):
        return self._belief

    def wait_future(self, fut, timeout):
        return True

    def _inject_object_yaw(self, err):
        self.injected.append(err)
        return True


# 把真方法绑到假对象上（_apply_table_yaw_fix 内部会调 self._table_yaw_probe ✓）
_FakeSelf._table_yaw_probe = G.GraspPhase._table_yaw_probe


e_deg = 25.7
truth = (2.19, 2.61, math.radians(-90.0))
belief = (2.19, 2.61, math.radians(-90.0 + e_deg))
fake = _FakeSelf(belief, synth_scan(truth))
ret = G.GraspPhase._apply_table_yaw_fix(fake)
check("有偏差时必须注入（返回非 None）", ret is not None, ret)
check("注入值 = e = belief − truth（{:+.1f}°，符号不能反）".format(e_deg),
      bool(fake.injected) and abs(math.degrees(fake.injected[-1]) - e_deg) < 0.6,
      [math.degrees(v) for v in fake.injected])

# 偏差很小（<2° 容差）⇒ 不当成"修正"（返回 None），但**必须显式写 ≈0**：
# 参数是节点级的、会跨驱动进程残留 ⇒ 不写就会拿上一趟的旧 e 算朝向 ✗
fake2 = _FakeSelf((2.19, 2.61, math.radians(-90.3)), synth_scan((2.19, 2.61, math.radians(-90.0))))
ret2 = G.GraspPhase._apply_table_yaw_fix(fake2)
check("偏差 0.3° < 2° 容差 ⇒ 不算修正（返回 None）", ret2 is None, (ret2, fake2.injected))
check("…但必须显式写入 ≈0（清掉上一趟可能残留的旧值 ✗）",
      bool(fake2.injected) and abs(math.degrees(fake2.injected[-1])) < 2.0, fake2.injected)
fake3 = _FakeSelf((2.19, 2.61, math.radians(-90.3)), synth_scan((2.19, 2.61, math.radians(-90.0))))
fake3._last_yaw_inject = 0.2
G.GraspPhase._apply_table_yaw_fix(fake3)
check("上一趟注入过 e=11.5°、这一趟没偏差 ⇒ 写入被清成 ≈0（不残留）",
      bool(fake3.injected) and abs(math.degrees(fake3.injected[-1])) < 2.0, fake3.injected)
check("上一趟的 _last_yaw_inject 被清掉（后续 ★ 合拢轴核对 才会按 e=0 判 ✓）",
      fake3._last_yaw_inject is None, fake3._last_yaw_inject)

# 量不出来（没有扫描点）⇒ 不注入
fake4 = _FakeSelf(belief, None)
ret4 = G.GraspPhase._apply_table_yaw_fix(fake4)
check("拿不到 /scan ⇒ 不注入、保持原行为", ret4 is None and not fake4.injected, ret4)

# ══════════════════ 汇总 ══════════════════
print("\n" + "=" * 72)
if FAILED:
    print("✗ {} 项失败：{}".format(len(FAILED), "、".join(FAILED)))
    sys.exit(1)
print("✓ 全部通过（{} 项断言）—— 桌腿基准能还原真偏航，注入值符号正确".format(NCHECK[0]))
