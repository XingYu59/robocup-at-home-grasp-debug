#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线自测：标签按实测尺寸纠正（relabel_decision）+ 驱动侧「抓错物体」判据。

两趟现场（2026-09-19）把这条路的边界钉死了：

  ① 10:32 —— **真·扁罐头**：分类器把它叫成 `chips can`(目录高 0.25 m)，
     掩码实测 55×74 mm（目录 50×97×82）⇒ 标签**明显不自洽** ⇒ 应该改标成 potted_meat_can ✓

  ② 10:42 —— **翻车现场**：调用方要 potted_meat_can，分类器**认对了**（tomato_soup_can 0.36），
     但番茄罐（66×66×101）的掩码实测是 **55×90 mm**（系统性偏小 ~11%），
     而 potted_meat_can 是 50×97×82 ⇒ 高度只差 8 mm ⇒ 我第一版把高度容差放到 15 mm，
     就把番茄罐**改标**成了扁罐头 ⇒ 驱动去抓番茄罐 ⇒ 合到 61.3 mm（目标窄边 50）时
     手指确实被挡住了 ⇒ 旧判据报 [contact]「抓取成功」✗✗

  结论（也写进了代码注释）：**单视角的 (横向,竖直) 分不开 82 mm 的方盒与 101 mm 的圆柱**
  （可见面长宽比 0.61 vs 0.65，而实测整体偏小 10% ⇒ 小一号的物体冒充目标类别）
  ⇒ 唯一安全的规则：**分类器自己的标签只要还自洽，就绝不改它** ✓
  ⇒ 并且驱动侧必须能用「实测间距 vs 目标窄边」看出「夹住的是另一个东西」✓

用法（不用起仿真）：
    python3 HANDOFF_harness/test_shape_relabel.py
"""
import math
import os
import sys
import types

CAT = {   # objects.yaml 的真实尺寸 (depth, width, height)
    "potted_meat_can": (0.050, 0.097, 0.082),
    "chips_can": (0.075, 0.075, 0.250),
    "sugar_box": (0.038, 0.089, 0.175),
    "tomato_soup_can": (0.066, 0.066, 0.101),
    "coke can": (0.067, 0.067, 0.1239),
    "cracker_box": (0.060, 0.158, 0.210),
    "gelatin_box": (0.073, 0.085, 0.028),
    "mustard_bottle": (0.058, 0.095, 0.190),
    "apple": (0.0757, 0.0749, 0.0724),
    "bowl": (0.1603, 0.1574, 0.0544),
}
CATALOG = {k: {"dims": v, "graspable": True} for k, v in CAT.items()}

HERE = os.path.dirname(os.path.abspath(__file__))
VISION = os.path.abspath(os.path.join(HERE, "..", "turtlebot3_manipulation_grasp", "scripts"))
NAV = os.path.abspath(os.path.join(HERE, "..", "turtlebot3_manipulation_navigation2", "scripts"))

# 只取纯函数：把源码里那一段直接 exec 出来，免得为了它去造 rclpy/torch 的一堆桩 ✓
src = open(os.path.join(VISION, "detect_grasp_target_node.py"), encoding="utf-8").read()
_ns = {"math": math}
exec(src[src.index("def face_dev_mm"):src.index("class DetectGraspTargetNode")], _ns)
relabel_decision = _ns["relabel_decision"]

_STUBS = ("rclpy", "rclpy.action", "rclpy.time", "tf2_ros", "geometry_msgs", "geometry_msgs.msg",
          "sensor_msgs", "sensor_msgs.msg", "nav2_msgs", "nav2_msgs.action",
          "turtlebot3_manipulation_grasp", "turtlebot3_manipulation_grasp.msg",
          "turtlebot3_manipulation_grasp.srv")


class _Any(types.ModuleType):
    def __getattr__(self, n):
        if n.startswith("__"):
            raise AttributeError(n)
        ch = _Any("{}.{}".format(self.__name__, n))
        setattr(self, n, ch)
        return ch

    def __call__(self, *a, **k):
        return _Any("stub")


for _n in _STUBS + ("numpy", "cv2"):
    if _n not in sys.modules:
        try:
            __import__(_n)
        except Exception:
            sys.modules[_n] = _Any(_n)
if NAV not in sys.path:
    sys.path.insert(0, NAV)
import grasp_phase as G  # noqa: E402

FAILED = []
NCHECK = [0]


def check(name, cond, detail=""):
    NCHECK[0] += 1
    print("  {} {}".format("✓" if cond else "✗", name) + ("" if cond else "   ← " + str(detail)))
    if not cond:
        FAILED.append(name)


print("=" * 72)
print("用例① ★ 10:32（真·扁罐头）：分类器叫 chips can，掩码实测 55×74 mm，要 potted_meat_can")
c1, why1 = relabel_decision(0.055, 0.074, "chips_can", ["potted_meat_can"], CATALOG)
check("标签偏差 ~176 mm ⇒ 明显不自洽 ⇒ 必须改标成 potted_meat_can ✓",
      c1 == "potted_meat_can", (c1, why1))
print("     理由: " + why1)

print("\n" + "=" * 72)
print("用例② ★★ 10:42（翻车现场）：分类器**认对了**番茄罐，掩码实测 55×90 mm")
c2, why2 = relabel_decision(0.055, 0.090, "tomato_soup_can", ["potted_meat_can"], CATALOG)
check("标签自洽（偏差 16 mm）⇒ 绝不许改标（改了就抓错物体 ✗✗）", c2 is None, (c2, why2))
print("     理由: " + why2)
c2b, _ = relabel_decision(0.054, 0.090, "tomato_soup_can", ["potted_meat_can"], CATALOG)
check("换一帧（54×90 mm）同样不许改标", c2b is None, c2b)
c2c, _ = relabel_decision(0.058, 0.107, "coke can", ["potted_meat_can"], CATALOG)
check("可乐罐（实测 58×107）也不许改成扁罐头", c2c is None, c2c)

print("\n" + "=" * 72)
print("用例③ 该改的还是要改（高矮差好几倍的错标签）")
c3, _ = relabel_decision(0.055, 0.074, "sugar_box", ["potted_meat_can"], CATALOG)
check("sugar_box(0.175 m) vs 实测 74 mm ⇒ 改标成 potted_meat_can ✓", c3 == "potted_meat_can", c3)
c4, _ = relabel_decision(0.060, 0.076, "cracker_box", ["potted_meat_can"], CATALOG)
check("cracker_box(0.21 m) 同理 ⇒ 改标 ✓", c4 == "potted_meat_can", c4)

print("\n" + "=" * 72)
print("用例④ 护栏：对不上 / 量不到 / 没点类 ⇒ 不改也不猜")
c5, why5 = relabel_decision(0.075, 0.250, "bowl", ["potted_meat_can"], CATALOG)
check("实测 75×250 mm ⇒ 与 potted_meat_can 也对不上 ⇒ 不改 ✗", c5 is None, (c5, why5))
print("     理由: " + why5)
c6, why6 = relabel_decision(None, None, "chips_can", ["potted_meat_can"], CATALOG)
check("量不到尺寸 ⇒ 不改 ✓", c6 is None, why6)
c7, why7 = relabel_decision(0.055, 0.074, "chips_can", [], CATALOG)
check("调用方没点类 ⇒ 不改 ✓", c7 is None, why7)
c8, why8 = relabel_decision(0.075, 0.240, "gelatin_box", ["potted_meat_can"], CATALOG)
check("实测 75×240 mm 明显不是扁罐头 ⇒ 不许硬改 ✗", c8 is None, (c8, why8))

print("\n" + "=" * 72)
print("用例⑤ 驱动侧判据：实测间距比目标窄边宽 >4 mm ⇒ 判【抓错物体】（10:42 就是这样）")
v1, w1 = G.classify_closure(47.0, 61.3, 50.0, open_mm=80.0)
check("指令 47 / 实测 61.3 / 目标窄边 50 ⇒ wrong_object ✗（旧判据给 contact = 成功 ✗✗）",
      v1 == "wrong_object", (v1, w1))
print("     理由: " + w1)
v2, _ = G.classify_closure(47.0, 50.0, 50.0, open_mm=80.0)
check("真夹住目标（实测 50.0 ≈ 窄边 50）⇒ 仍是 contact ✓", v2 == "contact", v2)
v3, _ = G.classify_closure(47.0, 47.0, 50.0, open_mm=80.0)
check("空合（实测 = 指令 47.0）⇒ 仍是 empty ✓", v3 == "empty", v3)
v4, _ = G.classify_closure(47.0, 80.0, 50.0, open_mm=80.0)
check("合爪没动 ⇒ 仍是 unclosed ✓", v4 == "unclosed", v4)
v5, _ = G.classify_closure(47.0, 53.5, 50.0, open_mm=80.0)
check("实测比窄边宽 3.5 mm（容差内）⇒ 不算抓错，仍判 contact ✓", v5 == "contact", v5)
v6, _ = G.classify_closure(62.9, 65.9, 66.0, open_mm=80.0)
check("圆柱（番茄罐 66 mm）正常夹住 ⇒ contact ✓（新判据不误伤正常抓取 ✓）", v6 == "contact", v6)

print("\n" + "=" * 72)
print("用例⑥ 标签抖动替代的半径（12:20 现场：用 0.12 m 外的可乐罐代替番茄罐 ⇒ 抓空 ✗✗）")


class _FakeGP:
    """只提供 _fresh_target 用到的成员（真方法绑上去跑 ✓）"""
    _fresh_target = G.GraspPhase._fresh_target
    _to_map = staticmethod(G.GraspPhase._to_map)     # staticmethod：不然会被当绑定方法 ✗

    def __init__(self, dets, track_map_xy, rp_map=(2.60, 2.60, -1.5708)):
        self.log = G.GraspPhase.__dict__  # 占位，下面换成真的假日志
        self.lines = []

        class _L:
            def info(self, m):
                self_ = None

            def warn(self, m):
                pass

        self.log = types.SimpleNamespace(
            info=lambda m: self.lines.append(("INFO", m)),
            warn=lambda m: self.lines.append(("WARN", m)),
            error=lambda m: self.lines.append(("ERROR", m)))
        self._dets = dets                     # [(class_id, base_x, base_y, conf)]
        self._track = {"tomato_soup_can": (1e9, track_map_xy)}   # (时间戳, map 坐标)
        self._rp = rp_map
        self.last_targets = []

    def fetch_targets(self):
        out = []
        for (c, x, y, cf) in self._dets:
            t = types.SimpleNamespace(class_id=c, confidence=cf,
                                      point=types.SimpleNamespace(x=x, y=y))
            out.append(t)
        return out

    def _robot_map_pose(self):
        return self._rp


def _run(dets, track_map_xy):
    gp = _FakeGP(dets, track_map_xy)
    tgt, obs, src = gp._fresh_target("tomato_soup_can")
    return gp, tgt, src


# 12:20 现场：番茄罐没被认出，最近的其它类别检测（可乐罐）在 0.12 m 外
_RP = (2.60, 2.60, -1.5708)
_DET_BASE = (0.352, 0.117)                       # 检测到的可乐罐（base 系）
_DET_MAP = G.GraspPhase._to_map(_DET_BASE, _RP)  # 它在 map 里的位置（跟踪基准要跟它比）
near12, t12, s12 = _run([("coke can", _DET_BASE[0], _DET_BASE[1], 0.54)],
                        (_DET_MAP[0] + 0.12, _DET_MAP[1]))
check("别的类别检测在 0.12 m 外 ⇒ **拒绝替代**（返回 None）✗", t12 is None, (t12, s12))
check("并且打出了『那是另一个物体，不许替代』的解释",
      any("另一个物体" in m for _t, m in near12.lines), near12.lines)
# 同一物体的标签抖动：位置只差 5 cm ⇒ 允许替代
near5, t5, s5 = _run([("coke can", _DET_BASE[0], _DET_BASE[1], 0.54)],
                     (_DET_MAP[0] + 0.05, _DET_MAP[1]))
check("位置只差 0.05 m ⇒ 判为同一物体标签抖动，允许替代 ✓", t5 is not None, (t5, s5))
check("常量 LABEL_FLICKER_RADIUS = 0.10 m", abs(G.LABEL_FLICKER_RADIUS - 0.10) < 1e-9,
      G.LABEL_FLICKER_RADIUS)

print("\n" + "=" * 72)
print("用例⑦ 默认（不带 --classes）请求的类别集 = objects.yaml 全部类别，不再是空 ✗")
srcg = open(os.path.join(NAV, "grasp_phase.py"), encoding="utf-8").read()
check("fetch_targets 里写的是 `self.target_classes or self.all_classes`（空列表不再是空请求 ✓）",
      "self.target_classes or self.all_classes" in srcg, "没找到")
check("__init__ 里准备了 all_classes（来自 objects.yaml ✓）",
      "self.all_classes = sorted(self.graspable)" in srcg, "没找到")

# ══════════════════ 汇总 ══════════════════
print("\n" + "=" * 72)
if FAILED:
    print("✗ {} 项失败：{}".format(len(FAILED), "、".join(FAILED)))
    sys.exit(1)
print("✓ 全部通过（{} 项断言）—— 改标判据 / 抓错物体判据 / 标签抖动替代半径 / 默认请求类别集".format(NCHECK[0]))
