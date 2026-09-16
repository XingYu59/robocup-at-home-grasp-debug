#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线自测：合拢时刻判定（analyze_closure / format_closure_report）。

不跑仿真、不跑 ROS：直接喂合成序列 `(t, x, y, z, gap_mm)`，看它能不能
① 认出"合拢开始/结束"的时刻，② 把"最大 x"正确归到 接近 / 合拢期间 / 合拢结束之后。

背景（现场那条待查线索）：日志里 `指尖轨迹` 报最远 x=0.400（比契约点 0.366 深 34 mm），
而 `落点核对` 报"差 0 mm" —— 后者只取最接近契约点的那一次采样，可能只是路过 ✗。
真正要回答的是：**两指开始合拢那一刻指尖在哪个 x**。本测试保证这段判定不会误报：

  用例①合拢期间 x 变深 34 mm  ⇒ 必须报 "出现在合拢【期间】"
  用例②合拢后 z 上升 + x 变深 ⇒ 必须报 "出现在合拢结束【之后】…属抬升/放回"
  另外还覆盖：最大 x 在接近段、间距采样有空洞、服务提前返回（结束时刻只能估计）、
  整段没有合拢动作（必须报"无法判定"，不许硬凑 ✗）

用法：
    python3 HANDOFF_harness/test_closure_phase.py
（直接跑即可：模块 import 不到 ROS 时会退化成桩依赖 —— 被测的两个函数本来就是纯函数）
"""
import os
import sys
import types

# ── 取到 grasp_phase 里的两个纯函数（不实例化任何节点）────────────────────
SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                                       "turtlebot3_manipulation_navigation2", "scripts"))
# import grasp_phase 会 import rclpy / 消息包：没 source install/setup.bash 时补桩。
# 被测代码不碰 ROS（假对象那两例也是），桩只为了让 import 走通 ✓
_STUBS = ("rclpy", "rclpy.action", "rclpy.time", "tf2_ros",
          "geometry_msgs", "geometry_msgs.msg", "sensor_msgs", "sensor_msgs.msg",
          "nav2_msgs", "nav2_msgs.action",
          "turtlebot3_manipulation_grasp", "turtlebot3_manipulation_grasp.msg",
          "turtlebot3_manipulation_grasp.srv")


class _Any(types.ModuleType):
    """假模块：任何属性都取得到（且同一个属性每次取到的是同一个对象）、还能当函数调用。"""

    def __getattr__(self, name):
        if name.startswith("__"):                 # 魔法属性照常报错，别搅乱 Python 内部
            raise AttributeError(name)
        child = _Any("{}.{}".format(self.__name__, name))
        setattr(self, name, child)                # 缓存：msg.point.x = 0.366 之后要读得回来
        return child

    def __call__(self, *args, **kwargs):
        return _Any("stub")


for _name in _STUBS:
    if _name not in sys.modules:
        try:
            __import__(_name)
        except Exception:
            sys.modules[_name] = _Any(_name)

if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)
import grasp_phase as G  # noqa: E402

TGT = (0.366, 0.004)        # 契约点 base(x, y)


# ══════════════════ 合成序列 ══════════════════
DT = 0.1                    # 与 grasp_phase 的采样周期一致（10 Hz）


def phase(n, gap0, gap1, x0, x1, z0, z1, y=0.004, gap_none=()):
    """造一段 n 个采样的线性段（含端点）→ [(t, x, y, z, gap), ...]，段内 dt = DT。"""
    out = []
    for k in range(n):
        a = k / (n - 1.0) if n > 1 else 0.0
        out.append((k * DT, x0 + (x1 - x0) * a, y, z0 + (z1 - z0) * a,
                    None if k in gap_none else gap0 + (gap1 - gap0) * a))
    return out


def seq(*parts):
    """把多段首尾相接（每段 n 个采样、dt = DT），时间戳全局单调、无空洞 ✓。"""
    out, t = [], 0.0
    for p in parts:
        for (ts, x, y, z, g) in p:
            out.append((ts + t, x, y, z, g))
        t = out[-1][0] + DT
    return out


FAILED = []
NCHECK = [0]


def check(name, cond, detail=""):
    NCHECK[0] += 1
    print("  {} {}".format("✓" if cond else "✗", name) + ("" if cond else "   ← " + detail))
    if not cond:
        FAILED.append(name)


def report(title, samples):
    f = G.analyze_closure(samples)
    print("\n── {} ──".format(title))
    for ln in G.format_closure_report(f, TGT[0], TGT[1]):
        print(ln)
    return f, G.format_closure_report(f, TGT[0], TGT[1])


# ══════════════════ 用例①：合拢【期间】x 变深 34 mm ══════════════════
# 接近段到契约点(0.366) → 合拢期间指尖继续深到 0.400（= 合拢时刻就深 34 mm）
# 注意：10 Hz 采样 + 2 mm 阈值 ⇒ "触发帧"必然比"合拢真正启动"晚 1 帧
#      （这段合拢 28 mm/1.8 s 里一帧 ~2.9 mm ⇒ 触发帧 x=0.370、前一帧 x=0.366）
print("=" * 72)
print("用例① 合拢期间 x 变深 34 mm ⇒ 期望：出现在合拢【期间】")
s1 = seq(phase(11, 80.0, 80.0, 0.214, 0.366, 0.845, 0.845),        # 接近段
         phase(9, 80.0, 57.0, 0.366, 0.400, 0.845, 0.845),         # 合拢：x 变深
         phase(11, 57.0, 57.0, 0.400, 0.400, 0.845, 0.845))        # 停住（0.5 s 静止）
f1, L1 = report("用例①", s1)
check("触发帧 = 累计减小刚过 2 mm 的那一帧（x 在契约点与最深处之间）",
      0.366 < f1["xyz_start"][0] < 0.400, str(f1["xyz_start"]))
check("触发前一帧仍在契约点 x = 0.366（合拢真启动处）",
      abs(f1["xyz_pre"][0] - 0.366) < 1e-9, str(f1["xyz_pre"]))
check("阶段归属 = during", f1["phase"] == "during", f1["phase"])
check("日志报 出现在合拢【期间】", "出现在合拢【期间】" in L1[0], L1[0])
check("日志给出合拢期间 x 范围（上限=最深处 0.400）",
      "[0.370, 0.400]" in L1[0], L1[0])
check("日志把『启动时的 x』括成区间（前一帧 → 触发帧）",
      "合拢前最后一帧 x=+0.366" in L1[1] and "触发帧 x=+0.370" in L1[1], L1[1])
check("合拢前 80.0 mm → 结束 57.0 mm",
      "合拢前 80.0 mm → 结束 57.0 mm" in L1[0], L1[0])
check("结束时刻未被标成估计值", "估计值" not in L1[1], L1[1])

# ══════════════════ 用例②：合拢【结束之后】z 上升、x 变深（抬升/放回）══════════════════
# 合拢在 x=0.366（契约点）结束 → 之后抬升 z 0.845→1.075 且 x 变深到 0.400 ⇒ 属抬升/放回
print("\n" + "=" * 72)
print("用例② 合拢后 z 上升 + x 变深 ⇒ 期望：出现在合拢结束【之后】…属抬升/放回")
s2 = seq(phase(11, 80.0, 80.0, 0.214, 0.366, 0.845, 0.845),        # 接近段
         phase(9, 80.0, 57.0, 0.366, 0.366, 0.845, 0.845),         # 合拢（x 不动）
         phase(7, 57.0, 57.0, 0.366, 0.366, 0.845, 0.845),         # 停住 → 判定结束
         phase(12, 57.0, 57.0, 0.366, 0.400, 0.845, 1.075))        # 抬升/放回
f2, L2 = report("用例②", s2)
check("合拢触发帧仍有 x = 契约点 0.366（合拢时指尖没动）",
      abs(f2["xyz_start"][0] - 0.366) < 1e-9, str(f2["xyz_start"]))
check("阶段归属 = after", f2["phase"] == "after", f2["phase"])
check("日志报 出现在合拢结束【之后】", "出现在合拢结束【之后】" in L2[0], L2[0])
check("日志报 属抬升/放回，与抓取无关", "属抬升/放回，与抓取无关" in L2[0], L2[0])
check("日志给出 z 在上升", "z 0.845→1.075 在上升" in L2[0], L2[0])
check("日志不含『出现在合拢【期间】』", "出现在合拢【期间】" not in L2[0], L2[0])

# ══════════════════ 用例③：最大 x 出现在【接近段】══════════════════
# 接近时冲过头到 0.400，合拢时已经在 0.366 ⇒ 0.400 与合拢时刻无关
print("\n" + "=" * 72)
print("用例③ 最大 x 在接近段（合拢前）⇒ 期望：出现在合拢【之前】")
s3 = seq(phase(11, 80.0, 80.0, 0.214, 0.400, 0.845, 0.845),        # 接近：冲过头
         phase(9, 80.0, 57.0, 0.366, 0.366, 0.845, 0.845),         # 合拢在契约点
         phase(11, 57.0, 57.0, 0.366, 0.366, 0.845, 0.845))
f3, L3 = report("用例③", s3)
check("阶段归属 = before", f3["phase"] == "before", f3["phase"])
check("日志报 出现在合拢【之前】", "出现在合拢【之前】" in L3[0], L3[0])
check("合拢时刻 Δx 仍为 +0 mm（不被路过值污染）", "Δx=+0 mm" in L3[0], L3[0])

# ══════════════════ 用例④：间距采样有空洞 ══════════════════
print("\n" + "=" * 72)
print("用例④ 间距采样有空洞（TF 偶尔查不到）⇒ 判定不受影响")
s4 = seq(phase(11, 80.0, 80.0, 0.214, 0.366, 0.845, 0.845, gap_none=(3, 7)),
         phase(9, 80.0, 57.0, 0.366, 0.400, 0.845, 0.845, gap_none=(4,)),
         phase(11, 57.0, 57.0, 0.400, 0.400, 0.845, 0.845))
f4, L4 = report("用例④", s4)
check("阶段归属仍 = during", f4["phase"] == "during", f4["phase"])
check("间距有效次数 < 总采样次数", f4["n_gap"] < f4["n"],
      "n={} n_gap={}".format(f4["n"], f4["n_gap"]))

# ══════════════════ 用例⑤：服务在静止判据满足前返回 ⇒ 结束时刻标"估计" ══════════════════
print("\n" + "=" * 72)
print("用例⑤ 合拢后只采到 0.2 s 就返回 ⇒ 结束时刻必须标『估计值』")
s5 = seq(phase(11, 80.0, 80.0, 0.214, 0.366, 0.845, 0.845),
         phase(9, 80.0, 57.0, 0.366, 0.400, 0.845, 0.845),
         phase(3, 57.0, 57.0, 0.400, 0.400, 0.845, 0.845))         # 合拢后仅 0.2 s 就返回
f5, L5 = report("用例⑤", s5)
check("结束时刻 end_estimated = True", f5["end_estimated"] is True, str(f5["end_estimated"]))
check("日志标出『结束时刻为估计值』", "结束时刻为估计值" in L5[1], L5[1])

# ══════════════════ 用例⑥：整段没有合拢 ⇒ 必须报"无法判定"（不许硬凑 ✗）══════════════════
print("\n" + "=" * 72)
print("用例⑥ 间距全程 80 mm 不变（没有合拢动作）⇒ 期望：无法判定")
s6 = seq(phase(20, 80.0, 80.0, 0.214, 0.400, 0.845, 1.075))
f6, L6 = report("用例⑥", s6)
check("analyze_closure 返回 None", f6 is None, str(f6))
check("日志报 无法判定（不输出合拢时刻数字）",
      "无法判定" in L6[0] and "合拢时刻: 指尖" not in L6[0], L6[0])
check("样本太少也返回 None", G.analyze_closure([(0.0, 0.3, 0.0, 0.9, 80.0)]) is None)

# ══════════════════ 用例⑦：单帧 TF 抖动不许当成"合拢开始" ══════════════════
# 接近段里间距抖一下（80 → 77 → 80），随后才真合拢 ⇒ 触发帧必须落在真合拢处
print("\n" + "=" * 72)
print("用例⑦ 接近段单帧间距抖动 80→77→80 ⇒ 不许误判为合拢开始")
s7 = seq(phase(6, 80.0, 80.0, 0.214, 0.300, 0.845, 0.845))          # 接近
s7 += [(s7[-1][0] + DT, 0.320, 0.004, 0.845, 77.0)]                  # 单帧抖动
s7 += [(s7[-1][0] + DT, 0.340, 0.004, 0.845, 80.0)]                  # 立刻弹回
s7 += [(t + s7[-1][0] + DT, x, 0.004, 0.845, g)                      # 真合拢（从 0.366 起）
       for (t, x, _, _, g) in phase(9, 80.0, 57.0, 0.366, 0.400, 0.845, 0.845)]
s7 += [(t + s7[-1][0] + DT, x, 0.004, 0.845, g)                      # 停住
       for (t, x, _, _, g) in phase(11, 57.0, 57.0, 0.400, 0.400, 0.845, 0.845)]
f7, L7 = report("用例⑦", s7)
check("合拢前最后一帧 x = 0.366（不受抖动帧 x=0.320 影响）",
      abs(f7["xyz_pre"][0] - 0.366) < 1e-9, str(f7["xyz_pre"]))
check("合拢前最后一帧间距仍是张开的 80.0 mm", abs(f7["gap_pre"] - 80.0) < 1e-9,
      str(f7["gap_pre"]))
check("抖动帧没被算成『合拢期间』（阶段仍 = during）", f7["phase"] == "during",
      f7["phase"])

# ══════════════════ 用例⑧：假对象跑一遍真 call_grasp（验证接线）══════════════════
# 纯函数对不代表接线对：这里用一个只有 call_grasp 会用到的成员的假对象，把
# GraspPhase.call_grasp 真跑一遍（真 rclpy 换成桩、真服务换成假 future），
# 确认 ① 采样确实带着时间戳/间距落进 samples，② ★ 合拢时刻那一行确实进了日志 ✓
class _FakeLog:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(m)

    def warn(self, m):
        self.lines.append("WARN " + m)

    def error(self, m):
        self.lines.append("ERROR " + m)


class _FakeFuture:
    def __init__(self, owner):
        self.owner = owner

    def done(self):
        return self.owner.i >= len(self.owner.series)

    def result(self):
        return G.GraspFixedObject.Response()


class _FakeGrasp:
    """假 self：只提供 call_grasp 用到的成员（不碰真 ROS、不发服务）。"""

    def __init__(self, series):
        self.series = series                      # [(x, z, gap_mm), ...] 每次采样一项
        self.i = 0
        self.log = _FakeLog()
        self._finger_q = 0.040
        self.object_sizes = {}
        self.close_squeeze = 0.0
        self.grasp_client = types.SimpleNamespace(call_async=self._call_async)

    def _call_async(self, req):
        self.i = 0                                # 进服务后再开始正式采样
        return _FakeFuture(self)

    def wait_client(self, client, timeout):
        return True

    def finger_gap_mm(self):
        return self.series[min(self.i, len(self.series) - 1)][2]

    def tcp_pose_base(self):
        s = self.series[min(self.i, len(self.series) - 1)]
        self.i += 1
        return (s[0], 0.004, s[1])

    def tcp_axes_base(self):
        return None

    def sleep(self, seconds):
        pass


def make_target(x, y, z, class_id="tomato_soup_can"):
    """真的 GraspTargetStamped（source 过 install 时）或桩对象都行 ✓。"""
    t = G.GraspTargetStamped()
    t.point.x, t.point.y, t.point.z = x, y, z
    t.class_id = class_id
    return t


def run_call_grasp(series):
    """跑一遍真 call_grasp，返回它打出来的日志行。"""
    fake = _FakeGrasp(series)
    tgt = make_target(TGT[0], TGT[1], 0.845)
    real_rclpy, G.rclpy = G.rclpy, types.SimpleNamespace(ok=lambda: True)
    try:
        G.GraspPhase.call_grasp(fake, tgt, [])
    finally:
        G.rclpy = real_rclpy                       # 桩只在这几秒内生效，跑完还回去
    return [m for m in fake.log.lines if not m.startswith(("WARN", "ERROR"))]


def series_from(*parts):
    """拼成 call_grasp 用的 (x, z, gap) 序列（同一套分段，见上面的 phase/seq）。"""
    return [(x, z, g) for (_t, x, _y, z, g) in seq(*parts)]


print("\n" + "=" * 72)
print("用例⑧ 假对象真跑 call_grasp：合拢期间 x 变深 34 mm ⇒ 日志必须报【期间】")
log8 = run_call_grasp(series_from(
    phase(10, 80.0, 80.0, 0.214, 0.366, 0.845, 0.845),      # 接近
    phase(9, 80.0, 57.0, 0.366, 0.400, 0.845, 0.845),       # 合拢（x 变深）
    phase(6, 57.0, 57.0, 0.400, 0.400, 0.845, 0.845),       # 停住 → 判定结束
    phase(9, 57.0, 57.0, 0.400, 0.360, 0.845, 0.950)))      # 抬升
star8 = [m for m in log8 if "★ 合拢时刻" in m]
check("call_grasp 打出了 ★ 合拢时刻 那一行", len(star8) == 1, str(log8))
check("该行报『出现在合拢【期间】』",
      bool(star8) and "出现在合拢【期间】" in star8[0], str(star8))
check("采样确实落盘（原有『指尖轨迹』那行还在）",
      any("指尖轨迹" in m for m in log8), str(log8))

print("\n" + "=" * 72)
print("用例⑨ 假对象真跑 call_grasp：合拢后抬升时 x 变深 ⇒ 必须报【之后】…属抬升/放回")
log9 = run_call_grasp(series_from(
    phase(10, 80.0, 80.0, 0.214, 0.366, 0.845, 0.845),      # 接近到契约点
    phase(9, 80.0, 57.0, 0.366, 0.366, 0.845, 0.845),       # 合拢（x 不动 = 正常）
    phase(6, 57.0, 57.0, 0.366, 0.366, 0.845, 0.845),       # 停住 → 判定结束
    phase(9, 57.0, 57.0, 0.366, 0.400, 0.845, 1.075)))      # 抬升/放回（x 变深）
star9 = [m for m in log9 if "★ 合拢时刻" in m]
check("★ 行仍打出且合拢时刻 x = 契约点（Δx=+0 mm）",
      bool(star9) and "Δx=+0 mm" in star9[0], str(star9))
check("该行报『出现在合拢结束【之后】』",
      bool(star9) and "出现在合拢结束【之后】" in star9[0], str(star9))
check("该行报『属抬升/放回，与抓取无关』",
      bool(star9) and "属抬升/放回，与抓取无关" in star9[0], str(star9))
print("  ── call_grasp 实际打出来的那两行（现场日志长这样）──")
for m in log9:
    if "合拢" in m:
        print("  " + m)

# ══════════════════ 汇总 ══════════════════
print("\n" + "=" * 72)
if FAILED:
    print("✗ {} 项失败：{}".format(len(FAILED), "、".join(FAILED)))
    sys.exit(1)
print("✓ 全部通过（{} 项断言）—— 合拢开始/结束判定与阶段归属符合预期".format(NCHECK[0]))
