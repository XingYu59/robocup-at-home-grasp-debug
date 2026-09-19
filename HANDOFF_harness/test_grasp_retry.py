#!/usr/bin/env python3
"""抓取闭环（接触判据 + 偏置重抓阶梯）的离线单测 —— 2026-09-18。

**不需要仿真**（不起 Gazebo / Nav2 / 抓取服务）：全部用纯函数 + 假对象跑真代码。

验收的 4 件事：
    ① classify_closure：空合 / 夹到 / 采不到数据 三种判定（判据 = 实测间距 − 指令间距）
    ② 偏置阶梯的方向与顺序：第一条必须是 (0,0)，**侧向先于纯径向**（依据见常量注释）
    ③ 真跑 _grasp_with_offsets（假 call_grasp）：
         · 前两档空合、第三档接触 ⇒ 必须在第三档停下并报成功 ✓
         · 全程空合 ⇒ 必须报失败（**不能因为服务返回 stage=0 就算成功** ✗）
         · 返回 stage=5（BAD_TARGET）⇒ 必须立刻停，不再扫偏置 ✓
    ④ 偏置确实加在发给服务的抓取点上（Δx 径向 / Δy 侧向），且障碍物一起平移 ✓

用法：
    cd ~/Robocup@home_ws && source install/setup.bash
    python3 src/HANDOFF_harness/test_grasp_retry.py
退出码 0 = 全过
"""

import sys
import types

sys.path.insert(0, "/home/xing/Robocup@home_ws/src/turtlebot3_manipulation_navigation2/scripts")

import grasp_phase as G          # noqa: E402

FAIL = []


def check(name, ok, detail=""):
    print("  {} {}{}".format("✓" if ok else "✗", name, ("  ← " + detail) if detail else ""))
    if not ok:
        FAIL.append(name)


class _Log:
    def info(self, *a): pass
    def warn(self, *a): pass
    def error(self, *a): pass


class _Clock:
    class _Now:
        @staticmethod
        def to_msg():
            from builtin_interfaces.msg import Time
            return Time(sec=0, nanosec=0)

    def now(self):
        return self._Now()


class StubGP:
    """只提供 `_grasp_with_offsets` / `_mk_target` 需要的成员（不碰真 ROS ✓）。"""

    _mk_target = G.GraspPhase._mk_target
    _grasp_with_offsets = G.GraspPhase._grasp_with_offsets

    def __init__(self, verdicts, fresh=None):
        # verdicts = [(verdict, stage, success), ...] 按调用顺序返回
        # fresh    = [(x, y) | None, ...] 每次"重测"返回的物体新位置（模拟被推走 ✓）
        self.log = _Log()
        self.node = types.SimpleNamespace(get_clock=lambda: _Clock())
        self.calls = []              # 每次调用收到的目标点/障碍点
        self._verdicts = list(verdicts)
        self._fresh = list(fresh or [])
        self.n_fresh = 0
        self.n_creep = 0

    def creep(self, cls, goal_d=None):
        """假微调：只计数（真身会用里程计闭环把物体开到 base(goal,0) ✓）"""
        self.n_creep += 1
        return True

    def _apply_table_yaw_fix(self, rp=None, verbose=True):
        """假桌腿基准（2026-09-19）：本套测试不碰激光/参数注入 ⇒ 只记调用次数 ✓
        （真身会用 /scan 扫桌腿反解偏航偏差，再注入 grasp_node 的 object_yaw_map）"""
        self.n_yaw_fix = getattr(self, "n_yaw_fix", 0) + 1
        return None

    def _remember(self, cls, tgt):
        """假跟踪刷新（真身会把它记成 map 坐标的跟踪基准 ✓）"""
        self.n_remember = getattr(self, "n_remember", 0) + 1

    def measure(self, cls):
        """假 measure：给【当前这一次】重测到的位置（跳过里程计快路径）"""
        if not self._fresh:
            return None
        return self._fresh[min(self.n_fresh - 1, len(self._fresh) - 1)]

    def _fresh_target(self, cls):
        """假重测：按 _fresh 表依次给"物体现在在哪"（空表 ⇒ 一直认不出来 ✓）"""
        self.n_fresh += 1
        if not self._fresh:
            return None, None, "none"
        x, y = self._fresh[min(self.n_fresh - 1, len(self._fresh) - 1)]
        return mk(cls, x, y), [], "vision"

    def call_grasp(self, target, obstacles):
        self.calls.append((target.point.x, target.point.y, target.class_id,
                           [(o.point.x, o.point.y) for o in obstacles]))
        v, stage, ok = self._verdicts[min(len(self.calls) - 1, len(self._verdicts) - 1)]
        return ok, stage, "stub", {"verdict": v, "why": "stub"}


def mk_target(cls, conf, x, y):
    return StubGP._mk_target(types.SimpleNamespace(), cls, conf, (x, y))


def mk(cls, x, y, conf=0.8):
    """造一个 GraspTargetStamped（不依赖 self）"""
    t = G.GraspTargetStamped()
    t.header.frame_id = "base_footprint"
    t.class_id = cls
    t.confidence = conf
    t.point.x, t.point.y, t.point.z = x, y, G.TABLE_TOP_Z
    return t


def main():
    print("=" * 72)
    print("① classify_closure（唯一物理判据：实测两指间距 − 指令间距）")
    print("=" * 72)
    # squeeze=0.0015（当前 YAML）→ 罐 66 mm 的指令值 = 66 − 3 = 63 mm
    v, why = G.classify_closure(63.0, 63.0, 66.0)
    check("指令 63 / 实测 63（罐，squeeze=0.0015）⇒ 空合 ✗", v == "empty", why)
    v, why = G.classify_closure(63.0, 66.0, 66.0)
    check("指令 63 / 实测 66（手指被罐挡住）⇒ 夹到 ✓", v == "contact", why)
    # squeeze=0.0045（旧值）→ 指令 57 mm，判别余量 9 mm
    v, _ = G.classify_closure(57.0, 57.0, 66.0)
    check("指令 57 / 实测 57（squeeze=0.0045）⇒ 空合 ✗", v == "empty")
    v, _ = G.classify_closure(57.0, 66.0, 66.0)
    check("指令 57 / 实测 66 ⇒ 夹到 ✓", v == "contact")
    v, _ = G.classify_closure(57.0, 29.0, 38.0)
    check("指令 29 / 实测 29（糖盒空合）⇒ 空合 ✗", v == "empty")
    v, _ = G.classify_closure(29.0, 25.0, 38.0, open_mm=80.0)
    check("窄边 38 / 指令 29 / 实测 25（合过头、比窄边还小）⇒ 空合 ✗（物体被挤走/不在两指间）",
          v == "empty")
    v, _ = G.classify_closure(None, 63.0, 66.0)
    check("没采到指令值 ⇒ unknown（不能瞎判 ✓）", v == "unknown")
    # ★ 现场实证：真夹住那次实测 65.9 mm（指令 63.0，窄边 66.0）—— 只比指令高 2.9 mm，
    #   阈值 2.5 mm ⇒ 只差 0.4 mm 就误判成"空合"✗ ⇒ 驱动会再抓一次（重复抓取流程 ✗）
    v, why = G.classify_closure(63.0, 65.9, 66.0, open_mm=80.0)
    check("真夹住那次（实测 65.9 / 指令 63.0 / 窄边 66.0）⇒ contact ✓（旧判据靠 0.4 mm 余量）",
          v == "contact", why)
    v, _ = G.classify_closure(63.0, 65.0, 66.0, open_mm=80.0)
    check("停在 65.0（= 窄边−1.0）⇒ contact ✓（第二判据生效）", v == "contact")
    v, _ = G.classify_closure(63.0, 64.0, 66.0, open_mm=80.0)
    check("停在 64.0（= 窄边−2.0，纯空合附近）⇒ 仍判 empty ✓（不误报）", v == "empty")
    v, _ = G.classify_closure(63.0, None, 66.0)
    check("没采到实测间距 ⇒ unknown", v == "unknown")
    # ★ 2026-09-18 现场日志抓到的真 bug：规划失败、夹爪一动没动，却报"夹到了"
    v, why = G.classify_closure(35.0, 80.0, 38.0, open_mm=80.0)
    check("夹爪一动没动（80→80，指令 35）⇒ unclosed（**不是** contact）", v == "unclosed", why)
    # ★ 2026-09-19 改判（依据现场 10:42）：目标窄边 38 mm 却停在 62 mm ⇒ 两指之间那东西
    #   比目标宽 24 mm ⇒ 夹住的**不是这一类物体**（旧期望写的是 contact，那时还不知道
    #   "手指被挡住"不等于"夹的是目标类别"——10:42 就是拿 50 mm 的目标夹住了 66 mm 的番茄罐
    #   且报成功 ✗）。现在判 wrong_object，驱动会停手并交回上层重新识别 ✓
    v, _ = G.classify_closure(35.0, 62.0, 38.0, open_mm=80.0)
    check("张开 80 / 实测 62（比目标窄边 38 宽 24 mm）⇒ wrong_object ✗（不是 contact）",
          v == "wrong_object")
    v, _ = G.classify_closure(35.0, 35.0, 38.0, open_mm=80.0)
    check("张开 80 / 实测 35（走满指令）⇒ empty ✗", v == "empty")
    v, _ = G.classify_closure(35.0, 79.0, 38.0, open_mm=80.0)
    check("张开 80 / 实测 79（只走了 1 mm）⇒ unclosed", v == "unclosed")

    print("\n" + "=" * 72)
    print("② 偏置阶梯：第一条 (0,0)、侧向先于纯径向、条数不超过 MAX_GRASP_TRIES")
    print("=" * 72)
    lad = G.GRASP_RETRY_OFFSETS
    check("第一条是 (0,0)（先试原估计点）", lad[0] == (0.0, 0.0), str(lad[0]))
    lat = [i for i, (dx, dy) in enumerate(lad) if dx == 0.0 and dy != 0.0]
    deeper = [i for i, (dx, dy) in enumerate(lad) if dx > 0 and dy == 0.0]
    # 2026-09-18 定稿：侧向优先（合爪方向余量最窄：罐 ±7 mm ⇒ 一偏就从旁边滑过），
    # 径向 ±20/±45/±80 作为兜底（原来的系统性偏浅已由"桌面剔除 + contract_range_offset=0"修掉）
    check("第一档之后先试【侧向】（罐的合爪方向只有 ±7 mm 余量）",
          lat and min(lat) < min(deeper), "侧向 idx={} 更深 idx={}".format(lat, deeper))
    check("径向兜底覆盖 ≥±45 mm（残留估计误差），最深 ≥80 mm",
          max(dx for dx, _ in lad) >= 0.080 and min(dx for dx, _ in lad) <= -0.045,
          "最深 {:+.0f} / 最浅 {:+.0f} mm".format(1000 * max(dx for dx, _ in lad),
                                                  1000 * min(dx for dx, _ in lad)))
    check("侧向覆盖 ±12 / ±24 mm（罐的余量只有 ±7 mm ⇒ 必须覆盖到）",
          (0.0, 0.012) in lad and (0.0, -0.012) in lad
          and (0.0, 0.024) in lad and (0.0, -0.024) in lad)
    check("也保留「更浅」档（报出的点偏深时的反向可能）",
          any(dx < 0 and dy == 0.0 for dx, dy in lad))
    check("阶梯长度 ≥ MAX_GRASP_TRIES（截断不会越界）",
          len(lad) >= G.MAX_GRASP_TRIES, "len={} max_tries={}".format(len(lad), G.MAX_GRASP_TRIES))

    print("\n" + "=" * 72)
    print("③ 真跑 _grasp_with_offsets（假 call_grasp）：空合→换偏置→接触")

    print("=" * 72)
    tgt = mk("tomato_soup_can", 0.38, 0.01)
    obs = [mk("bowl", 0.30, 0.20)]
    need = 2                      # 前 2 档空合，第 3 档接触
    verdicts = [("empty", 0, True)] * need + [("contact", 0, True)]
    gp = StubGP(verdicts)
    ok, stage, msg, tried = gp._grasp_with_offsets(tgt, obs)
    check("前 2 档空合、第 3 档接触 ⇒ 报成功 ✓", ok is True, "ok={} stage={}".format(ok, stage))
    check("正好试了 3 档就停（不浪费后面的 MTC 周期）", len(tried) == need + 1,
          "tried={}".format(tried))
    check("试过的档位 = 阶梯前 3 条", tried == list(lad[:need + 1]), str(tried))
    # 偏置确实加在发给服务的点上：第 k 次调用的点 = 原点 + 阶梯第 k 条
    okp = True
    for k, (dx, dy) in enumerate(lad[:need + 1]):
        got = gp.calls[k][:2]
        want = (tgt.point.x + dx, tgt.point.y + dy)
        if abs(got[0] - want[0]) > 1e-9 or abs(got[1] - want[1]) > 1e-9:
            okp = False
    check("每次调用的抓取点 = 原点 + 该档偏置（Δx 径向 / Δy 侧向）", okp,
          str([(round(c[0], 3), round(c[1], 3)) for c in gp.calls]))
    okp = all(abs(c[3][0][0] - (obs[0].point.x + dx)) < 1e-9
              for c, (dx, _dy) in zip(gp.calls, lad)) if gp.calls else False
    check("障碍物随抓取点一起平移（否则 MTC 会把障碍留在原地 ✗）", okp)

    gp2 = StubGP([("empty", 0, True)])
    ok, stage, msg, tried = gp2._grasp_with_offsets(tgt, obs)
    check("全程空合 ⇒ **报失败**（服务返回 stage=0 也不算成功 ✓）",
          ok is False and stage == 0, "ok={} stage={}".format(ok, stage))
    check("全程空合时试满 MAX_GRASP_TRIES 档", len(tried) == G.MAX_GRASP_TRIES,
          "{} 档".format(len(tried)))

    # ★ 现场实证（2026-09-19）：每次尝试都把罐子推向"更深 + −y"，且方向完全一致
    #   ⇒ 位移方向 = 物体真实所在方向 ⇒ 修正偏置应朝那个方向累加，跨 creep/重测都不丢 ✓
    #   数据取自真实日志：0.343,+0.003 → 0.363,-0.004 → 0.359,-0.014
    tgt2 = mk("tomato_soup_can", 0.343, 0.003)
    gp4 = StubGP([("empty", 0, True), ("empty", 0, True), ("contact", 0, True)],
                 fresh=[(0.363, -0.004), (0.359, -0.014)])
    ok4, _, _, _ = gp4._grasp_with_offsets(tgt2, [], standoff=0.33)
    pts4 = [(round(c[0], 3), round(c[1], 3)) for c in gp4.calls]
    check("被推走时按【跟随位移】累加修正偏置（朝更深 / −y 走），creep 后也不丢",
          pts4 == [(0.343, 0.003), (0.383, -0.011), (0.375, -0.031)],
          "实发探点={}".format(pts4))
    check("跟随模式下不再叠固定阶梯偏置，且单步不超过增益×位移（增益 1.0 ⇒ 保守）",
          all(abs(p_[0] - 0.343) < 0.09 for p_ in pts4) and G.FOLLOW_GAIN <= 1.0,
          "探点都在 ±90 mm 内 ✓；FOLLOW_GAIN={}".format(G.FOLLOW_GAIN))
    check("每档都重测一次（第 2、3 档前各一次）", gp4.n_fresh == 2,
          "n_fresh={}".format(gp4.n_fresh))
    check("物体横向挪动 >30 mm ⇒ 会自动重新对正（creep）", gp4.n_creep >= 1,
          "n_creep={}".format(gp4.n_creep))
    gp5 = StubGP([("empty", 0, True), ("contact", 0, True)], fresh=[(0.60, 0.002)])
    ok5, _, _, _ = gp5._grasp_with_offsets(mk("tomato_soup_can", 0.55, 0.010), [], standoff=0.38)
    pts5 = [(round(c[0], 3), round(c[1], 3)) for c in gp5.calls]
    check("重测认出物体在更深处（0.60 m）⇒ 直接按新位置抓（不再用旧的 0.38）",
          pts5[1][0] > 0.60 - 0.001, "实发点={}".format(pts5))

    # ★ 2026-09-19：跟随位移 —— 每次尝试都把物体推向 −y，探点应朝 −y 追过去并最终夹到
    #   现场日志：5 次尝试每次都把罐子推向 −y（方向完全一致）⇒ 位移方向 = 物体所在方向 ✓
    tgt3 = mk("tomato_soup_can", 0.343, 0.003)
    gp6 = StubGP([("empty", 0, True), ("empty", 0, True), ("contact", 0, True)],
                 fresh=[(0.350, -0.008), (0.355, -0.020)])
    ok6, _, _, _ = gp6._grasp_with_offsets(tgt3, [], standoff=0.33)
    pts6 = [(round(c[0], 3), round(c[1], 3)) for c in gp6.calls]
    check("第 3 档就夹到 ⇒ 报成功 ✓（收敛在 2 步内）", ok6 is True, "ok={}".format(ok6))
    check("探点序列沿「更深 + −y」收敛（不来回乱试）",
          pts6 == [(0.343, 0.003), (0.357, -0.019), (0.367, -0.043)],
          "实发探点={}".format(pts6))

    gp3 = StubGP([("unknown", 5, False)])
    ok, stage, msg, tried = gp3._grasp_with_offsets(tgt, obs)
    check("服务返回 stage=5（BAD_TARGET）⇒ 立刻停，只试 1 档（换偏置没用）",
          ok is False and stage == 5 and len(tried) == 1, "stage={} tried={}".format(stage, tried))

    print("\n" + "=" * 72)
    print("④ 目标选择：支撑面档位（本桌 0 / 不明 1 / 邻桌脚印内 2）")
    print("=" * 72)
    check("本桌脚印内 (2.30,2.00) ⇒ 档 0", G.GraspPhase._support_rank((2.30, 2.00)) == 0)
    check("邻桌 table_2 脚印内 (2.951,1.569) ⇒ 档 2",
          G.GraspPhase._support_rank((2.951, 1.569)) == 2)
    check("桌外不明处 (5.0,5.0) ⇒ 档 1（不硬拒，只是排后面 ✓）",
          G.GraspPhase._support_rank((5.0, 5.0)) == 1)

    print("\n" + "=" * 72)
    print("⑤ 视觉尺寸核对按【距离】放宽（P0-B）：远处掩码盖不满不再整帧丢弃")
    print("=" * 72)
    sys.path.insert(0, "/home/xing/Robocup@home_ws/src/turtlebot3_manipulation_grasp/scripts")
    import numpy as np                                     # noqa: E402
    import detect_grasp_target_node as D                   # noqa: E402

    class StubSize:
        _size_ok = D.DetectGraspTargetNode._size_ok
        size_check = True
        size_lo = 0.7
        size_hi = 1.4
        object_sizes = {"tomato_soup_can": (0.066, 0.066, 0.101),
                        "bowl": (0.16, 0.16, 0.07)}

    st = StubSize()

    def cloud(spread_x, z, n=200):
        xs = np.linspace(-spread_x / 2.0, spread_x / 2.0, n)
        return np.stack([xs, np.zeros(n), np.full(n, z)], axis=1)

    ok_far, obs_far, rng_far, z_far = st._size_ok("tomato_soup_can", cloud(0.027, 1.22))
    check("远处（1.22 m）掩码只盖 0.027 m ⇒ 放宽后**保留**（旧行为会丢整帧 ✗）",
          ok_far is True, "obs={:.3f} 区间下限={:.3f}".format(obs_far, rng_far[0]))
    ok_near, obs_near, rng_near, _ = st._size_ok("tomato_soup_can", cloud(0.027, 0.45))
    check("近处（0.45 m）掩码只盖 0.027 m ⇒ 仍然判不过（近处盖不满就是掩码有问题 ✓）",
          ok_near is False, "obs={:.3f} 区间={:.3f}~{:.3f}".format(obs_near, *rng_near))
    ok_big, obs_big, _, _ = st._size_ok("tomato_soup_can", cloud(0.160, 0.80))
    check("把 0.16 m 的碗当成罐（0.066）⇒ 上界仍然拦得住 ✓", ok_big is False,
          "obs={:.3f}".format(obs_big))
    ok_good, obs_good, _, _ = st._size_ok("tomato_soup_can", cloud(0.060, 1.00))
    check("正常（1.0 m 处实测 0.060 m ≈ 罐 0.066）⇒ 通过 ✓", ok_good is True,
          "obs={:.3f}".format(obs_good))

    print("\n" + ("全部通过 ✓" if not FAIL else "有 {} 条不过 ✗：{}".format(len(FAIL), FAIL)))
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
