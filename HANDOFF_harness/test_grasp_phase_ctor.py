#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""冒烟测试：**能不能把 GraspPhase 构造出来**（不跑仿真、不发服务）。

为什么必须有它（2026-09-19 现场事故）：
    我给"不带 --classes 就请求全部类别"加了一行
        self.all_classes = sorted(self.graspable) ...
    但它被放到了 `self.graspable = load_graspable()` **之前** ✗
    ⇒ 构造就抛 `'GraspPhase' object has no attribute 'graspable'`
    ⇒ 整个 Phase 2 直接不跑，用户看到的就是"改完之后没法运行正常任务" ✗✗
    py_compile 过（语法没错）、离线单测也全过（它们都不构造 GraspPhase ✗）
    ⇒ 缺的正是这一条"构造冒烟测试" ✓

做法：用假 node 构造 GraspPhase（订阅/客户端/发布器都给桩），断言：
  ① 构造不抛异常；
  ② 关键成员齐全（graspable / all_classes / object_sizes / close_squeeze / obs_normal…）；
  ③ all_classes = objects.yaml 的全部类别（不是空 ✗ ——它是"不带 --classes"时的请求内容）；
  ④ 不带 --classes 时 fetch_targets 会把 all_classes 填进请求（静态检查那一行还在 ✓）。

用法（不用起仿真、不用 source）：
    python3 HANDOFF_harness/test_grasp_phase_ctor.py
"""
import os
import sys
import types

SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                                      "turtlebot3_manipulation_navigation2", "scripts"))
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


for _n in _STUBS:
    if _n not in sys.modules:
        try:
            __import__(_n)
        except Exception:
            sys.modules[_n] = _Any(_n)

# tf2_ros 真身会去 rclpy 的 Node 上挂订阅/回调组（跟本测试无关）⇒ 直接换桩 ✓
#   本测试只验证"构造不抛异常 + 成员齐全"，不碰任何 TF 查询 ✓
_tf = types.ModuleType("tf2_ros")


class _Buf:
    def lookup_transform(self, *a, **k):
        raise RuntimeError("stub: 没有 TF")


class _TL:
    def __init__(self, buf, node):
        pass


_tf.Buffer = _Buf
_tf.TransformListener = _TL
sys.modules["tf2_ros"] = _tf

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


class _FakeLog:
    def info(self, m):
        pass

    def warn(self, m):
        pass

    def error(self, m):
        pass

    def debug(self, m):
        pass


class _Permissive:
    """取属性 → 可调用；调用 → 又一个 _Permissive（够 TransformListener 用 ✓）"""

    def __init__(self, name="stub"):
        self._name = name

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        return _Permissive("{}.{}".format(self._name, name))

    def __call__(self, *a, **k):
        return _Permissive(self._name + "()")

    # rclpy 的 ActionClient 会写 `with node.handle:` ⇒ 桩要能当上下文管理器 ✓
    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False


class _FakeNode:
    """只提供 GraspPhase.__init__ 用到的 Node 接口。"""

    def __init__(self):
        self.subs, self.clients, self.pubs = [], [], []

    def get_logger(self):
        return _FakeLog()

    def create_subscription(self, *a, **k):
        self.subs.append(a[0] if a else None)
        return types.SimpleNamespace()

    def create_client(self, srv_type, name):
        self.clients.append(name)
        return types.SimpleNamespace(wait_for_service=lambda timeout_sec=0.0: False)

    def create_publisher(self, *a, **k):
        self.pubs.append(a)
        return types.SimpleNamespace(publish=lambda m: None)

    # ── 兜底：真身 tf2_ros.TransformListener 会用到别的 Node API
    #    （default_callback_group / add_entity / destroy_subscription / get_clock…）
    #    ⇒ 未知属性一律给"什么都能调、取到什么都能继续用"的桩 ✓
    def __getattr__(self, name):
        if name.startswith("_"):
            raise AttributeError(name)
        return _Permissive(name)


print("=" * 72)
print("用例① 构造 GraspPhase（假 node）—— 不能抛异常")
node = _FakeNode()
gp = None
try:
    # nav_client 直接给桩：rclpy 的 ActionClient 是 pybind 类，构造时要真 Node ✗
    # （GraspPhase 本来就支持外部注入 nav_client —— patrol_task 就是这么传的 ✓）
    gp = G.GraspPhase(node, nav_client=types.SimpleNamespace())
    ok = True
    err = ""
except Exception as e:                       # noqa: BLE001
    import traceback
    traceback.print_exc()                    # 出问题要看到底哪一行 ✗
    ok, err = False, "{}: {}".format(type(e).__name__, e)
check("构造成功（不抛异常）", ok, err)
if ok:
    print("\n" + "=" * 72)
    print("用例② 关键成员齐全")
    for attr in ("graspable", "all_classes", "object_sizes", "close_squeeze", "obs_normal",
                 "_track", "_scan", "_last_yaw_inject", "_table_legs_map", "tf_buffer",
                 "vision_client", "grasp_client", "cmd_pub"):
        check("self.{} 存在".format(attr), hasattr(gp, attr), "缺失")
    print("\n" + "=" * 72)
    print("用例③ all_classes = objects.yaml 全部类别（不带 --classes 时的请求内容）")
    check("all_classes 非空（{} 类）".format(len(getattr(gp, "all_classes", []))),
          len(getattr(gp, "all_classes", [])) > 0, gp.all_classes if ok else None)
    if getattr(gp, "graspable", None):
        check("all_classes == sorted(graspable)（同一来源 ✓）",
              list(gp.all_classes) == sorted(gp.graspable), (gp.all_classes, sorted(gp.graspable)))
    check("含 potted_meat_can / tomato_soup_can / cracker_box（本桌测试物体 ✓）",
          all(c in gp.all_classes for c in ("potted_meat_can", "tomato_soup_can", "cracker_box")),
          gp.all_classes)
    check("订阅了 /joint_states 与 /scan", len(node.subs) >= 2, node.subs)

print("\n" + "=" * 72)
print("用例④ 静态检查：不带 --classes 时请求里填的是 all_classes（不是空列表 ✗）")
src = open(os.path.join(SCRIPTS, "grasp_phase.py"), encoding="utf-8").read()
check("fetch_targets: req.class_ids = self.target_classes or self.all_classes",
      "self.target_classes or self.all_classes" in src, "没找到")
check("all_classes 的赋值在 self.graspable 之后（构造顺序 ✗ 别再犯）",
      src.index("self.all_classes = sorted") > src.index("self.graspable = load_graspable"),
      "顺序不对")

# ══════════════════ 汇总 ══════════════════
print("\n" + "=" * 72)
if FAILED:
    print("✗ {} 项失败：{}".format(len(FAILED), "、".join(FAILED)))
    sys.exit(1)
print("✓ 全部通过（{} 项断言）—— GraspPhase 能构造、默认类别集正确".format(NCHECK[0]))
