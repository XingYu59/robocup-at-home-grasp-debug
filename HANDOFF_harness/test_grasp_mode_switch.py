#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线自测：视觉节点"要不要切抓取模式"的判据 needs_grasp_mode。

背景（2026-09-19 现场，一次 8 连败）：
    驱动在观察位扫 3 个角度 + 近看位扫 5 个角度，**8 次调用全部**返回
    "no detection / no class matched"、0 目标 ⇒ 直接 "没有可夹的目标" ✗
    而同一时刻的视觉日志里，那几帧明明认出了
        coke can 0.59 / 0.48 / 0.66 / 0.88、apple 0.35 / 0.42 / 0.43
    —— 全是**队友默认 4 类词表**（apple / coke can / bowl / banana）里的东西，
       而调用方要的是 potted_meat_can（不在那 4 类里）✗

    旧判据：`grasp_mode_on = (grasp_mode == "always"
                              or (grasp_mode == "auto"
                                  and not any(c in graspable for c in f0)))`
    ⇒ 只问"默认词表里有没有**可夹**的东西"，**完全不管调用方点了哪几类**。
      于是只要画面里有任何可夹的东西（本桌的 coke can、隔壁桌的 apple…）就永远不切模式，
      调用方点的那一类**永远没机会被命名** ⇒ 响应恒为 "no class matched" ✗✗

用法（不用起仿真、不用 source）：
    python3 HANDOFF_harness/test_grasp_mode_switch.py
"""
import importlib.util
import os
import sys
import types

SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                                      "turtlebot3_manipulation_grasp", "scripts"))
_STUBS = ("rclpy", "rclpy.node", "rclpy.callback_groups", "rclpy.qos",
          "rclpy.executors", "rclpy.time",
          "sensor_msgs", "sensor_msgs.msg", "geometry_msgs", "geometry_msgs.msg",
          "std_msgs", "std_msgs.msg", "vision_msgs", "vision_msgs.msg",
          "cv_bridge", "tf2_ros", "tf2_geometry_msgs",
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

# 这个节点 import torch/numpy 等重家伙 —— 用桩挡掉（本测试只用它的纯函数 ✓）
for _n in ("torch", "torchvision", "numpy", "cv2", "PIL", "PIL.Image"):
    if _n not in sys.modules:
        try:
            __import__(_n)
        except Exception:
            sys.modules[_n] = _Any(_n)

if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)
import detect_grasp_target_node as V  # noqa: E402

FAILED = []
NCHECK = [0]


def check(name, cond, detail=""):
    NCHECK[0] += 1
    print("  {} {}".format("✓" if cond else "✗", name) + ("" if cond else "   ← " + str(detail)))
    if not cond:
        FAILED.append(name)


GRASPABLE = ["apple", "coke can", "bleach_cleanser", "chips_can", "cracker_box",
             "gelatin_box", "mustard_bottle", "potted_meat_can", "sugar_box", "tomato_soup_can"]
HAS = getattr(V, "needs_grasp_mode", None)
print("=" * 72)
print("用例① 函数存在且可被离线调用")
check("detect_grasp_target_node.needs_grasp_mode 存在", callable(HAS), HAS)
if not callable(HAS):
    print("✗ 无法继续")
    sys.exit(1)

print("\n" + "=" * 72)
print("用例② ★ 现场那一次：调用方要 potted_meat_can，第 1 帧只认出 coke can/apple")
# 旧判据在这里返回 False ⇒ 不切模式 ⇒ potted_meat_can 永远没机会被命名 ⇒ 8 次调用全 0 目标 ✗
check("必须切抓取模式（否则调用方要的类永远认不出来 ✗）",
      HAS(["coke can", "apple"], ["potted_meat_can"], GRASPABLE) is True,
      HAS(["coke can", "apple"], ["potted_meat_can"], GRASPABLE))
# ★ 2026-09-19 第二版：**只要调用方点了类就一律走闭集复核** ✓
#   为什么：默认 4 类词表只能命名 apple/coke can/bowl/banana，而抓取侧请求的是 objects.yaml
#   的类别 ⇒ 旧判据"命中了就不切"会让整帧只给出那 4 类的名字（现场 12:20：apple 报在
#   隔壁餐桌位置 = 幻影 ✗）⇒ 抓取侧拿到幻影类别 ⇒ 抓空 ✗✗
check("点了类、第 1 帧正好命中 ⇒ **仍然切**（4 类词表给不出目录表里的类名 ✓）",
      HAS(["potted_meat_can"], ["potted_meat_can"], GRASPABLE) is True, None)
check("点了多类、命中其中一类 ⇒ 仍然切（同上 ✓）",
      HAS(["cracker_box"], ["potted_meat_can", "cracker_box"], GRASPABLE) is True, None)
check("调用方点了多类、一类都没命中 ⇒ 必须切",
      HAS(["coke can"], ["potted_meat_can", "cracker_box"], GRASPABLE) is True, None)
check("第 1 帧什么都没认出来 + 调用方点了类 ⇒ 切（走闭集复核 ✓）",
      HAS([], ["potted_meat_can"], GRASPABLE) is True, None)

print("\n" + "=" * 72)
print("用例③ 没点类时保持原行为（兼容队友的 auto 语义）")
check("空请求 + 认出可夹的 coke can ⇒ 不切（与原实现一致 ✓）",
      HAS(["coke can"], [], GRASPABLE) is False, None)
check("空请求 + 只认出不可夹的 bowl ⇒ 切（原实现一致 ✓）",
      HAS(["bowl"], [], GRASPABLE) is True, None)
check("空请求 + 什么都没认出 ⇒ 切（原实现一致 ✓）",
      HAS([], [], GRASPABLE) is True, None)

print("\n" + "=" * 72)
print("用例④ grasp_mode 参数 override（always / never 不能被上面的判据盖掉）")
check("always ⇒ 永远切（即使第 1 帧正好认出了调用方要的类）",
      HAS(["potted_meat_can"], ["potted_meat_can"], GRASPABLE, "always") is True, None)
check("never ⇒ 永远不切（即使什么都没认出来）",
      HAS([], ["potted_meat_can"], GRASPABLE, "never") is False, None)
check("auto（默认）⇒ 按上面的判据",
      HAS(["coke can"], ["potted_meat_can"], GRASPABLE, "auto") is True, None)

print("\n" + "=" * 72)
print("用例⑤ 回归：把现场 8 次调用的实际帧内容逐条喂进判据，必须都判成『切』")
# 现场视觉日志：4 类词表在 8 次调用里的命中情况（见 ~/.ros/log/python_30621_*.log）
FIELD_FRAMES = [
    {"coke can"}, {"coke can", "apple"}, {"coke can"}, {"coke can"},
    {"coke can"}, {"apple"}, {"coke can", "apple"}, set(),
]
n_switch = sum(1 for f in FIELD_FRAMES
               if HAS(sorted(f), ["potted_meat_can"], GRASPABLE))
check("8 次里 {} 次都会切抓取模式（旧判据是 0/8 ✗）".format(n_switch), n_switch == 8, n_switch)

print("\n" + "=" * 72)
if FAILED:
    print("✗ {} 项失败：{}".format(len(FAILED), "、".join(FAILED)))
    sys.exit(1)
print("✓ 全部通过（{} 项断言）—— 调用方点了类就必须按它判，否则永远是 no class matched"
      .format(NCHECK[0]))
