#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线自测：基础题"待计数物品清单"是否真的把**三处词表**一次同步（count_items）。

背景（规则书 2.3/2.5）：裁判现场发布三个物品英文名，机器人要识别+计数+在 RViz(/map) 打 Marker
+终端打印名称与数量 + 写 <组号>_answer.json（类别集合必须与裁判发布完全一致 ✗ 不能多不能少）。

原来"待计数物品"硬编码在三处，且**改漏一处就静默失效**：
    ① vision_pipeline.ITEM_NAMES / ITEM_ALIASES / TEXT_PROMPT   ← 开集 prompt + 归一表
    ② insid3_review.TARGET_CLASSES                              ← 闭集复核保留过滤
    ③ patrol_task.NAME_TO_JSON / TARGET_CLASSES_JSON            ← 答案 JSON 类别集合
本测试保证：喂一个"裁判给了 cola / bowl / windex bottle"的配置，三处**同时**变成它，
别名能命中（cola→coke can），答案类别按裁判写法，参考图里没有的类别会被点出来 ✓

用法（不用起仿真）：
    python3 HANDOFF_harness/test_count_items.py
"""
import os
import sys
import tempfile
import types

SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                                      "turtlebot3_manipulation_navigation2", "scripts"))
_STUBS = ("rclpy", "rclpy.action", "rclpy.node", "rclpy.parameter", "rclpy.time", "tf2_ros",
          "geometry_msgs", "geometry_msgs.msg", "nav2_msgs", "nav2_msgs.action",
          "sensor_msgs", "sensor_msgs.msg", "visualization_msgs", "visualization_msgs.msg",
          "turtlebot3_manipulation_grasp", "turtlebot3_manipulation_grasp.msg",
          "turtlebot3_manipulation_grasp.srv")
for _n in _STUBS:
    if _n not in sys.modules:
        try:
            __import__(_n)
        except Exception:
            sys.modules[_n] = types.ModuleType(_n)
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

import count_items  # noqa: E402
import vision_pipeline as vp  # noqa: E402
import insid3_review as ir  # noqa: E402

FAILED = []
NCHECK = [0]


def check(name, cond, detail=""):
    NCHECK[0] += 1
    print("  {} {}".format("✓" if cond else "✗", name) + ("" if cond else "   ← " + str(detail)))
    if not cond:
        FAILED.append(name)


CFG = """
count_items:
  - name: "coke can"
    aliases: ["coke", "coca", "cola", "soda", "can"]
  - name: "bowl"
    aliases: ["bowl"]
  - name: "windex bottle"
    aliases: ["windex", "glass cleaner"]
answer_classes: ["coke_can", "bowl", "windex_bottle"]
group_number: 7
"""

print("=" * 72)
print("用例① 现场场景：裁判给 cola / bowl / windex bottle ⇒ 三处词表是否同时变")
fd, path = tempfile.mkstemp(suffix=".yaml", dir=os.path.dirname(os.path.abspath(__file__)))
with os.fdopen(fd, "w", encoding="utf-8") as f:
    f.write(CFG)
before = (list(vp.ITEM_NAMES), list(ir.TARGET_CLASSES))
items, answers, group = count_items.apply_from_config(path=path, log=lambda *_: None)
WANT = ["coke can", "bowl", "windex bottle"]
check("① vision_pipeline.ITEM_NAMES 同步为 {}".format(WANT), vp.ITEM_NAMES == WANT,
      (before[0], vp.ITEM_NAMES))
check("② insid3_review.TARGET_CLASSES 同步（否则复核会把目标剔掉 ✗）",
      ir.TARGET_CLASSES == WANT, (before[1], ir.TARGET_CLASSES))
check("   开集 prompt 跟着变（GroundingDINO 只认 prompt 里的名字 ✓）",
      vp.TEXT_PROMPT == " . ".join(WANT) + " .", vp.TEXT_PROMPT)
check("   答案 JSON 类别 = 裁判写法", answers == ["coke_can", "bowl", "windex_bottle"], answers)
check("   组号读到了", group == 7, group)

print("\n" + "=" * 72)
print("用例② 别名要能命中裁判的叫法（规则书 2.3 的例子就是 'cola'）")
check("classify_phrase('cola') → coke can", vp.classify_phrase("cola") == "coke can",
      vp.classify_phrase("cola"))
check("classify_phrase('windex') → windex bottle",
      vp.classify_phrase("windex") == "windex bottle", vp.classify_phrase("windex"))
check("classify_phrase('a bowl on the table') → bowl",
      vp.classify_phrase("a bowl on the table") == "bowl",
      vp.classify_phrase("a bowl on the table"))
check("不在清单里的物体被丢弃（不参与计数 ✓）",
      vp.classify_phrase("banana") is None, vp.classify_phrase("banana"))

print("\n" + "=" * 72)
print("用例③ 护栏：参考图里没有的类别要点出来（复核叫不出名字 ⇒ 会被叫成最像的一类 ✗）")
msgs = []
count_items.apply_to_vision([{"name": "banana", "aliases": ["banana"]},
                             {"name": "not_a_real_item", "aliases": []}],
                            log=lambda m: msgs.append(str(m)))
check("没参考图的类别被点名警告", any("闭集参考图里没有" in m for m in msgs), msgs[-2:])
check("有参考图的类别（banana ✓）不报警",
      not any("banana" in m and "没有" in m for m in msgs), msgs)

print("\n" + "=" * 72)
print("用例④ 护栏：配置文件缺失/写坏时不能把整条链弄崩（退回代码默认 ✓）")
items2, answers2, group2 = count_items.apply_from_config(path="/nonexistent.yaml",
                                                         log=lambda *_: None)
check("文件不存在 ⇒ 返回空清单（patrol_task 沿用默认 4 类 ✓）",
      items2 == [] and answers2 == [], (items2, answers2))
with open(path, "w", encoding="utf-8") as f:
    f.write("count_items: [{name: }]\n")
items3, answers3, _ = count_items.apply_from_config(path=path, log=lambda *_: None)
check("YAML 写坏 ⇒ 不崩、返回空清单", items3 == [], items3)
os.unlink(path)

print("\n" + "=" * 72)
print("用例⑤ 现场配置自洽性（config/count_items.yaml 是**给人现场改的** ⇒ 只查一致性 ✓）")
its, ans, grp = count_items.load_items()
check("配置读得到（当前 {} 类）".format(len(its)), len(its) > 0, its)
check("answer_classes 非空（裁判要求类别集合不能少 ✓）", len(ans) > 0, ans)
# 答案类别必须是某一条物品的"下划线写法"（否则答案里那个类永远计数为 0 ✗）
keys = {it["name"].replace(" ", "_") for it in its}
check("answer_classes 都能对上物品清单（{}）".format(sorted(keys)),
      all(a in keys for a in ans), (ans, sorted(keys)))
# 物品名必须在闭集参考图里（否则复核叫不出名字 ⇒ 计数恒 0 ✗）
have = {"".join(k.lower().replace("_", " ").split()) for k in ir.INSID3_CLASSES}
bad = [it["name"] for it in its if "".join(it["name"].split()) not in have]
check("物品都在闭集参考图里（不在的点名 ✗）", not bad, bad)
print("     当前清单: {}".format([it["name"] for it in its]))
print("     答案类别: {}".format(ans))
# 代码兜底仍是原来的 4 类（没配置时行为不变 ✓）
import patrol_task
check("没配置时 patrol_task 仍回退到原来的 4 类（行为不变 ✓）",
      set(patrol_task.NAME_TO_JSON) >= {"apple", "coke can", "bowl", "banana"},
      patrol_task.NAME_TO_JSON)
check("patrol_task 里已经接线（main() 开头调用 apply_from_config ✓）",
      "count_items.apply_from_config" in open(os.path.join(SCRIPTS, "patrol_task.py"),
                                              encoding="utf-8").read(), "没找到")

# ══════════════════ 汇总 ══════════════════
print("\n" + "=" * 72)
if FAILED:
    print("✗ {} 项失败：{}".format(len(FAILED), "、".join(FAILED)))
    sys.exit(1)
print("✓ 全部通过（{} 项断言）—— 一处配置同步三处词表，计数链路完整".format(NCHECK[0]))
