#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""基础题"待计数物品清单"的**唯一入口**：读 config/count_items.yaml，
把三处硬编码词表一次同步掉（vision_pipeline / insid3_review / patrol_task）✓

为什么需要它：待计数物品原来散在三处硬编码 ✗，裁判现场发布三个物品名之后
必须同时改这三处，且**改漏一处就静默失效**：
    ① `vision_pipeline.ITEM_NAMES/ITEM_ALIASES/TEXT_PROMPT`
       ⇒ GroundingDINO 的 prompt（不认的物品根本不会被检出 ✗）
       ⇒ `classify_phrase()` 的归一表（不在表里的物体会被丢掉 ✗）
    ② `insid3_review.TARGET_CLASSES`
       ⇒ 闭集复核的保留过滤（不在表里的目标会被复核剔掉 ✗）
    ③ `patrol_task.NAME_TO_JSON / TARGET_CLASSES_JSON`
       ⇒ 答案 JSON 的类别集合（与裁判发布不一致 ⇒ 不计分 ✗）
另外三个都是**模块级常量**（import 时定死）⇒ 必须在建节点/建流水线**之前**改 ✓，
所以 patrol_task.main() 一进来就调 apply_from_config() ✓

用法（patrol_task 已经替你调好，一般不用手动）：
    python3 scripts/count_items.py            # 只打印将生效的清单（自检 ✓）
"""
import os

HERE = os.path.dirname(os.path.realpath(__file__))
DEFAULT_CFG = os.path.join(os.path.dirname(HERE), "config", "count_items.yaml")


def _name(s):
    """类别名/别名归一：小写、下划线→空格、压掉多余空白（**保留单个空格** ✓）

    必须保留空格：`classify_phrase()` 是拿别名去 detector 的 phrase 里做**子串匹配**的
    （phrase 里就是 "red round apple" 这种带空格的写法）⇒ 别名必须同形 ✓
    """
    return " ".join((s or "").lower().replace("_", " ").split())


def _key(s):
    """只用于**比较**（"coke can" 与 "coke_can" 等价 ✓），不写回任何词表 ✓"""
    return "".join((s or "").lower().replace("_", " ").split())


def load_items(path=None):
    """→ (items, answer_classes, group_number)；items = [{'name':…, 'aliases':[…]}]

    items[i]['name'] 用**空格写法**（视觉 prompt 的写法 ✓）；
    answer_classes 用**裁判的写法**（下划线/大小写照抄 ✓）。
    """
    p = path or os.environ.get("COUNT_ITEMS_CFG") or DEFAULT_CFG
    items, answers, group = [], [], None
    if not os.path.isfile(p):
        return items, answers, group
    try:
        import yaml
        doc = yaml.safe_load(open(p, encoding="utf-8")) or {}
        for it in (doc.get("count_items") or []):
            if isinstance(it, dict) and it.get("name"):
                items.append({"name": _name(it["name"]),
                              "aliases": [_name(a) for a in (it.get("aliases") or []) if a]})
        answers = [str(a) for a in (doc.get("answer_classes") or [])]
        group = doc.get("group_number")
    except Exception as e:                       # noqa: BLE001
        print("✗ count_items 配置读取失败（{}）：{}".format(p, e), flush=True)
        return [], [], None
    return items, answers, group


def apply_to_vision(items, log=print):
    """把清单写进 vision_pipeline 与 insid3_review 的模块级词表（建节点前调用 ✓）。"""
    if not items:
        return None
    names = [it["name"] for it in items]
    alias_map = {it["name"]: (it["aliases"] or [it["name"]]) for it in items}

    import vision_pipeline as vp
    old = list(getattr(vp, "ITEM_NAMES", []))
    vp.ITEM_NAMES = names
    vp.ITEM_ALIASES = alias_map
    vp.TEXT_PROMPT = " . ".join(names) + " ."       # GroundingDINO 的开集 prompt

    reviewed = None
    try:
        import insid3_review as ir
        old_ir = list(getattr(ir, "TARGET_CLASSES", []))
        ir.TARGET_CLASSES = names                    # 闭集复核的保留过滤
        reviewed = (old_ir, names)
    except Exception as e:                           # noqa: BLE001
        log("  ⚠ 没能同步 insid3_review.TARGET_CLASSES（{}）⇒ 复核可能把目标剔掉 ✗".format(e))

    log("  待计数物品已生效: {}".format(names))
    log("    prompt = {!r}".format(vp.TEXT_PROMPT))
    if reviewed:
        log("    vision_pipeline.ITEM_NAMES {} → {}".format(old, names))
        log("    insid3_review.TARGET_CLASSES {} → {}".format(reviewed[0], names))
    # 参考图里没有的类别 ⇒ 复核**叫不出**这个名字（会被叫成最像的那类 ✗），提前点出来
    try:
        import insid3_review as ir
        have = {_key(k) for k in getattr(ir, "INSID3_CLASSES", {})}
        missing = [n for n in names if _key(n) not in have]
        if missing:
            log("  ⚠ 这些类别在闭集参考图里没有（复核会叫错名）: {}".format(missing))
    except Exception:                                # noqa: BLE001
        pass
    return names


def apply_from_config(path=None, log=print):
    """一步到位：读配置 → 同步视觉两处 → 返回 (items, answers, group) 给 patrol_task 用 ✓"""
    items, answers, group = load_items(path)
    if not items:
        log("  ⚠ 没读到待计数清单（{}）⇒ 沿用代码里的默认 4 类".format(path or DEFAULT_CFG))
        return items, answers, group
    apply_to_vision(items, log=log)
    if not answers:
        answers = [it["name"].replace(" ", "_") for it in items]
    log("  答案 JSON 类别: {}".format(answers))
    return items, answers, group


if __name__ == "__main__":
    its, ans, grp = load_items()
    print("配置文件: {}".format(os.environ.get("COUNT_ITEMS_CFG") or DEFAULT_CFG))
    print("待计数物品 ({}):".format(len(its)))
    for it in its:
        print("  {:<18} 别名 {}".format(it["name"], it["aliases"] or "（无，用本名）"))
    print("答案 JSON 类别: {}".format(ans or "（留空 ⇒ 用上面全部，下划线写法）"))
    print("组号: {}".format(grp))
