# 基础题（视觉识别 + 计数）换物品清单 —— 一处配置，三处词表联动

> 规则书 2.3 / 2.5：裁判**现场发布三个物品英文名**（例：`cola, bowl and windex bottle`），
> 机器人要：识别 → 计数 → 在 RViz `/map` 用 Marker 标出位置 → 终端打印英文名+数量 →
> 写 `<组号>_answer.json`（**类别集合必须与裁判发布完全一致，不能多不能少**）。

## 要改哪些部分（原来散在三处，改漏一处就静默失效 ✗）

| # | 位置 | 作用 | 改漏的后果 |
|---|---|---|---|
| ① | `vision_pipeline.py`：`ITEM_NAMES` / `ITEM_ALIASES` / `TEXT_PROMPT` | GroundingDINO 的**开集 prompt** + `classify_phrase()` 的**归一表** | prompt 里没有的名字根本检不出；检出的也会被归一表丢掉 ✗ |
| ② | `insid3_review.py`：`TARGET_CLASSES` | **闭集复核的保留过滤** | 复核会把目标当干扰剔掉（或叫成最像的类）✗ |
| ③ | `patrol_task.py`：`NAME_TO_JSON` / `TARGET_CLASSES_JSON` / `GROUP_NUMBER` | **答案 JSON** 的键与类别集合、组号 | 类别集合与裁判不一致 ⇒ 不计分 ✗ |

三处都是**模块级常量**（import 时定死）⇒ 必须在建节点/建流水线**之前**改 ✓。

## 现在怎么改（改一个 YAML 就够 ✓）

**`turtlebot3_manipulation_navigation2/config/count_items.yaml`**

```yaml
count_items:                      # 要识别/计数/打 Marker 的物品（= 裁判发布的三个）
  - name: "coke can"              # 视觉词表用名（prompt / 复核标签都用它）
    aliases: ["coke", "coca", "cola", "soda", "can"]   # 裁判可能这么说 ⇒ 都归到本类 ✓
  - name: "bowl"
    aliases: ["bowl"]
  - name: "windex bottle"
    aliases: ["windex", "glass cleaner"]

answer_classes: ["coke_can", "bowl", "windex_bottle"]  # 答案 JSON 的键：照抄裁判写法 ✓
group_number: 7                                        # 答案文件名 <组号>_answer.json
```

然后**重启 patrol_task**（视觉模型 ~10 s；不用改任何代码、不用重建 ✓）。启动时会打印：

```
  待计数物品已生效: ['coke can', 'bowl', 'windex bottle']
    prompt = 'coke can . bowl . windex bottle .'
    vision_pipeline.ITEM_NAMES [...] → [...]
    insid3_review.TARGET_CLASSES [...] → [...]
  答案 JSON 类别: ['coke_can', 'bowl', 'windex_bottle']
视觉流水线就绪，耗时 10.4s；词表 ['coke can', 'bowl', 'windex bottle']     ← 与裁判一致才算改对 ✓
```

## 三条硬约束（踩了就直接丢分）

1. **名字必须在闭集参考图里**：`scripts/reference_views/` 下当前有 18 类
   （apple / banana / beer / bleach_cleanser / bowl / chips_can / coke can / cracker_box /
   gelatin_box / master_chef_can / mustard_bottle / pitcher_base / potted_meat_can /
   pudding_box / sugar_box / tomato_soup_can / tuna_fish_can / windex bottle）。
   写了参考图里没有的类别 ⇒ 复核**叫不出这个名字**（会退化成"最像的那一类"✗）；
   `count_items` 会在启动日志里**点名警告** ✓。
2. **别名要覆盖裁判的用词**：规则书例子用的是 `cola`（不是 `coke can`）⇒ 别名表必须有它 ✓。
3. **答案类别照抄裁判写法**（`coke_can` / `windex_bottle` 这种下划线、大小写）——
   `NAME_TO_JSON` 负责把内部空格写法转过去 ✓。

## 怎么验证（不用等比赛）

```bash
# ① 只打印将生效的清单（不加载模型）
python3 src/turtlebot3_manipulation_navigation2/scripts/count_items.py
# ② 离线自测：三处词表是否同时同步 + 别名命中 + 护栏（16 项）
python3 src/HANDOFF_harness/test_count_items.py
# ③ 实跑：日志首行"词表 [...]"应与 count_items.yaml 完全一致 ✓
```

## 不改变现状的保证

仓库里的默认 `count_items.yaml` 与改动前的硬编码**完全等价**
（`ITEM_NAMES` 4 类 ✓ / `TARGET_CLASSES` 4 类 ✓ / 答案类别 `["apple","coke_can"]` ✓）⇒
不动它时行为与之前一模一样 ✓（`test_count_items.py` 用例⑤ 把这条钉住了 ✓）。
配置文件缺失或写坏 ⇒ 退回代码默认、不崩 ✓。
