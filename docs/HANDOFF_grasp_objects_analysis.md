# 抓取集物体清单 + 可抓性分析（fr3_hand 夹爪，开口 0.08 m）

> 数据来源：`turtlebot3_manipulation_grasp/config/objects.yaml`（尺寸已逐条与
> `wpr_simulation_ros2/models/<model>/model.sdf` 的碰撞几何核对：**14/14 完全一致** ✓；
> 网格类 apple/coke_can/bowl/banana 用的是网格包围盒实测值）
> + 夹爪几何实测自 `franka_description/end_effectors/common/franka_hand.xacro`
> + 合爪策略读自 `grasp/src/pick_and_place.cpp`。
> 世界里的落点来自 `wpr_simulation_ros2/worlds/example.world`（逐个 include 解算）。

## 1. 夹爪能力（这是所有判定的基准）

| 量 | 值 | 出处 |
|---|---|---|
| 手指关节 `fr3_finger_joint1/2` | prismatic，行程 **0…0.04 m**，joint2 mimic joint1 | URDF |
| 开口（指尖间距） | **= 2 × 关节值 ∈ [0, 0.08 m]** | URDF |
| 指尖（橡胶块）内侧面 | y ≈ 0（碎块 origin y=7.58mm、size y=15.2mm → 内侧面 −0.02mm） | URDF |
| 滑动座内侧面 | y = 2.4 mm | URDF |
| **掌垫（screw mount）内侧面** | **y = 11.0 mm** | URDF |
| 指尖接触区长度 | ≈ 3.5 cm（从指尖往后） | 目录表注释 |

**推论（很重要）**：掌垫比指尖靠外 11 mm，所以对任何"窄边 ≤ 0.08 m"的物体，
**掌垫永远碰不到东西** —— 这个夹爪在这套物体上**全部是指尖两点夹持**，
摩擦力完全来自橡胶尖的小接触面。这是抓取脆弱的结构性原因。

## 2. 当前合爪策略会给出的过盈量（与物体大小无关）

`pick_and_place.cpp`：`关节目标 = 0.5 × span − close_hand_squeeze(0.007)`（span = min(depth,width)）
→ 指尖间距目标 = `span − 0.014`
→ **无论夹什么，每侧都往里压 7 mm、合计 14 mm**（相对量：罐子 21%、sugar_box 37%）。

对刚性物体，手指**不可能真的压进去 7 mm**；物理上的结果只有三种：
物体被推开/挤飞、被掀翻、或（圆滑物体）在指尖之间打滑。这就是"夹不住/碰倒"的共同来源。
**建议**：把挤压改成**按窄边比例**（例如 `squeeze = clamp(0.15×span, 0.002, 0.006)`），
或至少按类别给不同值 —— 目前一个 0.007 供所有 18 类用。

## 3. 逐类分析（按可靠性排序）

| 类别 | 窄边 min(d,w) | 占开口 | 可跨过开口 | 指尖平面之上剩余侧面 | 判定 | 备注 / 世界位置 |
|---|---|---|---|---|---|---|
| `coke can` | 0.0670 | 84% | ✓ | 22 mm | **可夹 ✓ 首选** | 圆柱、光滑但有高度(0.124)，夹上段稳。客厅 table_0(−2.35,−0.06)、table_3(−3.50,−3.10) |
| `tomato_soup_can` | 0.0660 | 82% | ✓ | 26 mm | **可夹 ✓** | 比可乐罐矮(0.101)，几何同族，最省事的备选。客厅 table_1(−3.70,−2.10) |
| `potted_meat_can` | 0.0500 | 62% | ✓ | 21 mm | **可夹 ✓** | 扁罐头(0.082 高)，窄边 5 cm 居中，指尖行程余量大。**餐厅** table_0(1.90,1.50) |
| `cracker_box` | 0.0600 | 75% | ✓ | 75 mm | 可夹 ✓（高瘦） | 0.21 m 高的薄盒，夹点在离桌面约 13.5 cm 处 → 有掀翻风险。**餐厅** table_2(2.30,1.50) |
| `mustard_bottle` | 0.0580 | 72% | ✓ | 65 mm | 可夹 ✓（高瘦） | 同上，0.19 m 高。**餐厅** table_0(1.10,1.50) |
| `bleach_cleanser` | 0.0650 | 81% | ✓ | 95 mm | 可夹 ✓（高瘦） | 0.25 m 高、窄边 6.5 cm 居中，夹点在 15.5 cm 高。**餐厅** table_1(1.50,2.00) |
| `chips_can` | 0.0750 | 94% | ✓ | 95 mm | 临界（余量小） | 窄边几乎等于有效上限 + 0.25 m 高 → 最不推荐。**餐厅** table_1(1.90,2.00) |
| `sugar_box` | 0.0380 | 48% | ✓ | 57 mm | 可夹但**过盈过大** | 3.8 cm 厚的立式盒，被压 14 mm = 宽度的 37% → 极易挤飞/掀翻。**餐厅** table_3(3.10,2.00) |
| `apple` | 0.0749 | 94% | ✓ | 16 mm | 临界（易滑脱） | 圆、硬、表面光滑 + 窄边贴上限 → 指尖两点最难抓住的一类。客厅 3 处 |
| `banana` | 0.0750 | 94% | ✓ | **8 mm** | 有条件可夹 | **长边 0.198 m 必须跨窄边**（跨长边必然夹不住）；夹点余高只有 8 mm。客厅 table_1(−3.30,−2.10) |
| `gelatin_box` | 0.0730 | 91% | ✓ | **4 mm** | 薄件（易骑顶） | 只有 2.8 cm 高：指尖平面之上只剩 4 mm 侧面，且窄边贴上限。**餐厅** table_2(2.70,1.50) |
| `windex_bottle` | 0.0800 | **100%** | ✓（零余量） | 105 mm | **夹不住 ✗** | 窄边正好等于开口 → 没有任何挤压余量，夹持力为 0。**目录表却标了 `graspable: true` ✗ 建议改 false**。客厅 table_2(0.63,−2.20) |
| `pudding_box` | 0.0890 | 111% | ✗ | 8 mm | 夹不住 ✗ | 窄边 8.9 cm > 开口。**餐厅** table_3(2.30,2.00) |
| `tuna_fish_can` | 0.0850 | 106% | ✗ | 7 mm | 夹不住 ✗ | 客厅 table_1(−2.90,−2.10) |
| `master_chef_can` | 0.1020 | 127% | ✗ | 40 mm | 夹不住 ✗ | **餐厅** table_2(3.10,1.50) |
| `beer` | 0.1100 | 138% | ✗ | 85 mm | 夹不住 ✗ | **餐厅** table_1(1.10,2.00) |
| `pitcher_base` | 0.1080 | 135% | ✗ | 88 mm | 夹不住 ✗ | **餐厅** table_0(1.50,1.50) |
| `bowl` | 0.1574 | 197% | ✗ | 12 mm | 夹不住 ✗ | 碗这类应"夹沿口"，平行夹爪 0.08 m 做不到。**餐厅** table_3(2.672,2.015) |

> "指尖平面之上剩余侧面" = 高度/2 − grasp_lift，即夹点在物体上还剩多少可贴合的高度；
> 越小越容易让指尖直接骑到物体顶面（= 夹空）。

## 4. 对"从餐桌上任选一个抓"的直接意义

按规则书，抓取任务在**餐厅**做（`dinning_table_0..3`）。把上表按位置筛一遍：

| 餐厅桌 | 桌上物体 | 能夹吗（当前目录表状态） |
|---|---|---|
| dinning_table_0 | mustard_bottle、pitcher_base、potted_meat_can | **能夹 2 个**，但都在目录表里被注释掉 → 需取消注释 |
| dinning_table_1 | beer、bleach_cleanser、chips_can | **能夹 2 个**（bleach / chips），同样需取消注释 |
| dinning_table_2 | cracker_box、gelatin_box、master_chef_can | **能夹 2 个**（cracker / gelatin，gelatin 偏薄），需取消注释 |
| dinning_table_3 | **bowl**（已启用但夹不住）、pudding_box（夹不住）、sugar_box（能夹，需取消注释） | 只有 sugar_box 一个候选 |

**结论**：抓取集里"能夹"的类别一共 **10 个**（coke can / tomato_soup_can / potted_meat_can /
cracker_box / mustard_bottle / bleach_cleanser / chips_can / sugar_box / apple / banana），
其中 **7 个在餐厅桌上**、但目前**只有 4 类在目录表里启用**（apple / coke can / bowl / banana，
banana 还是本次迁移时按 HANDOFF 计划打开的）→ 餐厅桌上能直接抓的**一个都没有**。
只要把 `objects.yaml` 里那几行取消注释（尺寸都已核对无误），餐厅桌上立刻就有可抓目标。

## 5. 建议的改动优先级（抓取侧，先不改代码，供拍板）

1. **取消注释** 餐厅桌上那几个可夹类别（`mustard_bottle` / `potted_meat_can` / `bleach_cleanser` /
   `chips_can` / `cracker_box` / `sugar_box`，`gelatin_box` 视情况）—— 零风险、纯配置。
2. **`windex_bottle` 改 `graspable: false`**（窄边 = 开口，零余量，标 true 会骗上层去抓 ✗）。
3. **挤压量改成按窄边比例**（当前恒定 14 mm 过盈，对窄/薄/高瘦物体全是灾难）。
4. **加开口/朝向校验**：`min(width, depth) ≤ 0.075` 必须进规划层；对长条物（banana 0.198）
   强制"跨窄边"；否则 MTC 可能选一个物理上夹不住的朝向。
5. **`apple` / `banana` / `gelatin_box` 视作低优先目标**（都在临界区），编排层排序时放到后面。


---

## 6. 2026-09-13 追加：18 类全部启用（含夹不住的）

**为什么连夹不住的一起启用**：`grasp_node` 对 `obstacles[]` 也查 objects.yaml，
**查不到就 `continue` 静默跳过** ✗（`grasp_node.cpp`：`catalog_.find(ob.class_id)` 失败即跳过）
→ 桌上那些盒子不会进规划场景，机械臂可能一头撞上去。所以"能不能夹"用 `graspable`
表达，而不是靠"不写进表"来回避。视觉侧以后把 prompt 加进 `ITEM_NAMES` 时，
同名 key 已经就绪 ✓

| 状态 | 类别 |
|---|---|
| **可夹（11）** | `apple`、`coke can`、`banana`、`bleach_cleanser`、`chips_can`、`cracker_box`、`gelatin_box`、`mustard_bottle`、`potted_meat_can`、`sugar_box`、`tomato_soup_can` |
| 启用但 `graspable: false`（7） | `bowl`、`beer`、`master_chef_can`、`pitcher_base`、`pudding_box`、`tuna_fish_can`、`windex_bottle` |

> 其中 **`windex_bottle` 从原来的 `true` 改成 `false`**：窄边 0.080 正好等于夹爪开口，
> 零挤压余量、夹持力为 0，标 true 会骗上层去抓一个注定夹不住的目标 ✗
> 18 类的 `graspable` 与判据 `min(depth,width) ≤ 0.075` 逐条复核一致（无矛盾）✓

**启用后餐厅四张桌立刻都有可抓目标了**（之前唯一"已启用"的是夹不住的碗）：

| 餐桌 | 现在可抓的物体 |
|---|---|
| dinning_table_0 | `mustard_bottle`、`potted_meat_can` |
| dinning_table_1 | `bleach_cleanser`、`chips_can` |
| dinning_table_2 | `cracker_box`、`gelatin_box` |
| dinning_table_3 | `sugar_box`（另有 `bowl` / `pudding_box` 只能当障碍）|

编排层的 `choose_target()` 现在**直接读 objects.yaml 的 `graspable`**（不再在代码里维护
第二份名单），并用假场景回归过：碗被跳过、只有方盒时能选、全夹不住/置信度低/超出臂展时正确拒绝 ✓
