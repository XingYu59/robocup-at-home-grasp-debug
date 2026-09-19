# Agent 执行计划：视觉目标选择 + 导航站位（2026-09-18）

> 接手者从这里开始。本计划基于 2026-09-18 两整次运行的日志证据（路径见 §6），
> 覆盖"视觉报什么目标 → 导航去哪站位"这一整段；抓取物理层（夹持高度/干涉量）
> 见 `HANDOFF_debug_state_2026-09-14.md`，不重复。
>
> **⚠ 今天已回退一次改动**：多帧投票+中位数融合（含两个真 bug 修复），按用户要求
> 已从工作区撤下，完整 patch 存档在 `HANDOFF_harness/patches/multiframe_fusion_2026-09-18.patch`
> （配套离线测试 `test_handle_detect_2026-09-18.py` 同目录）。落地条件见 §4 Phase 2。

---

## 0. 一句话现状

整条链能跑，但**目标选择会被"邻桌幻影"劫持**：观察位隔着餐桌 3 能看到后面餐桌 2 上的
物体，视觉把它们也报出来；支撑面校验被禁用（只提示不拦截）拦不住 → 站位解算为桌外
目标生成"绕到桌子侧后方"的怪站位 → 导航过去后抓取侧才拒绝（BAD_TARGET）→ 换下一个
幻影继续绕。**真目标（番茄罐）反而死在尺寸核对**（1.2 m 处掩码只盖 27 mm/66 mm）。

## 1. 2026-09-18 故障链（全部有日志证据，编号 L*）

运行 2（融合版，`~/.ros/log/python3_12606_*.log` + `python_12428_*.log`）：

| # | 事件 | 证据 |
|---|---|---|
| L1 | 观察位 (2.70,3.15) 视觉返回 4 目标：bowl/sugar_box/**cracker_box/potted_meat_can**——后两个是**餐桌 2**（中心 2.7,1.5）上的物体，隔着餐桌 3 看到 | vision 日志"投票…potted_meat_can 3票" |
| L2 | **真罐子被尺寸核对全帧拒绝**：`tomato_soup_can 尺寸核对不过（帧 2-5）：实测横向 0.027 m 不在 0.046~0.131 m`——1.22 m 处罐子只有 ~29 px，SAM2 掩码只盖住约 40% 宽度 | vision 日志 4 条 WARN |
| L3 | 幻影标签来源：闭集复核把 gelatin_box(2.7,1.5) 误标成 potted_meat_can（0.51）；cracker_box(2.3,1.5) 标签正确 | 复核结果行 + example.world:1363/1358 |
| L4 | `SUPPORT_GATE_ENFORCE=False`（grasp_phase.py:53）→ `_on_support` 直接 return True，**连邻桌排除都没跑** | 源码；e1acaf3 改的（当时它误杀真目标会整轮放弃） |
| L5 | choose_target 圆柱优先选中幻影 potted_meat_can@map(2.951,1.569)（桌外，桌脚印 y∈[1.75,2.25]）→ 站位 = **(3.561,1.569, yaw=π)** ← 用户看到的"导航位点很奇怪" | driver 日志"② 抓取站位…" |
| L6 | 站位上 f0 看到 banana → `objects.yaml: banana graspable: true` → 自动切抓取模式被抑制 → 走默认 4 类词表（不含番茄罐）→ "no detection" | vision 日志"第 2/3 帧（默认(4 类 + 复核)）" |
| L7 | 退回观察位旧估计 + 里程计外推 → 抓取侧支撑面足迹校验 BAD_TARGET → 换 gelatin_box@map(1.82,1.82)（又是桌外，餐桌 1 方向）→ 站位 (1.82,2.36) → 导航失败重试 | driver 日志 |

**结论：怪站位的根因是 L1-L4-L5，与已回退的融合改动无关**（融合当天工作正常：
4 帧中位数、帧间偏差 0-1 mm、无"没有新图"警告）。回退后此链仍会复现 —— Phase 0 先验证这一点。

## 2. 已验证事实（不要重新审计 ✗）

| 环节 | 验证方式 | 结论 |
|---|---|---|
| 机械臂执行 | 指令 vs 指尖轨迹 | ✓ ±2 mm |
| 相机内参 / 深度尺度 / 桌面高度 | fx vs hfov 反算；桌平面校验 0.780±0.0003 | ✓ |
| 相机→base 外参 | 指尖自标定 7px；桌平面斜率 | ✓ |
| 契约点数学（圆柱） | 手算：相机高出桌面 39 mm、低于罐心 11.5 mm、仰角 2.1°，p5+r ≈ 轴心+3 mm | ✓ 掩码干净时无偏 |
| TCP/指腹/目录尺寸 vs SDF | check_grasp_geometry.py | ✓ |
| 多帧融合（已回退） | 2026-09-18 运行 2 实测 | ✓ 功能正常（patch 存档） |
| **已确认的 bug（patch 里带修复）** | ① 投票 3 帧跑同一张图（确定性模型 ⇒ 零信息）；② rclpy 默认回调组互斥 ⇒ 服务回调期间相机订阅停摆 | 需随 Phase 2 重新落地 |

## 3. 问题清单（按优先级）

### P0-A 桌外目标没有"导航前"拦截 → 怪站位（L4/L5）
- **修法**：`choose_target` 把支撑面/邻桌校验从"整轮硬门"改成**逐候选打分**：
  在支撑面矩形（margin 0.30）+ 邻桌脚印内的候选 → 优先；全都不在才考虑落选者，
  且此时打 WARN。**绝不因校验失败放弃整轮**（这正是 e1acaf3 关掉它的原因）。
  另：`--classes` 指定的类别加优先权。
- **验收**：观察位调用后，选中的目标 map 坐标必在本桌脚印 ±0.35 m 内；
  L5 的站位 (3.56,1.57,π) 不再出现。

### P0-B 真罐子死于尺寸核对（L2）
- **修法**（detect_grasp_target_node.py `_size_ok`）：横向跨度下限按**距离**放宽——
  远处（>1 m）掩码天然只盖标签带：min 从 `0.7×narrow` 改为 `max(0.7×narrow, 0.35×narrow×距离因子)`
  或直接对 >1 m 目标用 0.4×narrow；核对失败降级为**置信度惩罚**（×0.7）而不是丢弃，
  交多帧投票/复核去压。二选一先试"惩罚"，改动最小。
- **验收**：观察位（1.15 m）调用里 tomato_soup_can 出现在候选中（conf 随惩罚下降没关系）。

### P0-C banana graspable: true 抑制抓取模式（L6）
- **修法**：`objects.yaml` banana `graspable: false`（两指平行爪夹不住弯曲香蕉；
  且它不在比赛抓取目标里）。顺手复查 4 类计数词表里其它 graspable 标记。
- **验收**：站位上 f0 出现 banana 时日志仍显示"→ 抓取模式"。

### P1-A 多帧融合重新落地（patch 存档，见文首）
- 顺序：先落 **bug 修复部分**（真·新帧 + 回调组 + 解包），跑
  `patches/test_handle_detect_2026-09-18.py`（离线，不起仿真）；再落融合。
- **验收**：服务日志 ① 无"没有新图"警告；② 有"融合[..]: N 帧中位数…帧间最大偏差 X mm"；
  ③ 车静止连调 5 次，散布 ≤ 5 mm（>5 mm 先加 vote_frames=5 / 评估 plane 源，别急着抓）。

### P1-B "地面测距"是死代码
- 相机只高出桌面 39 mm < 80 mm 门槛（`_ground_point` 的 cam_h 检查）→ `position_source=auto`
  下**永远退回掩码法**，独立第二测距从未运行。方向：融合落地后 A/B 一次
  `position_source:=plane`（逐帧噪声交给中位数压），或接受掩码单模态。

### P1-C 近看位停太远
- 实测停在 0.88 m（`align_and_approach dist_tol=0.15` 容差上沿，目标 0.68）→ 罐子 ~50 px
  < 70 px 测距阈值。修法：近看这一趟传 `dist_tol=0.08`，或 `CLOSE_LOOK_DIST` 0.68→0.60。

### P1-D 扫视"固定跑满 3 角度"（用户已表态：不必要 + 角度安排有问题）
- 修法已实现过一次（随 patch 存档）：`SURVEY_YAWS=(0°,-25°,+25°)` 直拍优先、
  扫到即停、只在转过时回正。注意：扫视 ±25° 会把餐桌 1 的物体带进画面
  （chips_can@1.9,2.0 在 ±46° 方位）——P0-A 落地后这是安全的。

### P2（记录在案，不急）
- AMCL map 偏置 ~0.16 m vs 扫视合并 8 cm 阈值（兜底路径可能合错）；
  sugar_box 可视网格比碰撞体胖 11.5 mm（掩码横向偏宽 ~5 mm）；
  深度噪声 7 mm/px（分位数平均后可忽略）。

## 4. 执行阶段（一次一个变量；每阶段先读验收，过了才进下一阶段）

| Phase | 内容 | 验收 | 回退点 |
|---|---|---|---|
| 0 | **基线复跑**（回退后代码，全流程一次，归档日志到 `.run/`） | 确认 L1-L7 是否复现（预期：复现 ⇒ 与融合无关坐实）；不复现 ⇒ 重查 | 无改动 |
| 1 | **目标选择**：P0-C（1 行）→ P0-B → P0-A，各跑一次全流程 | §3 各条验收 | 每条独立 commit/可单独撤销 |
| 2 | **融合重新落地**：P1-A（先 bug 修复、再融合，用存档测试） | §3 P1-A 三条 | patch 反向应用 |
| 3 | **站位质量**：P1-C dist_tol → P1-D 扫视直拍优先（从 patch 挑） | 近看位实测距离 ∈[0.60,0.76] m；扫视日志"正对直拍就有 N 个目标 → 不再转角补扫" | 同上 |
| 4 | **抓取闭环**：真抓 1-2 次，读两指最小间距（≈66 mm=接触 / =指令值=空合）；然后 `contract_range_offset` A/B 一次 0 vs -0.022，接触则归零 | 手指被挡（间距≈窄边） | 参数一行 |
| 5 | **固化**：把本 plan 的验收项并进 `HANDOFF_debug_state`；`.run/` 归档 | — | — |

## 5. 实验纪律（每条都是踩过的坑）

1. **每次实验之间重启仿真**（物体被碰动/碰倒后所有读数作废）；跑前确认罐子立在 (2.3,2.0)。
2. 日志归档：`ros2 launch … 2>&1 | tee src/.run/<名字>.log`；判据行见 §6。
3. **一次只改一个变量**；改完重启对应服务（Python 脚本 install 是软链，免编译 ✓）。
4. 别信 stage=success（逻辑附着）；唯一判据是两指真实间距。
5. 视觉一次调用 20-30 s；调用方超时 ≥150 s。

## 6. 索引

| 项 | 位置 |
|---|---|
| 已回退的融合 patch + 离线测试 | `HANDOFF_harness/patches/multiframe_fusion_2026-09-18.patch` / `test_handle_detect_2026-09-18.py` |
| 2026-09-18 运行日志 | `~/.ros/log/python3_12606_*.log`（driver）/ `python_12428_*.log`（视觉）/ `python3_10022_*.log`+`python_9711_*.log`（上午崩溃那次） |
| 支撑面配置（唯一来源） | `turtlebot3_manipulation_grasp/config/grasp_params.yaml: support_surface.*` |
| 邻桌/桌面常量 | `grasp_phase.py: NEIGHBOR_TABLES / TABLE_CENTER_XYZ / SUPPORT_GATE_ENFORCE(53)` |
| 尺寸核对 | `detect_grasp_target_node.py: _size_ok` |
| 物体真值（写死在 world） | `wpr_simulation_ros2/worlds/example.world`（餐桌3：bowl + tomato_soup_can(2.3,2.0) + sugar_box(3.1,2.0)；餐桌2：cracker_box(2.3,1.5) + gelatin_box(2.7,1.5)…） |
| 复现（四终端） | 见 `HANDOFF.md`；快速验：`dining_grasp_task.py --classes "tomato_soup_can" --keep-pose --skip-nav` |
| 判"夹到没有" | `两指真实间距: 起始 80.0 mm → 最小 X mm`（X≈窄边=夹到 ✓；X=指令值=空合 ✗） |
