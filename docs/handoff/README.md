# 交接文档索引（`docs/handoff/`）

按顺序读 ✓；最新状态永远看第 1 份。

| # | 文档 | 内容 |
|---|---|---|
| 1 | `HANDOFF_closing_axis_and_grasp_height_2026-09-19.md` | **最新**：抓取"停在物体顶上/合爪空合"的两个根因候选 —— 合拢轴 φ（长方体 span(φ)>80 mm 就夹不进去）与高度只剩 0.4 mm 余量；新增 `★ 合拢轴核对` 诊断；**P1 已落地**：激光扫桌腿反解车偏航 → 注入 grasp_node 的 `object_yaw_map`（§8）|
| 2 | `HANDOFF_nav_survey_and_grasp_tolerance_2026-09-18.md` | ①观察点扫视/不再绕桌撞桌（3 个根因 + 离线自检）②抓取接触闭环重抓 + 视觉 2 修复 |
| 3 | `HANDOFF_agent_plan_vision_nav_2026-09-18.md` | 视觉目标选择 + 导航站位执行计划（幻影目标故障链 / P0-P2 清单 / 阶段化验收）|
| 4 | `HANDOFF_debug_state_2026-09-14.md` | 进度 / 未解问题与方向 / 工具 / 复现 / 10 个坑 |
| 5 | `HANDOFF_grasp_fix_2026-09-14.md` | 抓取几何与规划层修复记录（抓取高度、对正桌子、彻底去 gz 真值）|
| 6 | `HANDOFF_grasp_orchestration_design.md` | Phase 2 编排设计（目标优先级、站位/creep、换目标策略）|
| 7 | `HANDOFF_vision_grasp_interface.md` | 视觉↔抓取的冻结接口（msg/srv 语义、坐标系约定）|
| 8 | `HANDOFF_grasp_objects_analysis.md` | 物体目录表分析（尺寸来源、可夹性判据、抓取方式）|
| 9 | `HANDOFF_migration_result.md` | 从旧工作区搬到队友工程的迁移记录与实测结论 |
| 10 | `HANDOFF_agent_plan.md` / `HANDOFF_conversation_summary.md` | 迁移期的计划与对话摘要（历史资料）|

相关但不在本目录：
- 仓库根 `HANDOFF.md` —— **交接总入口**（先看它）
- `HANDOFF_harness/README.md` —— 调试工具说明
- `docs/CURRENT_ARCHITECTURE.md` / `docs/GRASP_SERVICE_HOWTO.md` / `docs/dining-grasp-integration.md` —— 系统架构与服务用法
