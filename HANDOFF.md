# 抓取调试交接入口（**先看这份**）

> 本文件是接手调试/继续开发的**唯一入口**；所有交接文档集中在 `docs/handoff/` ✓
> 最近一次更新：2026-09-14

## 一句话现状

整条链（巡逻计数 → 观察位选目标 → 导航站位 → 相对微调 → MTC 抓取 → 放回）**已跑通到 `stage=0`** ✓；
但**物理上真正夹住并抬起来还没有成功过** ✗。固定现象：**夹爪看着包住物体、能闭合，物体却一点不动**。

## 按这个顺序读

| # | 文档 | 内容 |
|---|---|---|
| 1 | **[docs/handoff/HANDOFF_debug_state_2026-09-14.md](docs/handoff/HANDOFF_debug_state_2026-09-14.md)** | **最新进度 + 未解问题与方向 + 工具 + 复现步骤 + 踩过的坑**（最重要 ✓）|
| 2 | [docs/handoff/HANDOFF_grasp_fix_2026-09-14.md](docs/handoff/HANDOFF_grasp_fix_2026-09-14.md) | 抓取几何/规划层的修复记录（抓取高度、对正桌子、去真值）|
| 3 | [docs/handoff/README.md](docs/handoff/README.md) | 交接文档索引（更早的迁移/设计/接口文档）|
| 4 | [HANDOFF_harness/README.md](HANDOFF_harness/README.md) | 调试工具怎么用（几何自检/视觉探针/抓取取证/抓取点扫描）|

## 最关键的一条判据（抓取日志）

```
两指真实间距: 起始 80.0 mm → 最小 X mm
```
- `X ≈ 物体窄边`（如 38 mm / 66 mm）⇒ **手指被挡住 = 夹到了** ✓
- `X = 指令值`（如 29 mm）⇒ **两指之间是空的** ✗ ⇒ 问题在"抓取点的位置/高度"，不要再查别的 ✓

## 复现（四个终端）

```bash
ros2 launch turtlebot3_manipulation_gazebo turtlebot3_franka.launch.py      # 1 仿真
ros2 launch turtlebot3_manipulation_navigation2 navigation.launch.py        # 2 导航
source ~/venvs/vision_env/bin/activate                                      # 3 抓取服务（含视觉）
ros2 launch turtlebot3_manipulation_grasp grasp_service.launch.py
ros2 run turtlebot3_manipulation_navigation2 dining_grasp_task.py           # 4 抓取阶段
#   也可以只抓指定类别：  --classes "tomato_soup_can"
#   车已停好快速验：      --keep-pose --skip-nav
```

## 三个最该先做的事

1. **修视觉的绝对位置精度**（车停住连调 5 次看抖动；最优先 ✗）
2. 用**圆柱**（world 里已把餐桌3 的 pudding_box 换成 `tomato_soup_can` ✓）做干净对照
3. `sugar_box` 的**可视网格比碰撞体胖 11.5 mm** ⚠️ → 跟裁判确认（`docs/handoff/HANDOFF_debug_state_2026-09-14.md` §2.7）
