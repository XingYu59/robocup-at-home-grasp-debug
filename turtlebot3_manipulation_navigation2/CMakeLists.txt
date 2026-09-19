################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_manipulation_navigation2)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)

################################################################################
# Install
################################################################################
install(
  DIRECTORY launch map param rviz
  DESTINATION share/${PROJECT_NAME}
)

# 任务节点 — 可用 ros2 run 执行
# vision_pipeline.py 与 patrol_task.py 装到同一目录，供 patrol_task import
# insid3_review.py 也须同目录安装，否则 vision_pipeline 的 `import insid3_review`
# 失败、闭集复核静默降级（香蕉会被 GroundingDINO 误判成苹果）。
# dining_grasp_task.py 是抓取链的任务层驱动（导航到固定站位 + 相对闭环微调 + 调服务）。
# count_items.py（2026-09-19 新增）也必须一起装：patrol_task 在 main() 开头
# `import count_items` 去同步"待计数物品清单"（三处词表联动，见 docs/handoff/HANDOFF_count_items.md）
# 不装 ⇒ ImportError ⇒ 整条基础题跑不起来 ✗（实测踩过同类：insid3_review 漏装导致复核静默降级）
install(
  PROGRAMS scripts/patrol_task.py scripts/cmd_vel_relay.py scripts/vision_pipeline.py
           scripts/insid3_review.py scripts/dining_grasp_task.py scripts/grasp_phase.py
           scripts/count_items.py
  DESTINATION lib/${PROJECT_NAME}
)

# 待计数物品清单（config/count_items.yaml）也装进 share，方便比赛现场直接改 ✓
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
