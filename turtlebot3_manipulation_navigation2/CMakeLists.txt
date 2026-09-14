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
install(
  PROGRAMS scripts/patrol_task.py scripts/cmd_vel_relay.py scripts/vision_pipeline.py
           scripts/insid3_review.py scripts/dining_grasp_task.py scripts/grasp_phase.py
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
