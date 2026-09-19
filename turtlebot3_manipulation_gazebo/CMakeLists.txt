################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_manipulation_gazebo)

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)

################################################################################
# Install
################################################################################
install(
  DIRECTORY config gazebo launch ros2_control rviz urdf worlds models meshes
  DESTINATION share/${PROJECT_NAME}
)

# ★ 2026-09-18：finger_mimic_relay.py（把 joint1 的位置转发给 joint2）
#   —— Fortress 不支持 URDF mimic，必须由它补齐两指对称 ✗→✓
install(
  PROGRAMS scripts/finger_mimic_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
