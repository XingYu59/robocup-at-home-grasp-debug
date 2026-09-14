cmake_minimum_required(VERSION 3.5)
project(turtlebot3_moveit_config)

find_package(ament_cmake REQUIRED)

# Install config + launch into share/<pkg>
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
