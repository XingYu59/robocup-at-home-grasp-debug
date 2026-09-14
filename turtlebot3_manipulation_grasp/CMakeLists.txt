cmake_minimum_required(VERSION 3.16)
project(turtlebot3_manipulation_grasp)

# grasp_node = 中间节点：把「视觉目标」翻译成「pick_and_place 的输入」。
# 规划实现通过 GraspPlanner 接口注入（见 include/.../grasp_planner.hpp）。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)                    # 规划实现（pick_and_place）需要
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)  # PlanningSceneInterface
find_package(moveit_task_constructor_core REQUIRED)   # MTC 阶段图
find_package(moveit_task_constructor_msgs REQUIRED)   # MTC 执行 action
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_ros REQUIRED)                        # 目标/支撑面 帧变换
find_package(tf2_eigen REQUIRED)
find_package(ament_index_cpp REQUIRED)                # 定位 config/objects.yaml
find_package(yaml-cpp REQUIRED)                       # 解析物体目录表
find_package(rosidl_default_generators REQUIRED)

# 接口定义 -> 生成 C++ 类型
#   srv/GraspFixedObject.srv    ：抓取触发 + 目标/障碍透传（编排 → grasp_node）
#   srv/DetectGraspTarget.srv   ：按需检测（编排 → 视觉节点），返回 GraspTargetStamped[]
#   msg/GraspTargetStamped.msg  ：视觉目标契约
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GraspFixedObject.srv"
  "srv/DetectGraspTarget.srv"
  "msg/GraspTargetStamped.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# 规划器实现（定义 makePickPlacePlanner）
set(GRASP_PLANNER_SRC src/pick_and_place.cpp)

add_executable(grasp_node src/grasp_node.cpp ${GRASP_PLANNER_SRC})
target_include_directories(grasp_node PUBLIC include)
ament_target_dependencies(grasp_node
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  geometry_msgs
  shape_msgs
  moveit_msgs
  tf2_ros
  tf2_eigen
  ament_index_cpp
)
target_link_libraries(grasp_node
  yaml-cpp
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

install(TARGETS grasp_node DESTINATION lib/${PROJECT_NAME})
# detect_grasp_target_node.py：视觉适配层，提供 /detect_grasp_target 服务
# （实现 msg/GraspTargetStamped.msg + srv/DetectGraspTarget.srv 那份冻结契约）。
# ★ 它必须跑在带 torch/groundingdino/sam2 的虚拟环境里（source ~/venvs/vision_env/bin/activate）。
install(PROGRAMS scripts/detect_grasp_target_node.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
