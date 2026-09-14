// grasp_planner.hpp
//
// 「抓取触发/适配」与「抓取规划」之间的那条缝。
//
// 依赖方向（谁都不许反向依赖）：
//
//     grasp_node.cpp  ──►  grasp_planner.hpp  ◄──  mtc_grasp_planner.cpp
//     （ROS I/O、TF、查表、目标校验）              （唯一 include MoveIt/MTC 的实现）
//
// 本文件刻意【不】include 任何 MoveIt 头，也【不】include rclcpp 头
// （rclcpp::Node 只用前向声明），这样接口层可以独立阅读与单测。
//
// 分工：
//   * 适配器（grasp_node）：srv → GraspTargetStamped → TF/查表/校验 → GraspRequest；
//   * planner（本接口的实现）：GraspRequest → 规划场景 → MTC init/plan/execute → GraspResult。
//     建场景（MoveIt planning scene 的碰撞体）属于 planner 内部职责，因为它只对
//     MoveIt 有意义；适配器交出去的 GraspRequest 已经是纯几何。

#pragma once

#include <cstdint>
#include <memory>

#include <turtlebot3_manipulation_grasp/grasp_types.hpp>

namespace rclcpp {
class Node;  // 前向声明：接口层不引入 rclcpp 头文件
}  // namespace rclcpp

namespace turtlebot3_manipulation_grasp {

/// 抓取规划器。实现必须与调用方（服务回调线程）解耦地完成一次完整抓取。
class GraspPlanner
{
public:
  virtual ~GraspPlanner() = default;

  /// 建场景 → 规划 → 执行；阻塞直到结束（可能几分钟）。
  ///
  /// 约定：
  ///   * 输入 request 的几何量一律已经是 planning_frame 下的值，实现内部不需要再做 TF；
  ///   * 实现不得要求调用方的节点同时被 spin —— MTC 的 execute() 与 CurrentState
  ///     各自会创建并自旋自己的节点（见 moveit_task_constructor 源码），
  ///     所以从服务回调里同步调用是安全的；
  ///   * 任何失败都必须归因到 GraspResult::stage，不要抛异常穿透到 ROS 回调。
  virtual GraspResult run(const GraspRequest& request) = 0;
};

/// 工厂：把实现细节（MoveIt/MTC/参数读取）全部关在实现文件里。
/// node 用于读参数（robot_description / SRDF / 组名 / 帧名 / 命名状态 / 抓取调参）
/// 以及打日志。适配器只 include 本头文件，因此不含任何 MoveIt 符号。
///
/// 实现位置：src/pick_and_place.cpp（MTCTaskNode，阶段图照官方 pick_place demo，
/// 机械臂相关部分换成 FR3）。改换实现只需替换该文件里 makePickPlacePlanner 的定义，
/// 以及 CMakeLists.txt 里 GRASP_PLANNER_SRC 的指向。
std::unique_ptr<GraspPlanner> makePickPlacePlanner(const std::shared_ptr<rclcpp::Node>& node);

}  // namespace turtlebot3_manipulation_grasp
