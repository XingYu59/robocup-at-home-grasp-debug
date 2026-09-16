// pick_and_place.cpp —— 真正的抓取规划（GraspPlanner 的实现）
//
// 底子 = 本工程原先那份照着 MTC 官方例程写的 MTCTaskNode；本文件把它补全成
// 完整阶段图，并作为 GraspPlanner 的实现接到 grasp_node 后面。
//
// ══════════ 阶段图（与官方 pick_place demo 一致，未改逻辑）══════════
//   PredicateFilter(CurrentState)「applicability test」
//     → MoveTo「open hand」
//     → Connect「move to pick」
//     → SerialContainer「pick object」
//          approach object → generate grasp pose(+IK) → allow coll.(hand,object)
//          → close hand → attach object → allow coll.(object,support)
//          → lift object → forbid coll.(object,support)
//     → Connect「move to place」
//     → SerialContainer「place object」
//          lower object → generate place pose(+IK) → open hand
//          → forbid coll.(hand,object) → detach object → retreat after place
//     → MoveTo「move home」
//
// ══════════ 机械臂与官方 Panda 例程的差异（逐项对应）══════════
//   官方                        本工程
//   --------------------------  --------------------------------------------
//   "panda_arm"                 "arm"（SRDF 组：chain base_link → fr3_link8）
//   "hand"（组）                "hand"
//   "panda_hand"（抓取帧）      "fr3_hand_tcp"（URDF 固定帧 = 指尖平面）
//                               ⚠ 不是 fr3_link8：两者差 10.34 cm 平移 + 45° yaw，
//                                 用错会让手指在离物体 10 cm 的地方闭合
//   "world"                     "base_footprint"（本工程没有 world 帧；升降/下压方向也用它）
//   "ready" 姿态                "home"
//   圆柱 0.25/0.02              GraspRequest 里的长方体（尺寸按 class_id 查 objects.yaml）
//   写死场景 (0.5,-0.25)        外部传入（视觉检测结果 + 桌面 + 其它物品当障碍）
//   grasp_frame_transform       当前用【顶抓】[0, 0, 0.03, π, 0, 0]：
//                                  手前进轴竖直向下 → 从上方下探
//                                  手指闭合轴水平   → 两指在物体两侧水平合拢
//                                指尖平面停在箱心上方 3cm（= 夹物体上段）；
//                                per-class grasp_lift（objects.yaml）覆盖这个高度。
//                                官方 Panda 用 [0,0,0.1,1.571,0.785,1.571]（0.1 是因为
//                                panda_hand 在指尖后方 10cm，而我们的 tcp 就在指尖平面）。
//                                备选【水平侧抓】见 grasp_params.yaml 注释：位姿本身可达
//                                （/compute_ik 实测能解），但 MTC 的 ComputeIK 对这套姿态
//                                25/25 全拒；工具链修好后再按类别启用（细高物体收益最大）。
//                                启动时会打印"抓取方式自检"（接近方向在物体系里的成分），
//                                改完参数看一眼日志就知道是侧抓还是顶抓。
//   速度缩放默认 0.1             sampling/cartesian 都显式设为 1.0（上一版文档里记的慢就是因为这个）
//
// 输入几何一律已经是规划帧（base_footprint）下的值 —— GraspRequest 由 grasp_node
// 的适配层算好，本文件不做任何 TF。

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <stdexcept>
#include <algorithm>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/introspection.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/task_constructor/task.h>

#include <turtlebot3_manipulation_grasp/grasp_planner.hpp>
#include <turtlebot3_manipulation_grasp/grasp_types.hpp>

using turtlebot3_manipulation_grasp::GraspPlanner;
using turtlebot3_manipulation_grasp::GraspRequest;
using turtlebot3_manipulation_grasp::GraspResult;
using turtlebot3_manipulation_grasp::GraspStage;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_and_place");
namespace mtc = moveit::task_constructor;

namespace {

/// 桌面碰撞体 id（与 allowCollisions(object, support) 里的 support 名字必须一致）
const std::string kTableId = "table";

/// 抓取那条 ComputeIK 阶段的名字 —— 抓取姿态的体检数据（合拢轴、注释里的采样角）
/// 都从它产生的子解里量，见 MTCTaskNode::graspClosingYaw()。
/// ★ 必须与 createTask() 里建那个 stage 时用的名字一致（下面用同一个常量建的）
const char* const kGraspIkStage = "grasp pose IK";

/// "合拢轴正对物体主轴"的判据容差（rad）：采样步长 90° 时对齐角正好落在
/// 0°/90°/180°/270°，实测偏差只有 1e-12 量级；取 1° 是为了容忍浮点/URDF 取整，
/// 同时仍能把 15°/30° 那种"斜着夹"认出来 ✗
constexpr double kAlignedTol = 1.0 * M_PI / 180.0;

/// [x,y,z,r,p,y] → Isometry3d（与官方例程一致：Rx·Ry·Rz）
Eigen::Isometry3d vectorToEigen(const std::vector<double>& v) {
  return Eigen::Translation3d(v[0], v[1], v[2]) *
         Eigen::AngleAxisd(v[3], Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(v[4], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(v[5], Eigen::Vector3d::UnitZ());
}

geometry_msgs::msg::Pose vectorToPose(const std::vector<double>& v) {
  return tf2::toMsg(vectorToEigen(v));
}

/// 一个长方体碰撞体（本工程里桌面/物体/障碍统一用 BOX）
moveit_msgs::msg::CollisionObject makeBox(const std::string& id, const std::string& frame,
                                          const geometry_msgs::msg::Pose& center, double depth, double width,
                                          double height) {
  moveit_msgs::msg::CollisionObject object;
  object.id = id;
  object.header.frame_id = frame;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object.primitives[0].dimensions = { depth, width, height };  // BOX 顺序 = x, y, z
  object.primitive_poses.push_back(center);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

}  // namespace

class MTCTaskNode : public GraspPlanner
{
public:
  explicit MTCTaskNode(const rclcpp::Node::SharedPtr& node) : node_(node) {
    loadParams();
    // move_group 的 ApplyPlanningScene 服务：加场景前先探一下，
    // 免得 move_group 没起来时卡在 PlanningSceneInterface 内部等待里
    apply_scene_client_ = node_->create_client<moveit_msgs::srv::ApplyPlanningScene>(
        "/apply_planning_scene");
  }

  /// GraspPlanner 接口：建场景 → 建任务 → init/plan/execute
  GraspResult run(const GraspRequest& request) override;

  /// 把外部传入的碰撞环境加进规划场景（原 setupPlanningScene，改成入参）
  bool setupPlanningScene(const GraspRequest& request);

  /// 组装阶段图（原 createTask，改成入参 + 参数化）
  mtc::Task createTask(const GraspRequest& request);

  /// 递归体检：把解里的每条子轨迹打成一行日志。
  /// 执行层出的问题（控制器拒收轨迹）靠这行定位是"哪一段、多少点、时间戳对不对"。
  /// 实测踩过：arm_controller 报 "Time between points 0 and 1 is not strictly
  /// increasing, it is 0.000000 and 0.000000"，就是前两个路径点时间戳都是 0。
  void dumpSolution(const mtc::SolutionBase& s, int depth = 0) const {
    if (const auto* seq = dynamic_cast<const mtc::SolutionSequence*>(&s)) {
      for (const auto* sub : seq->solutions())
        if (sub)
          dumpSolution(*sub, depth + 1);
      return;
    }
    const std::string stage = s.creator() ? s.creator()->name() : "?";
    const auto* sub = dynamic_cast<const mtc::SubTrajectory*>(&s);
    if (!sub || !sub->trajectory() || sub->trajectory()->getWayPointCount() == 0) {
      RCLCPP_INFO(LOGGER, "  解轨迹[%d] %-26s <无轨迹>", depth, stage.c_str());
      return;
    }
    const auto& traj = *sub->trajectory();
    const std::size_t n = traj.getWayPointCount();
    bool bad = false;
    for (std::size_t i = 1; i < n; ++i)
      if (traj.getWayPointDurationFromStart(i) <= traj.getWayPointDurationFromStart(i - 1))
        bad = true;
    // GenerateGraspPose 把"采样角"写进注释里（rad），这里顺手打出来 ——
    // 它是"手指到底从哪个方向夹进去"的唯一直接证据 ✓
    const std::string comment = s.comment();
    RCLCPP_INFO(LOGGER, "  解轨迹[%d] %-26s 组=%-6s 点数=%2zu 时间 %.3f→%.3f%s%s", depth, stage.c_str(),
                traj.getGroupName().c_str(), n, traj.getWayPointDurationFromStart(0),
                traj.getWayPointDurationFromStart(n - 1),
                bad ? "   ← 时间戳非严格递增（控制器会拒收！）" : "",
                comment.empty() ? "" : ("  角=" + comment).c_str());
    if (!comment.empty())
      RCLCPP_INFO(LOGGER, "      ↑ 抓取采样角 %.4f rad = %.1f°（绕物体 z 轴；90/270 = 正对薄边）",
                  std::atof(comment.c_str()), std::atof(comment.c_str()) * 180.0 / M_PI);
  }

  /// 兜底：给"没有时间戳"的子轨迹补时间参数化（在 execute() 之前调用）。
  ///
  /// 背景：MTC 的 PipelinePlanner 构造 MoveIt PlanningPipeline 时用的是字符串重载
  /// （只传 request_adapters 的参数名），本版本 MoveIt 不会因此挂上 response
  /// adapters —— 于是【采样规划】出来的轨迹时间戳全为 0：Connect 三段
  /// （move to pick / move to place / move home）全中招，joint_trajectory_controller
  /// 直接拒收："Time between points 0 and 1 is not strictly increasing, it is
  /// 0.000000 and 0.000000"，表现为 stage=6 EXEC_FAILED、错误码 99999。
  /// 上游已知问题：moveit_task_constructor#624 / #330。
  /// （往 ompl_planning.yaml 加 response_adapters 没用——本版本 MoveIt 不读这个键；
  ///   笛卡尔段由 MTC 求解器自己算时间，所以当时只有采样段是坏的。）
  void fixTrajectoryTimes(const mtc::SolutionBase& s) const {
    if (const auto* seq = dynamic_cast<const mtc::SolutionSequence*>(&s)) {
      for (const auto* sub : seq->solutions())
        if (sub)
          fixTrajectoryTimes(*sub);
      return;
    }
    auto* sub = const_cast<mtc::SubTrajectory*>(dynamic_cast<const mtc::SubTrajectory*>(&s));
    if (!sub || !sub->trajectory() || sub->trajectory()->getWayPointCount() < 2)
      return;
    const auto& traj = *sub->trajectory();
    const std::size_t n = traj.getWayPointCount();
    for (std::size_t i = 1; i < n; ++i) {
      if (traj.getWayPointDurationFromStart(i) > traj.getWayPointDurationFromStart(i - 1))
        continue;
      const std::string stage = s.creator() ? s.creator()->name() : "?";
      auto fixed = std::make_shared<robot_trajectory::RobotTrajectory>(traj);
      trajectory_processing::TimeOptimalTrajectoryGeneration totg;
      if (!totg.computeTimeStamps(*fixed, time_param_vel_, time_param_acc_)) {
        RCLCPP_ERROR(LOGGER, "  补时间戳失败: %s", stage.c_str());
        return;
      }
      RCLCPP_INFO(LOGGER, "  补时间戳: %-26s 组=%-6s 点数=%zu 时长 %.3f s", stage.c_str(),
                  fixed->getGroupName().c_str(), fixed->getWayPointCount(),
                  fixed->getWayPointDurationFromStart(fixed->getWayPointCount() - 1));
      sub->setTrajectory(fixed);
      return;
    }
  }

  /// 递归找 kGraspIkStage 那条子解，量两件事：
  ///   · closing_yaw   = 【手指合拢轴】在【物体坐标系】xy 平面里的偏航（rad）
  ///   · comment_angle = 解注释里的采样角（rad，仅用于交叉核对，可能为 NaN）
  /// 返回 false = 这个解里量不到（接口/结构变了）⇒ 调用方退回改动前的行为。
  ///
  /// ★ 为什么不直接读注释里的采样角来挑解（注释确实能读到：ComputeIK 会把上游
  ///   GenerateGraspPose 的注释原样拷过来，compute_ik.cpp:436；dumpSolution 打的
  ///   "抓取采样角"就是它）：
  ///   generate_grasp_pose.cpp:173-186 是这么写的 ——
  ///       double current_angle = 0.0;
  ///       while (...) { AngleAxisd(current_angle, rotation_axis); current_angle += delta;
  ///                     ... trajectory.setComment(std::to_string(current_angle)); }
  ///   也就是注释 = **用完之后的** current_angle = 真实采样角 + angle_delta ✗
  ///   步长 90° 时"注释里的 90°"其实是【真实 0°】：照注释挑"绕接近轴转得最少"的解
  ///   会刚好挑反（挑到多转 90° 的那个）。注释是字符串、又带这个偏移，太容易读错。
  /// ★ 合拢轴是几何量，不受这套约定影响：解的末态就是 ComputeIK 写好的 IK 构型
  ///   （compute_ik.cpp:451-453 把 ik_solutions 的关节角写进末态场景），做正运动学取
  ///   fr3_hand_tcp 的 y 轴即可 —— finger_joint1/2 都是沿 hand 系 y 轴平移的棱柱关节，
  ///   而 tcp 相对 hand 只有 z 向平移、没有旋转（franka_hand.xacro: tcp_rpy='0 0 0'）
  ///   ⇒ TCP 的 y 轴就是手指合拢方向 ✓
  bool graspClosingYaw(const mtc::SolutionBase& s, const geometry_msgs::msg::Quaternion& object_rot,
                       double& closing_yaw, double& comment_angle) const {
    if (const auto* seq = dynamic_cast<const mtc::SolutionSequence*>(&s)) {
      for (const auto* sub : seq->solutions())
        if (sub && graspClosingYaw(*sub, object_rot, closing_yaw, comment_angle))
          return true;
      return false;
    }
    const auto* creator = s.creator();
    if (!creator || creator->name() != kGraspIkStage)
      return false;
    const auto* sub = dynamic_cast<const mtc::SubTrajectory*>(&s);
    if (!sub || !sub->end() || !sub->end()->scene())
      return false;

    comment_angle = std::numeric_limits<double>::quiet_NaN();
    const std::string& comment = s.comment();
    if (!comment.empty()) {
      char* end = nullptr;
      const double v = std::strtod(comment.c_str(), &end);
      if (end && *end == '\0')   // 带尾巴的（"... no IK found"）不算，当没有 ✓
        comment_angle = v;
    }

    bool found = false;
    const Eigen::Isometry3d tcp =
        sub->end()->scene()->getCurrentState().getFrameTransform(hand_frame_, &found);
    if (!found)
      return false;
    Eigen::Quaterniond q;
    tf2::fromMsg(object_rot, q);
    // 物体坐标系里的合拢方向；xy 分量就是"绕接近轴（顶抓时=物体 z）转了多少" ✓
    const Eigen::Vector3d closing_obj = q.toRotationMatrix().transpose() * tcp.linear().col(1);
    closing_yaw = std::atan2(closing_obj.y(), closing_obj.x());
    return true;
  }

  /// 从可行解里挑一个执行：**先看"合拢轴是否正对物体主轴"，再比"绕接近轴转了多少"**。
  ///
  /// 为什么是这个次序（2026-09-16 现场：抓取前腕部无必要地转了近 90°）：
  ///   · 采样角只绕接近轴转手 ⇒ 绕接近轴的偏航就是"多余转腕"的量。圆柱（d≈w，
  ///     如 tomato_soup_can 0.066×0.066）各角度物理等价，被挑中的那个要转 90° 就是白转 ✗
  ///     采样集本来就是 {0°,90°,180°,270°}（GenerateGraspPose 从 current_angle=0 起扫），
  ///     0° 一直在候选集里 —— 它不是"没进候选集"，而是被 MTC 的代价（IK 关节距离）
  ///     挤掉了 ⇒ 这里把"旋转量"显式加进选解偏好即可，不用改采样步长 ✓
  ///   · 但"转得少"不能凌驾于"正对盒面"：薄盒（0.038×0.089）与薄边偏 ±30° 以内时
  ///     投影 77 mm < 开口 80 mm，MTC 也判可行 ⇒ 只看旋转量会把 15°/30° 斜夹挑出来 ✗
  ///     （那正是 angle_delta 从 15° 改回 90° 之前修掉的现场故障）
  ///   ⇒ 先按"对齐"分组（|合拢轴偏离最近的 90° 整数倍| ≤ kAlignedTol），组内再比旋转量；
  ///     同样旋转量时按 MTC 代价取便宜的（solutions() 本身按代价升序，先到先得 ✓）
  mtc::SolutionBaseConstPtr pickSolution(const geometry_msgs::msg::Quaternion& object_rot) const {
    const auto& solutions = task_.solutions();
    mtc::SolutionBaseConstPtr best;
    bool best_aligned = false;
    double best_rot = 0.0;
    std::size_t i = 0;
    for (const auto& s : solutions) {
      ++i;
      double yaw = 0.0, cang = 0.0;
      if (!s || !graspClosingYaw(*s, object_rot, yaw, cang)) {
        RCLCPP_WARN(LOGGER, "  解候选 %zu/%zu: 解里找不到 \"%s\" 子解 ⇒ 量不到合拢轴，跳过",
                    i, solutions.size(), kGraspIkStage);
        continue;
      }
      const double align = std::abs(std::remainder(yaw, M_PI / 2));      // 偏离物体主轴多少
      const double rot = std::abs(std::remainder(yaw + M_PI / 2, M_PI)); // 绕接近轴转了多少
      const bool aligned = align <= kAlignedTol;
      const std::string cang_txt = std::isnan(cang)
                                       ? std::string("   (无)")
                                       : std::to_string(cang * 180.0 / M_PI) + "°";
      RCLCPP_INFO(LOGGER,
                  "  解候选 %zu/%zu: 代价 %8.3f 合拢轴(物体系) %+7.1f° 注释角 %s "
                  "→ 偏离主轴 %4.1f°%s 绕接近轴 %5.1f°",
                  i, solutions.size(), s->cost(), yaw * 180.0 / M_PI, cang_txt.c_str(),
                  align * 180.0 / M_PI, aligned ? " ✓正对" : " ✗斜夹", rot * 180.0 / M_PI);
      if (!best || (aligned && !best_aligned) ||
          (aligned == best_aligned && rot < best_rot - 1e-9)) {
        best = s;
        best_aligned = aligned;
        best_rot = rot;
      }
    }
    return best;
  }

private:
  rclcpp::Node::SharedPtr node_;
  mtc::Task task_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_client_;

  // ── 规划层参数（与 config/grasp_params.yaml 同名）────────────────────
  std::string arm_group_name_{ "arm" };
  std::string eef_name_{ "hand" };
  std::string hand_group_name_{ "hand" };
  std::string hand_frame_{ "fr3_hand_tcp" };
  std::string hand_open_pose_{ "open" };
  std::string hand_close_pose_{ "close" };
  std::string arm_home_pose_{ "home" };
  std::string world_frame_{ "base_footprint" };
  std::vector<double> place_pose_{ 0.45, 0.15, 0.78, 0.0, 0.0, 0.0 };
  bool place_back_at_target_{ true };   ///< true = 放回抓取点；false = 用 place_pose 固定点
  double place_surface_offset_{ 0.0001 };
  std::vector<double> grasp_frame_transform_{ 0.0, 0.0, 0.03, M_PI, 0.0, 0.0 };
  double angle_delta_{ M_PI / 12.0 };
  double approach_min_{ 0.10 }, approach_max_{ 0.15 };
  double lift_min_{ 0.05 }, lift_max_{ 0.10 };
  double lower_min_{ 0.03 }, lower_max_{ 0.13 };
  double retreat_min_{ 0.12 }, retreat_max_{ 0.25 };
  int max_solutions_{ 10 };
  // MTC ComputeIK 的 IK 预算。注意这是【找 max_ik_solutions 个解的总预算】，
  // 不是每次调用的超时：kinematics.yaml 里 kinematics_solver_timeout=0.05 会让
  // 它只有 50ms 去找 8 个解 —— 侧抓这类"手腕构型不寻常"的位姿就找不到解
  // （实测：同样位姿用 move_group 的 /compute_ik 服务能解出来）。
  double ik_timeout_{ 0.5 };
  int max_ik_solutions_{ 8 };
  // 合爪时比物体窄边再多挤进去多少（m）：既保证控制器能到位（误差 ≈ 这个值，
  // 远小于 GripperActionController 默认 goal_tolerance 0.01），又提供夹持力。
  double close_hand_squeeze_{ 0.002 };
  /// 合爪速度缩放：1.0 = 用满关节速度上限。太大是"冲击式"合爪，会把轻物体挤飞。
  double close_hand_speed_scaling_{ 0.15 };
  // 兜底时间参数化（fixTrajectoryTimes）的缩放系数：与 sampling_planner 的
  // setMaxVelocityScalingFactor/setMaxAccelerationScalingFactor 保持一致（都是 1.0）。
  double time_param_vel_{ 1.0 };
  double time_param_acc_{ 1.0 };

  template <typename T>
  T paramOr(const std::string& name, const T& fallback) {
    if (!node_->has_parameter(name))
      node_->declare_parameter<T>(name, fallback);
    return node_->get_parameter(name).get_value<T>();
  }

  void loadParams() {
    arm_group_name_ = paramOr<std::string>("arm_group_name", arm_group_name_);
    eef_name_ = paramOr<std::string>("eef_name", eef_name_);
    hand_group_name_ = paramOr<std::string>("hand_group_name", hand_group_name_);
    hand_frame_ = paramOr<std::string>("hand_frame", hand_frame_);
    hand_open_pose_ = paramOr<std::string>("hand_open_pose", hand_open_pose_);
    hand_close_pose_ = paramOr<std::string>("hand_close_pose", hand_close_pose_);
    arm_home_pose_ = paramOr<std::string>("arm_home_pose", arm_home_pose_);
    world_frame_ = paramOr<std::string>("world_frame", world_frame_);
    place_pose_ = paramOr<std::vector<double>>("place_pose", place_pose_);
    place_back_at_target_ = paramOr<bool>("place_back_at_target", place_back_at_target_);
    place_surface_offset_ = paramOr<double>("place_surface_offset", place_surface_offset_);
    grasp_frame_transform_ =
        paramOr<std::vector<double>>("grasp_frame_transform", grasp_frame_transform_);
    angle_delta_ = paramOr<double>("angle_delta", angle_delta_);
    approach_min_ = paramOr<double>("approach_object_min_dist", approach_min_);
    approach_max_ = paramOr<double>("approach_object_max_dist", approach_max_);
    lift_min_ = paramOr<double>("lift_object_min_dist", lift_min_);
    lift_max_ = paramOr<double>("lift_object_max_dist", lift_max_);
    lower_min_ = paramOr<double>("lower_object_min_dist", lower_min_);
    lower_max_ = paramOr<double>("lower_object_max_dist", lower_max_);
    retreat_min_ = paramOr<double>("retreat_min_dist", retreat_min_);
    retreat_max_ = paramOr<double>("retreat_max_dist", retreat_max_);
    max_solutions_ = paramOr<int>("max_solutions", max_solutions_);
    ik_timeout_ = paramOr<double>("ik_timeout", ik_timeout_);
    max_ik_solutions_ = paramOr<int>("max_ik_solutions", max_ik_solutions_);
    close_hand_squeeze_ = paramOr<double>("close_hand_squeeze", close_hand_squeeze_);
    close_hand_speed_scaling_ = paramOr<double>("close_hand_speed_scaling", close_hand_speed_scaling_);
    // 兜底时间参数化用的缩放（与 sampling_planner 的 setMax*ScalingFactor 保持一致）
    time_param_vel_ = paramOr<double>("time_param_velocity_scaling", time_param_vel_);
    time_param_acc_ = paramOr<double>("time_param_acceleration_scaling", time_param_acc_);

    RCLCPP_INFO(LOGGER, "规划参数: group=%s eef=%s hand=%s ik_frame=%s world=%s",
                arm_group_name_.c_str(), eef_name_.c_str(), hand_group_name_.c_str(), hand_frame_.c_str(),
                world_frame_.c_str());
    RCLCPP_INFO(LOGGER, "  命名状态: %s / %s / %s；放置点 [%.3f, %.3f, %.3f]（%d 个解上限）",
                hand_open_pose_.c_str(), hand_close_pose_.c_str(), arm_home_pose_.c_str(), place_pose_[0],
                place_pose_[1], place_pose_[2], max_solutions_);

    // 抓取方式自检：把 grasp_frame_transform 的旋转换算成
    // "手的前进轴(+z)在物体坐标系里的朝向" = R_g^T 的第三列 = R_g 的第三行。
    // z 分量接近 0 → 水平侧抓；接近 ±1 → 顶抓/底抓。
    const Eigen::Matrix3d Rg = vectorToEigen(grasp_frame_transform_).rotation();
    const Eigen::Vector3d approach = Rg.transpose() * Eigen::Vector3d::UnitZ();
    RCLCPP_INFO(LOGGER, "抓取方式自检: 接近方向(物体坐标系) = (%.3f, %.3f, %.3f) → %s", approach.x(),
                approach.y(), approach.z(),
                std::abs(approach.z()) < 0.3 ? "水平侧抓（手指从侧面水平进入）" : "竖直抓（顶抓 / 底抓）");
    RCLCPP_INFO(LOGGER, "IK 预算: timeout=%.2fs (最多 %d 个解)", ik_timeout_, max_ik_solutions_);
  }
};

bool MTCTaskNode::setupPlanningScene(const GraspRequest& request) {
  // 先确认 move_group 在（它的 ApplyPlanningScene 服务可用），否则立刻失败：
  // move_group 没起来时 PlanningSceneInterface 内部会长时间等待，表现为"服务调用卡住"。
  if (!apply_scene_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(LOGGER, "move_group 的 /apply_planning_scene 服务不可用（move_group 没启动？）");
    return false;
  }

  // 等 ApplyPlanningScene 服务就绪（官方 setupDemoScene 也会先等一下）
  rclcpp::sleep_for(std::chrono::microseconds(100));
  moveit::planning_interface::PlanningSceneInterface psi;

  if (request.support.enabled) {
    if (!psi.applyCollisionObject(makeBox(kTableId, request.planning_frame, request.support.pose,
                                          request.support.length, request.support.width,
                                          request.support.thickness))) {
      RCLCPP_ERROR(LOGGER, "加入桌面碰撞体失败");
      return false;
    }
    RCLCPP_INFO(LOGGER, "  场景: 桌面 (%.3f x %.3f x %.3f) @ (%.3f, %.3f, %.3f)", request.support.length,
                request.support.width, request.support.thickness, request.support.pose.position.x,
                request.support.pose.position.y, request.support.pose.position.z);
  }

  if (!psi.applyCollisionObject(makeBox(request.object_id, request.planning_frame, request.object_box_center,
                                        request.object.depth, request.object.width, request.object.height))) {
    RCLCPP_ERROR(LOGGER, "加入目标碰撞体失败");
    return false;
  }
  RCLCPP_INFO(LOGGER, "  场景: 目标 \"%s\" (h %.4f d %.4f w %.4f) @ (%.3f, %.3f, %.3f)",
              request.object.class_id.c_str(), request.object.height, request.object.depth,
              request.object.width, request.object_box_center.position.x,
              request.object_box_center.position.y, request.object_box_center.position.z);

  for (const auto& obstacle : request.obstacles) {
    if (!psi.applyCollisionObject(makeBox(obstacle.id, request.planning_frame, obstacle.box_center,
                                          obstacle.model.depth, obstacle.model.width,
                                          obstacle.model.height))) {
      RCLCPP_WARN(LOGGER, "加入障碍物 %s 失败（继续）", obstacle.id.c_str());
      continue;
    }
    RCLCPP_INFO(LOGGER, "  场景: 障碍 \"%s\"(%s) @ (%.3f, %.3f, %.3f)", obstacle.id.c_str(),
                obstacle.model.class_id.c_str(), obstacle.box_center.position.x,
                obstacle.box_center.position.y, obstacle.box_center.position.z);
  }

  // 给 move_group 一点时间把场景广播出去（MTC 的 CurrentState 会去 get_planning_scene）
  rclcpp::sleep_for(std::chrono::milliseconds(200));
  return true;
}

mtc::Task MTCTaskNode::createTask(const GraspRequest& request) {
  const std::string& object = request.object_id;
  const std::string support_link = kTableId;

  mtc::Task task;
  task.stages()->setName("pick and place: " + request.object.class_id);
  task.loadRobotModel(node_);

  // ── 任务属性（官方例程同款；组名/帧名来自参数）──────────────────────
  task.setProperty("group", arm_group_name_);
  task.setProperty("eef", eef_name_);
  task.setProperty("hand", hand_group_name_);
  task.setProperty("hand_grasping_frame", hand_frame_);
  task.setProperty("ik_frame", hand_frame_);

  // ── 规划器 ────────────────────────────────────────────────────────
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  sampling_planner->setProperty("goal_joint_tolerance", 1e-5);
  sampling_planner->setMaxVelocityScalingFactor(1.0);      // 官方默认 0.1，太慢
  sampling_planner->setMaxAccelerationScalingFactor(1.0);

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  // 合爪单独用一个放慢的插值规划器：实测 0.4s 走完全程属"冲击式"，会把罐子挤出去
  // （时间线取证：合爪瞬间罐子被推走 1.6cm，手指仍合到命令值 → 没夹住）。
  auto close_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  close_planner->setMaxVelocityScalingFactor(close_hand_speed_scaling_);
  RCLCPP_INFO(LOGGER, "  合爪参数: 干涉 %.4f m，速度缩放 %.2f（张开仍用 1.0）", close_hand_squeeze_,
              close_hand_speed_scaling_);

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  // ── 抓取帧偏置：姿态用参数，抓取高度按类别覆盖 ──────────────────────
  // 顶抓（rpy = (π,0,0)）时手的前进轴竖直向下，所以"沿指尖方向的抬升量"
  // 对应平移量的 z 分量（下标 2）：指尖平面停在箱心上方 grasp_lift 处，
  // 手指因此夹住物体的上段。每类 objects.yaml:grasp_lift 覆盖它。
  //
  // ★ 为什么必须在这里做一道校验（2026-09-14 实测故障）：
  //   MTC 的 ComputeIK 在**算 IK 之前**就把手按目标位姿摆好做碰撞检查
  //   （compute_ik.cpp: isTargetPoseCollidingInEEF），手和物体一相交，25 个采样角
  //   全部直接被判 `eef in collision: fr3_hand - object`，连 IK 都不试 ✗
  //   而 hand 的碰撞网格只伸到【指尖平面上方 37.4 mm】，物体顶面要是高过它，
  //   物体顶段就插进手掌 → 无论视觉多准、位置多对，规划**必然** 0/25 失败。
  //   下面这几个数由 franka_hand.xacro 实测（HANDOFF_harness/check_grasp_geometry.py
  //   会用 URDF 重新量一遍，并逐采样角做精确碰撞检测）。
  constexpr double kPalmAboveTcp = 0.0374;   // fr3_hand 碰撞网格最低可及处
  constexpr double kTipAboveTcp = 0.0090;    // 指腹（橡胶指尖）上沿
  constexpr double kTipBelowTcp = 0.0095;    // 指腹下沿（决定"别插进桌面"）
  constexpr double kTableMargin = 0.005;     // 指尖离桌面的最小余量
  constexpr double kGripDepth = 0.023;       // 未配置时的默认夹持深度（从物体顶面往下）
  std::vector<double> grasp_transform = grasp_frame_transform_;
  if (grasp_transform.size() < 6)
    grasp_transform = { 0.0, 0.0, 0.03, M_PI, 0.0, 0.0 };
  {
    const double h2 = 0.5 * request.object.height;
    const double lo = std::max(h2 - kPalmAboveTcp,      // 物体顶面别顶到手掌
                               kTableMargin + kTipBelowTcp - h2);  // 指尖别插进桌面
    const double hi = h2 - kTipAboveTcp;
    double lift = request.object.grasp_lift;
    if (!(lift > 0.0)) {   // 目录表没给（或给了 0）→ 按几何自己算
      lift = h2 - kGripDepth;
      RCLCPP_INFO(LOGGER, "  objects.yaml 没给 \"%s\" 的 grasp_lift → 按几何算 %.4f",
                  request.object.class_id.c_str(), lift);
    }
    if (lo > hi) {
      RCLCPP_WARN(LOGGER, "  \"%s\" 高 %.3f m：夹持高度窗口为空（物体太矮：指尖会插进桌面）——"
                          "只能尽量靠近上沿",
                  request.object.class_id.c_str(), request.object.height);
    } else if (lift < lo - 1e-9 || lift > hi + 1e-9) {
      RCLCPP_WARN(LOGGER,
                  "  ★ \"%s\" 的 grasp_lift=%.4f 不在可行窗口 [%.4f, %.4f] 内 → "
                  "MTC 的 ComputeIK 很可能 0/25 全判 `fr3_hand - object` 碰撞 ✗；"
                  "建议 %.4f（详见 HANDOFF_harness/check_grasp_geometry.py）",
                  request.object.class_id.c_str(), lift, lo, hi,
                  lo <= hi ? 0.5 * (lo + hi) : h2 - kGripDepth);
    } else {
      RCLCPP_INFO(LOGGER, "  抓取高度: \"%s\" h=%.3f → lift=%.4f（可行窗口 [%.4f, %.4f] ✓）",
                  request.object.class_id.c_str(), request.object.height, lift, lo, hi);
    }
    grasp_transform[2] = lift;
  }

  /******************************************************
   *                     Current State                  *
   *****************************************************/
  mtc::Stage* initial_state_ptr = nullptr;
  {
    auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
    auto filter = std::make_unique<mtc::stages::PredicateFilter>("applicability test", std::move(current_state));
    filter->setPredicate([object](const mtc::SolutionBase& s, std::string& comment) {
      if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
        comment = "object with id '" + object + "' is already attached and cannot be picked";
        return false;
      }
      return true;
    });
    initial_state_ptr = filter.get();
    task.add(std::move(filter));
  }

  /******************************************************
   *                     Open Hand                      *
   *****************************************************/
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage->setGroup(hand_group_name_);
    stage->setGoal(hand_open_pose_);
    initial_state_ptr = stage.get();  // 给抓取位姿生成器做 monitored stage
    task.add(std::move(stage));
  }

  /******************************************************
   *                     Move to Pick                   *
   *****************************************************/
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move to pick", mtc::stages::Connect::GroupPlannerVector{ { arm_group_name_, sampling_planner } });
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  /******************************************************
   *                     Pick Object                    *
   *****************************************************/
  mtc::Stage* pick_stage_ptr = nullptr;
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "hand", "group", "ik_frame" });

    // approach object
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame_);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(approach_min_, approach_max_);
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame_;
      vec.vector.z = 1.0;  // 沿指尖方向前进
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    // generate grasp pose + IK
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose(hand_open_pose_);
      stage->setObject(object);
      stage->setAngleDelta(angle_delta_);
      stage->setMonitoredStage(initial_state_ptr);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>(kGraspIkStage, std::move(stage));
      wrapper->setMaxIKSolutions(static_cast<uint32_t>(max_ik_solutions_));
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setProperty("timeout", ik_timeout_);  // IK 总预算，默认 0.5s（见头文件说明）
      wrapper->setIKFrame(vectorToEigen(grasp_transform), hand_frame_);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    // allow collision (hand, object)：与官方例程一致，放在 IK 之后、close 之前
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions(
          object,
          task.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(),
          true);
      grasp->insert(std::move(stage));
    }

    // close hand —— 合爪目标按【物体实际宽度】算，不用命名状态 "close"（= 全闭 0.0）。
    //
    // 为什么必须改：真机上手指会被物体挡住。命令 0.0 时手指停在物体半宽处
    // （罐子 0.0335 m），位置误差远大于 GripperActionController 的默认
    // goal_tolerance(0.01) → 动作 abort → 表现成"规划全对、一到合爪就失败"。
    // 改成"窄一点点挤进去"：手指贴住物体即到位（误差 ≈ squeeze），同时有夹持力。
    // 物体宽度未知时退回命名状态，不会比原来更糟。
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", close_planner);
      stage->setGroup(hand_group_name_);
      std::map<std::string, double> close_goal;
      const auto& obj = request.object;
      const double span = std::min(obj.width, obj.depth);   // 取窄边：保证手指一定触到物体
      if (span > 0.0 && task.getRobotModel()) {
        const auto* jmg = task.getRobotModel()->getJointModelGroup(hand_group_name_);
        // 干涉量 = close_hand_squeeze（grasp_params.yaml 一个参数说了算，便于现场试）。
        //   历史上试过两版，都留在这里免得再踩：
        //     · 0.007（7 mm/指）：对 38 mm 厚的 sugar_box = 每根手指压进去 7 mm，
        //       现场看到"盒子被挤走 / 手指在空档里合拢" ✗
        //     · 0.08×窄边（约 3 mm/指）：现场仍报"闭合不够、贴着盒子上去" ✗
        //   现在按用户判断反向试：**让它合得更少**（当前 0.001 → 两指停在
        //   "物体窄边 − 2 mm"），也就是手指刚好贴到盒面、几乎不挤压 ✓
        const double squeeze = close_hand_squeeze_;
        const double target = 0.5 * span - squeeze;
        double clamped_min = target, clamped_max = target;
        if (jmg) {
          for (const auto& jname : jmg->getActiveJointModelNames()) {
            const auto* jm = task.getRobotModel()->getJointModel(jname);
            if (!jm || jm->getVariableCount() != 1)
              continue;
            const auto& lim = jm->getVariableBoundsMsg();   // 已确保单变量关节
            double v = target;
            if (!lim.empty() && lim[0].has_position_limits)
              v = std::max(lim[0].min_position, std::min(lim[0].max_position, v));
            clamped_min = std::min(clamped_min, v);
            clamped_max = std::max(clamped_max, v);
            close_goal[jname] = v;
          }
        }
        if (close_goal.empty()) {
          RCLCPP_WARN(LOGGER, "  合爪: 拿不到手部关节，退回命名状态 \"%s\"", hand_close_pose_.c_str());
        } else {
          RCLCPP_INFO(LOGGER, "  合爪目标: 物体 %.4f m（窄边 min(%.4f, %.4f)）− 干涉 %.4f "
                              "→ 关节 %.4f [%zu 个关节，实际 %.4f~%.4f]",
                      span, obj.width, obj.depth, squeeze, target, close_goal.size(),
                      clamped_min, clamped_max);
          if (clamped_max > target + 1e-9)
            RCLCPP_WARN(LOGGER, "  合爪目标被关节上限截断：物体可能比夹爪开口（0.08 m）还宽，夹不住");
        }
      }
      if (close_goal.empty())
        stage->setGoal(hand_close_pose_);
      else
        stage->setGoal(close_goal);
      grasp->insert(std::move(stage));
    }

    // allow collision (object, support) —— 注意：必须放在 attach 之前。
    // 官方例程把这一步放在 attach 之后，因为官方场景里物体与桌面不接触；
    // 我们的物体就【坐在】支撑面上，attach 会做一次整体状态校验，
    // 物体-桌面只要有一点点侵入就会失败（实测：配置桌面顶比罐底高 2.6mm →
    // attach object 直接报 "table colliding with object"，0/104）。
    // 适配器已把支撑面顶面对齐到目标点 z（grasp_node 步骤 5b），这里再加一道保险。
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (object,support)");
      stage->allowCollisions({ object }, { support_link }, true);
      grasp->insert(std::move(stage));
    }

    // attach object
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(object, hand_frame_);
      grasp->insert(std::move(stage));
    }

    // lift object
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(lift_min_, lift_max_);
      stage->setIKFrame(hand_frame_);
      stage->properties().set("marker_ns", "lift_object");
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = world_frame_;
      vec.vector.z = 1.0;  // 竖直向上
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    // forbid collision (object, support)
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (object,surface)");
      stage->allowCollisions({ object }, { support_link }, false);
      grasp->insert(std::move(stage));
    }

    pick_stage_ptr = grasp.get();
    task.add(std::move(grasp));
  }

  /******************************************************
   *                     Move to Place                  *
   *****************************************************/
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move to place", mtc::stages::Connect::GroupPlannerVector{ { arm_group_name_, sampling_planner } });
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  /******************************************************
   *                     Place Object                    *
   *****************************************************/
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "hand", "group" });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "hand", "group" });

    // lower object
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lower object", cartesian_planner);
      stage->properties().set("marker_ns", "lower_object");
      stage->properties().set("link", hand_frame_);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(lower_min_, lower_max_);
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = world_frame_;
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    // generate place pose + IK
    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "ik_frame" });
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject(object);

      // 放置点：默认【放回原处】—— 规则书的要求是"清楚地抬起，再放回去"，
      // 不是搬到桌上别的地点。所以 x,y 直接用目标箱心在规划帧下的坐标，
      // z 用它的支撑面高度（= 箱心 z − height/2，正是抓取时那个点的高度）。
      // 想搬去固定点调试就设 place_back_at_target: false 并给 place_pose。
      std::vector<double> place_pt = place_pose_;
      if (place_back_at_target_) {
        place_pt[0] = request.object_box_center.position.x;
        place_pt[1] = request.object_box_center.position.y;
        place_pt[2] = request.object_box_center.position.z - 0.5 * request.object.height;
      }
      geometry_msgs::msg::PoseStamped p;
      p.header.frame_id = request.planning_frame;  // place pose 在规划帧下给
      p.pose = vectorToPose(place_pt);
      // ★ 放置姿态必须和抓取时一样带上物体朝向（原本是 identity = 对齐车体系）：
      //   否则 GeneratePlacePose 会把物体"转"到车体系朝向再放下 ——
      //   对 0.038 × 0.089 这种薄盒就是"放下时手指横过来"，实测会把盒子碰倒 ✗
      p.pose.orientation = request.object_box_center.orientation;
      p.pose.position.z += 0.5 * request.object.height + place_surface_offset_;
      RCLCPP_INFO(LOGGER, "  放置点(规划帧)= (%.3f, %.3f, %.3f) 支撑面 z=%.3f %s", p.pose.position.x,
                  p.pose.position.y, p.pose.position.z, place_pt[2],
                  place_back_at_target_ ? "[放回原处]" : "[固定点 place_pose]");
      stage->setPose(p);
      stage->setMonitoredStage(pick_stage_ptr);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setIKFrame(vectorToEigen(grasp_transform), hand_frame_);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    // open hand
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name_);
      stage->setGoal(hand_open_pose_);
      place->insert(std::move(stage));
    }

    // forbid collision (hand, object)
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions(object, *task.getRobotModel()->getJointModelGroup(hand_group_name_), false);
      place->insert(std::move(stage));
    }

    // detach object
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(object, hand_frame_);
      place->insert(std::move(stage));
    }

    // retreat after place
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat after place", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(retreat_min_, retreat_max_);
      stage->setIKFrame(hand_frame_);
      stage->properties().set("marker_ns", "retreat");
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame_;
      vec.vector.z = -1.0;  // 沿指尖反方向退回
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  /******************************************************
   *                     Move to Home                    *
   *****************************************************/
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move home", sampling_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal(arm_home_pose_);
    stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
    task.add(std::move(stage));
  }

  return task;
}

GraspResult MTCTaskNode::run(const GraspRequest& request) {
  GraspResult result;

  // 1) 场景
  if (!setupPlanningScene(request)) {
    result.stage = GraspStage::SCENE_FAILED;
    result.message = "把碰撞环境加入规划场景失败（move_group 未就绪？）";
    return result;
  }

  // 2) 建任务
  task_ = createTask(request);

  // 3) init
  try {
    task_.init();
  } catch (const mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(LOGGER, "任务初始化失败:\n" << e);
    result.stage = GraspStage::INIT_FAILED;
    result.message = "MTC 任务初始化失败（组名/命名状态/IK 帧等配置问题，详见日志）";
    return result;
  }

  // 4) plan
  const std::size_t max_solutions = max_solutions_ > 0 ? static_cast<std::size_t>(max_solutions_) : 1u;
  RCLCPP_INFO(LOGGER, "开始搜索任务解（max %zu）", max_solutions);
  if (!task_.plan(max_solutions)) {
    RCLCPP_ERROR(LOGGER, "任务规划失败");
    result.stage = GraspStage::NO_SOLUTION;
    result.message = "MTC 未找到可行解（规划失败）";
    return result;
  }
  // ★ 选解：不再无脑取"代价最小"的第一个解，而是优先"合拢轴正对物体主轴、且绕接近轴
  //   转得最少"的那个（见 pickSolution 的说明）。量不到合拢轴时退回原行为（取第一个）✓
  mtc::SolutionBaseConstPtr chosen = pickSolution(request.object_box_center.orientation);
  if (!chosen) {
    chosen = task_.solutions().front();
    RCLCPP_WARN(LOGGER, "所有解都量不到合拢轴 → 退回改动前的行为：执行代价最小的那个");
  } else {
    double yaw = 0.0, cang = 0.0;
    if (graspClosingYaw(*chosen, request.object_box_center.orientation, yaw, cang))
      RCLCPP_INFO(LOGGER,
                  "从 %zu 个解里选中一个：代价 %.3f 合拢轴(物体系) %+.1f° 绕接近轴偏 %.1f°"
                  "（注释角 %+.1f° 是 MTC 的写法，比真实采样角大一个 angle_delta）",
                  task_.solutions().size(), chosen->cost(), yaw * 180.0 / M_PI,
                  std::abs(std::remainder(yaw + M_PI / 2, M_PI)) * 180.0 / M_PI,
                  cang * 180.0 / M_PI);
  }
  task_.introspection().publishSolution(*chosen);
  dumpSolution(*chosen);
  fixTrajectoryTimes(*chosen);

  // 5) execute
  const auto execute_result = task_.execute(*chosen);
  if (execute_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "任务执行失败，错误码 %d", execute_result.val);
    result.stage = GraspStage::EXEC_FAILED;
    result.message = "MTC 找到解但执行失败（错误码 " + std::to_string(execute_result.val) +
                     "；检查 arm_controller / gripper_controller）";
    return result;
  }

  result.success = true;
  result.stage = GraspStage::OK;
  result.message = "MTC pick & place 完成（抬升 " + std::to_string(lift_min_) + "~" +
                   std::to_string(lift_max_) + " m 后放回）";
  return result;
}

namespace turtlebot3_manipulation_grasp {

std::unique_ptr<GraspPlanner> makePickPlacePlanner(const std::shared_ptr<rclcpp::Node>& node) {
  return std::make_unique<MTCTaskNode>(node);
}

}  // namespace turtlebot3_manipulation_grasp
