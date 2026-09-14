// grasp_node.cpp —— 中间节点：把「视觉目标」翻译成「pick_and_place 的输入」
//
// 它在整个链路里的位置：
//
//   视觉节点 ──(/detect_grasp_target 服务, GraspTargetStamped)──► 编排 / 任务树
//                                                                    │
//                       /grasp_fixed_object (本节点提供, target 透传)  │
//                                                                    ▼
//   ┌──────────────────────── grasp_node（本文件）────────────────────────┐
//   │ 1) 新鲜度校验   stamp 太旧 → NO_TARGET                              │
//   │ 2) 查表        class_id → objects.yaml 的碰撞箱尺寸/抓取参数         │
//   │ 3) TF          target.frame(map) → base_footprint（用目标 stamp）    │
//   │ 4) 算箱心      箱心 = 支撑面上的轴心点 + (0,0,height/2)              │
//   │ 5) 目标校验     可夹？落在桌面足迹内？z≈桌面顶？在臂可达范围内？       │
//   │ 6) 交给规划器   GraspRequest（纯几何，已是规划帧）→ GraspPlanner      │
//   └────────────────────────────────────────────────────────────────────┘
//
// 本文件**只依赖 GraspPlanner 接口**（不含任何 MoveIt/MTC 符号）：真正的抓取
// 规划在 src/pick_and_place.cpp（MTCTaskNode，官方阶段图 + FR3 适配）。
//
// 线程：服务放独立回调组，抓取期间长时间阻塞也不会卡住本节点其它回调
// （TF 订阅在默认回调组，多线程执行器仍然能处理它）。

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <turtlebot3_manipulation_grasp/grasp_planner.hpp>
#include <turtlebot3_manipulation_grasp/grasp_types.hpp>
#include <turtlebot3_manipulation_grasp/srv/grasp_fixed_object.hpp>

using turtlebot3_manipulation_grasp::GraspPlanner;
using turtlebot3_manipulation_grasp::GraspRequest;
using turtlebot3_manipulation_grasp::GraspResult;
using turtlebot3_manipulation_grasp::GraspStage;
using turtlebot3_manipulation_grasp::makePickPlacePlanner;
using turtlebot3_manipulation_grasp::ObjectModel;
using turtlebot3_manipulation_grasp::ObstacleBox;
using turtlebot3_manipulation_grasp::SupportSurface;
using turtlebot3_manipulation_grasp::srv::GraspFixedObject;

namespace {

const rclcpp::Logger LOGGER = rclcpp::get_logger("turtlebot3_manipulation_grasp");

/// [x,y,z,r,p,y] → Pose
geometry_msgs::msg::Pose poseFromRpyArray(const std::vector<double>& v) {
  geometry_msgs::msg::Pose pose;
  if (v.size() < 3) {
    pose.orientation.w = 1.0;
    return pose;
  }
  const Eigen::Isometry3d iso =
      Eigen::Translation3d(v[0], v[1], v[2]) *
      Eigen::AngleAxisd(v.size() > 3 ? v[3] : 0.0, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(v.size() > 4 ? v[4] : 0.0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(v.size() > 5 ? v[5] : 0.0, Eigen::Vector3d::UnitZ());
  return tf2::toMsg(iso);
}

std::string fmt(double v) { return std::to_string(v); }

/// 只含绕 z 的偏航 → 四元数（物体朝向用，不做完整 RPY）
geometry_msgs::msg::Quaternion yawOnlyQuat(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

}  // namespace

class GraspNode : public rclcpp::Node
{
public:
  GraspNode() : Node("grasp_node") {
    loadParams();
    loadCatalog();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  /// 必须在 make_shared 之后调用（要用 shared_from_this 建规划器）
  void start() {
    planner_ = makePickPlacePlanner(shared_from_this());
    if (!planner_)
      throw std::runtime_error("makePickPlacePlanner() 返回空，规划器未接入");

    // 服务独立回调组：抓取会阻塞数分钟，不能拖住本节点其它回调。
    service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    srv_ = this->create_service<GraspFixedObject>(
        "/grasp_fixed_object",
        [this](const std::shared_ptr<GraspFixedObject::Request> req,
               std::shared_ptr<GraspFixedObject::Response> res) { onGrasp(req, res); },
        rmw_qos_profile_services_default, service_cb_group_);

    RCLCPP_INFO(LOGGER, "grasp_node 就绪: 规划帧=%s, 目录表 %zu 类, 支撑面=%s, 可达范围=[%.2f, %.2f] m",
                planning_frame_.c_str(), catalog_.size(), support_.enabled ? "on" : "off", reach_min_,
                reach_max_);
    RCLCPP_INFO(LOGGER, "等待 /grasp_fixed_object（目标由调用方从视觉服务取得后透传）");
  }

private:
  // ── 配置 ────────────────────────────────────────────────────────────
  std::string planning_frame_;
  std::string object_id_;
  std::string arm_base_frame_;
  double max_target_age_{ 10.0 };
  double reach_min_{ 0.10 };
  double reach_max_{ 0.85 };
  double reach_comfort_min_{ 0.35 };
  double reach_comfort_max_{ 0.55 };
  /// 目标帧若用这些帧（含 AMCL 定位误差），会告警提示改发相对底盘的帧
  std::vector<std::string> frames_with_localisation_error_;
  double z_tolerance_{ 0.03 };
  double footprint_margin_{ 0.05 };
  /// 物体自身坐标系在 map 里的偏航（本世界 18 个物体都按出厂姿态摆 → 0）。
  /// 用于把目录表的 depth/width 正确摆进规划帧，见 toRequest 步骤 4 的说明。
  double object_yaw_map_{ 0.0 };
  /// 机器人当前在 map 里的偏航（每次请求用 TF 现查）
  double robot_yaw_map_{ 0.0 };
  SupportSurface support_;
  std::map<std::string, ObjectModel> catalog_;

  // ── 运行时 ──────────────────────────────────────────────────────────
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<GraspPlanner> planner_;
  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  rclcpp::Service<GraspFixedObject>::SharedPtr srv_;
  std::atomic_bool busy_{ false };

  template <typename T>
  T paramOr(const std::string& name, const T& fallback) {
    if (!this->has_parameter(name))
      this->declare_parameter<T>(name, fallback);
    return this->get_parameter(name).get_value<T>();
  }

  void loadParams() {
    planning_frame_ = paramOr<std::string>("planning_frame", "base_footprint");
    object_id_ = paramOr<std::string>("object_id", "object");
    arm_base_frame_ = paramOr<std::string>("arm_base_frame", "fr3_link0");
    max_target_age_ = paramOr<double>("max_target_age", 10.0);
    reach_min_ = paramOr<double>("reach_min", 0.10);
    reach_max_ = paramOr<double>("reach_max", 0.85);
    reach_comfort_min_ = paramOr<double>("reach_comfort_min", 0.35);
    reach_comfort_max_ = paramOr<double>("reach_comfort_max", 0.55);
    frames_with_localisation_error_ =
        paramOr<std::vector<std::string>>("frames_with_localisation_error", { "map" });
    z_tolerance_ = paramOr<double>("z_tolerance", 0.03);
    footprint_margin_ = paramOr<double>("footprint_margin", 0.05);
    object_yaw_map_ = paramOr<double>("object_yaw_map", 0.0);

    support_.enabled = paramOr<bool>("support_surface.enabled", true);
    support_.frame = paramOr<std::string>("support_surface.frame", "map");
    support_.pose = poseFromRpyArray(paramOr<std::vector<double>>(
        "support_surface.pose", { 2.7, 2.0, 0.765, 0.0, 0.0, 1.5707963267948966 }));
    support_.length = paramOr<double>("support_surface.length", 0.5);
    support_.width = paramOr<double>("support_surface.width", 1.2);
    support_.thickness = paramOr<double>("support_surface.thickness", 0.03);
  }

  void loadCatalog() {
    const std::string file = paramOr<std::string>("objects_catalog", "objects.yaml");
    const std::string path =
        ament_index_cpp::get_package_share_directory("turtlebot3_manipulation_grasp") + "/config/" + file;

    YAML::Node root;
    try {
      root = YAML::LoadFile(path);
    } catch (const std::exception& e) {
      // 快速失败：目录表是查表的唯一来源，读不到就没有继续的意义
      throw std::runtime_error("读取物体目录表失败: " + path + " (" + e.what() + ")");
    }

    const YAML::Node objects = root["objects"];
    if (!objects || !objects.IsMap())
      throw std::runtime_error("物体目录表缺少 objects 映射: " + path);

    for (const auto& item : objects) {
      const std::string class_id = item.first.as<std::string>();
      const YAML::Node v = item.second;
      ObjectModel m;
      m.class_id = class_id;
      try {
        m.height = v["height"].as<double>();
        m.depth = v["depth"].as<double>();
        m.width = v["width"].as<double>();
        m.grasp_lift = v["grasp_lift"] ? v["grasp_lift"].as<double>() : 0.0;
        m.graspable = v["graspable"] ? v["graspable"].as<bool>() : true;
      } catch (const std::exception& e) {
        throw std::runtime_error("目录表条目 '" + class_id + "' 字段不完整: " + e.what());
      }
      catalog_[class_id] = m;
      RCLCPP_INFO(LOGGER, "  目录: \"%s\" 箱=(h %.4f, d %.4f, w %.4f) lift=%.3f graspable=%s",
                  class_id.c_str(), m.height, m.depth, m.width, m.grasp_lift, m.graspable ? "yes" : "no");
    }
  }

  // ── 小工具 ──────────────────────────────────────────────────────────
  /// 查 TF：先非阻塞探测一次，再带超时查。
  /// 这样"没有 TF / 帧不存在"这类常见问题会立刻返回一句可读的错误，
  /// 而不是陷在 tf2 的等待循环里（stamp=0 时会等"最新可用"，容易被拖住）。
  bool lookupTransform(const std::string& target, const std::string& source, const rclcpp::Time& stamp,
                       geometry_msgs::msg::TransformStamped& out, std::string& err) {
    if (target == source) {
      out = geometry_msgs::msg::TransformStamped();
      out.header.frame_id = target;
      out.child_frame_id = source;
      out.transform.rotation.w = 1.0;
      return true;
    }
    if (!tf_buffer_->canTransform(target, source, stamp, rclcpp::Duration::from_seconds(0.0)) &&
        !tf_buffer_->canTransform(target, source, stamp, rclcpp::Duration::from_seconds(0.5))) {
      err = "查不到 TF " + source + " → " + target +
            "（stamp=" + std::to_string(stamp.nanoseconds()) +
            "；机器人 / TF 发布器没起来？或该帧不在树里）";
      return false;
    }
    try {
      out = tf_buffer_->lookupTransform(target, source, stamp, rclcpp::Duration::from_seconds(0.5));
      return true;
    } catch (const tf2::TransformException& e) {
      err = "TF " + source + " → " + target + " 失败: " + e.what();
      return false;
    }
  }

  /// 点：from 帧 → planning_frame_
  bool transformPoint(const geometry_msgs::msg::Point& in, const std::string& from, const rclcpp::Time& stamp,
                      geometry_msgs::msg::Point& out, std::string& err) {
    if (from == planning_frame_ || from.empty()) {
      out = in;
      return true;
    }
    geometry_msgs::msg::TransformStamped tf;
    if (!lookupTransform(planning_frame_, from, stamp, tf, err))
      return false;
    const Eigen::Vector3d p = tf2::transformToEigen(tf) * Eigen::Vector3d(in.x, in.y, in.z);
    out.x = p.x();
    out.y = p.y();
    out.z = p.z();
    return true;
  }

  /// 位姿：from 帧 → planning_frame_
  bool transformPose(const geometry_msgs::msg::Pose& in, const std::string& from, const rclcpp::Time& stamp,
                     geometry_msgs::msg::Pose& out, std::string& err) {
    if (from == planning_frame_ || from.empty()) {
      out = in;
      return true;
    }
    geometry_msgs::msg::TransformStamped tf;
    if (!lookupTransform(planning_frame_, from, stamp, tf, err))
      return false;
    Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
    tf2::fromMsg(in, p);
    out = tf2::toMsg(tf2::transformToEigen(tf) * p);
    return true;
  }

  /// 点在支撑面自身坐标系里是否落在足迹内（含 margin）
  bool insideFootprint(const geometry_msgs::msg::Point& p_planning,
                       const geometry_msgs::msg::Pose& support_planning, std::string& err) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    tf2::fromMsg(support_planning, T);
    const Eigen::Vector3d p = T.inverse() * Eigen::Vector3d(p_planning.x, p_planning.y, p_planning.z);
    const double hx = 0.5 * support_.length + footprint_margin_;
    const double hy = 0.5 * support_.width + footprint_margin_;
    if (std::abs(p.x()) > hx || std::abs(p.y()) > hy) {
      err = "目标不在支撑面足迹内: 支撑面局部坐标 (" + fmt(p.x()) + ", " + fmt(p.y()) + ") 超出 ±(" + fmt(hx) +
            ", " + fmt(hy) + ")";
      return false;
    }
    return true;
  }

  // ── 核心：请求 → GraspRequest ────────────────────────────────────────
  bool toRequest(const GraspFixedObject::Request& req, GraspRequest& out, GraspStage& stage, std::string& err) {
    const auto& t = req.target;

    // 1) 新鲜度
    const rclcpp::Time stamp(t.header.stamp);
    if (stamp.nanoseconds() != 0) {
      const double age = (this->now() - stamp).seconds();
      if (age < -1.0) {
        // 时间戳比我还"未来"，几乎一定是时钟域不一致：调用方用墙钟、本节点用仿真钟
        stage = GraspStage::NO_TARGET;
        err = "目标时间戳比本节点时钟超前 " + fmt(-age) + "s —— 时钟域不一致。本节点 now=" +
              fmt(this->now().seconds()) + "s（use_sim_time 跟随 /clock），目标 stamp=" +
              fmt(stamp.seconds()) + "s。请让调用方也开 use_sim_time（或双方都用墙钟）";
        return false;
      }
      if (age > max_target_age_) {
        stage = GraspStage::NO_TARGET;
        err = "视觉目标不新鲜: age=" + fmt(age) + "s（阈值 " + fmt(max_target_age_) + "s）";
        return false;
      }
    } else {
      RCLCPP_WARN(LOGGER, "target.header.stamp 为 0：跳过新鲜度校验（手工调试），TF 取最新时刻");
    }

    // 1.5) 帧的"是否经过定位链"提醒：抓取只需要物体【相对底盘】的量，
    //      map 系目标会把 AMCL 的定位误差带进来。
    for (const auto& f : frames_with_localisation_error_) {
      if (t.header.frame_id == f) {
        RCLCPP_WARN(LOGGER,
                    "目标帧 \"%s\" 经过定位链（含 AMCL 误差）：抓取精度会随定位漂移。建议改发"
                    "相对底盘的帧（camera_rgb_optical_frame 或 base_footprint）",
                    f.c_str());
        break;
      }
    }

    // 2) 查表
    if (t.class_id.empty()) {
      stage = GraspStage::NO_TARGET;
      err = "target.class_id 为空";
      return false;
    }
    const auto it = catalog_.find(t.class_id);
    if (it == catalog_.end()) {
      stage = GraspStage::BAD_TARGET;
      err = "目录表里没有类别 \"" + t.class_id + "\"（请在 config/objects.yaml 加一行）";
      return false;
    }
    const ObjectModel& model = it->second;
    if (!model.graspable) {
      stage = GraspStage::BAD_TARGET;
      err = "类别 \"" + t.class_id + "\" 超出夹爪开口，标为不可夹（min(depth,width)=" +
            fmt(std::min(model.depth, model.width)) + " m）";
      return false;
    }

    // 3) TF：目标点 → 规划帧
    geometry_msgs::msg::Point p;
    std::string tf_err;
    if (!transformPoint(t.point, t.header.frame_id, stamp, p, tf_err)) {
      stage = GraspStage::SCENE_FAILED;
      err = tf_err;
      return false;
    }

    // 支撑面也换算到规划帧：桌子在世界里固定，而机器人停位有误差，
    // 两者用同一时刻的 TF 变换 → 相对几何不受停位误差影响。
    geometry_msgs::msg::Pose support_planning = support_.pose;
    if (support_.enabled && !transformPose(support_.pose, support_.frame, stamp, support_planning, tf_err)) {
      stage = GraspStage::SCENE_FAILED;
      err = "支撑面 " + tf_err;
      return false;
    }

    // 3b) 机器人当前在 map 里的偏航 —— 物体朝向要用它把"map 对齐的 depth/width"
    //     摆进 base_footprint 系（见下面步骤 4 的说明）。查不到 map 帧就退回 0
    //     （那时等价于旧行为：箱体对齐车体系，只在"车头正对 map +x"时正确）。
    robot_yaw_map_ = 0.0;
    {
      geometry_msgs::msg::TransformStamped map_base;
      std::string map_err;
      if (lookupTransform("map", planning_frame_, rclcpp::Time(0), map_base, map_err)) {
        const auto& q = map_base.transform.rotation;
        robot_yaw_map_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      } else {
        RCLCPP_WARN(LOGGER, "查不到 map←%s（%s）→ 物体朝向按车体系算（车头需正对 map +x 才准）",
                    planning_frame_.c_str(), map_err.c_str());
      }
    }

    // 4) 箱心 = 支撑面上的轴心点 + (0, 0, height/2)
    //
    // ★ 朝向必须带上（2026-09-14 实测故障：盒子被顶倒）：
    //   目录表的 depth/width 是**物体自身坐标系**下的（与出厂姿态一致 = 与世界 map 轴对齐），
    //   而规划帧是 base_footprint（跟着车转）。原来这里写 orientation.w = 1（= 对齐车体系），
    //   车一旦不是"车头正对 map +x"就会把 depth/width 两个轴对调 ✗：
    //   实测车正对餐桌（map yaw = −π/2）抓 sugar_box(0.038 × 0.089) 时，
    //   模型以为薄边沿车的 x，实际薄边沿车的 y → MTC 沿车的 x 合爪（真实宽度 0.089 > 开口 0.08）
    //   → 手指压上盒子把它顶倒 ✗（Gazebo 里看到"盒子倒了、没抓起来"就是这个）
    //   修法：箱子朝向 = Rz(−机器人 map 偏航)（物体在 map 里按出厂姿态摆，即与 map 轴对齐）；
    //   物体在 map 里的朝向由参数 object_yaw_map 给出（本世界 18 个物体全是 0）。
    geometry_msgs::msg::Pose box_center;
    box_center.position.x = p.x;
    box_center.position.y = p.y;
    box_center.position.z = p.z + 0.5 * model.height;
    box_center.orientation = yawOnlyQuat(-robot_yaw_map_ + object_yaw_map_);

    // 5) 目标校验
    if (support_.enabled) {
      const double surface_top = support_planning.position.z + 0.5 * support_.thickness;
      if (std::abs(p.z - surface_top) > z_tolerance_) {
        stage = GraspStage::BAD_TARGET;
        err = "目标点 z=" + fmt(p.z) + " 与桌面顶 z=" + fmt(surface_top) + " 相差超过 " + fmt(z_tolerance_) +
              "（检查视觉侧的支撑面高度参数 / TF）";
        return false;
      }
      std::string fp_err;
      if (!insideFootprint(p, support_planning, fp_err)) {
        stage = GraspStage::BAD_TARGET;
        err = fp_err;
        return false;
      }

      // 5b) 把支撑面【顶面】对齐到目标点 z：按契约，目标点就落在支撑面上，
      //     两者必须自洽。实测踩过：配置的桌面顶（中心 0.765 + 厚 0.03 = 0.78）
      //     比罐底真值 0.7774 高 2.6 mm，物体碰撞箱因此埋进桌面 2.6 mm，
      //     MTC 的 attach object 直接判 "table colliding with object"（0/104）失败。
      //     注意上面的 z_tolerance 校验仍是对【配置值】做的，粗差照样被拦住。
      support_planning.position.z = p.z - 0.5 * support_.thickness;
    }

    // 可达范围：臂基座位置用 TF 拿，不写死常数
    geometry_msgs::msg::TransformStamped base_tf;
    std::string base_err;
    if (!lookupTransform(planning_frame_, arm_base_frame_, rclcpp::Time(0), base_tf, base_err)) {
      stage = GraspStage::SCENE_FAILED;
      err = std::string("查询臂基座 ") + arm_base_frame_ + " 失败: " + base_err;
      return false;
    }
    {
      const Eigen::Vector3d base = tf2::transformToEigen(base_tf).translation();
      const double d = (Eigen::Vector3d(box_center.position.x, box_center.position.y, box_center.position.z) - base)
                           .norm();
      if (d < reach_min_ || d > reach_max_) {
        stage = GraspStage::BAD_TARGET;
        err = "目标距臂基座 " + fmt(d) + " m 超出 [" + fmt(reach_min_) + ", " + fmt(reach_max_) + "] m";
        return false;
      }
      RCLCPP_INFO(LOGGER, "  距臂基座 %s = %.3f m（上限 %.3f）", arm_base_frame_.c_str(), d, reach_max_);
      if (d < reach_comfort_min_ || d > reach_comfort_max_) {
        RCLCPP_WARN(LOGGER,
                    "  目标距臂基座 %.3f m 在舒适带 [%.2f, %.2f] 之外：仍会尝试规划，但接近臂展极限时"
                    "容易失败或夹持不稳 —— 考虑先让底盘微调到位（相对量微调，不受定位误差影响）",
                    d, reach_comfort_min_, reach_comfort_max_);
      }
    }

    // 6) 组装（交给规划器的几何一律已是规划帧下的值）
    out = GraspRequest();
    out.planning_frame = planning_frame_;
    out.object_id = object_id_;
    out.object = model;
    out.object_box_center = box_center;
    out.support = support_;
    if (support_.enabled)
      out.support.pose = support_planning;

    // 障碍物：桌上其它已识别到的物品。注意**不做 graspable 过滤** ——
    // 夹不住的东西（比如 bowl）更要放进场景，否则 MTC 会一头撞上去。
    // 查不到表或 TF 失败的：跳过并告警，不让它拖垮这次抓取。
    int obstacle_index = 0;
    for (const auto& ob : req.obstacles) {
      if (ob.class_id.empty())
        continue;
      const auto ob_it = catalog_.find(ob.class_id);
      if (ob_it == catalog_.end()) {
        RCLCPP_WARN(LOGGER, "障碍物类别 \"%s\" 不在目录表里，跳过（未进场景）", ob.class_id.c_str());
        continue;
      }
      geometry_msgs::msg::Point q;
      if (!transformPoint(ob.point, ob.header.frame_id, stamp, q, tf_err)) {
        RCLCPP_WARN(LOGGER, "障碍物 \"%s\" 变换失败，跳过: %s", ob.class_id.c_str(), tf_err.c_str());
        continue;
      }
      ObstacleBox box;
      box.id = "obstacle_" + std::to_string(++obstacle_index);
      box.model = ob_it->second;
      box.box_center.position.x = q.x;
      box.box_center.position.y = q.y;
      box.box_center.position.z = q.z + 0.5 * box.model.height;
      // 障碍物同样不能写 identity（理由见步骤 4）：否则它的 depth/width 也会对调
      box.box_center.orientation = yawOnlyQuat(object_yaw_map_ - robot_yaw_map_);
      out.obstacles.push_back(box);
      RCLCPP_INFO(LOGGER, "障碍 \"%s\"(%s) → 箱心(%.3f, %.3f, %.3f)", box.id.c_str(), ob.class_id.c_str(),
                  box.box_center.position.x, box.box_center.position.y, box.box_center.position.z);
    }

    RCLCPP_INFO(LOGGER,
                "目标 \"%s\" conf=%.2f: %s(%.3f, %.3f, %.3f) → %s(%.3f, %.3f, %.3f) | 箱心(%.3f, %.3f, %.3f) "
                "箱=(h %.4f, d %.4f, w %.4f) lift=%.3f 箱体偏航(规划帧)=%.1f°",
                t.class_id.c_str(), t.confidence, t.header.frame_id.c_str(), t.point.x, t.point.y, t.point.z,
                planning_frame_.c_str(), p.x, p.y, p.z, box_center.position.x, box_center.position.y,
                box_center.position.z, model.height, model.depth, model.width, model.grasp_lift,
                (object_yaw_map_ - robot_yaw_map_) * 180.0 / M_PI);
    return true;
  }

  void onGrasp(const std::shared_ptr<GraspFixedObject::Request> req,
               std::shared_ptr<GraspFixedObject::Response> res) {
    res->success = false;
    res->stage = static_cast<uint8_t>(GraspStage::OK);
    res->message.clear();

    // 重入保护：同一时刻只跑一次抓取
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
      res->stage = static_cast<uint8_t>(GraspStage::BUSY);
      res->message = "已有抓取正在执行";
      RCLCPP_WARN(LOGGER, "%s", res->message.c_str());
      return;
    }

    const auto t0 = std::chrono::steady_clock::now();
    GraspStage stage = GraspStage::OK;
    std::string err;
    GraspRequest request;

    if (toRequest(*req, request, stage, err)) {
      const GraspResult result = planner_->run(request);  // 阻塞：建场景 → 规划 → 执行
      res->success = result.success;
      res->stage = static_cast<uint8_t>(result.stage);
      res->message = result.message;
    } else {
      res->stage = static_cast<uint8_t>(stage);
      res->message = err;
      RCLCPP_ERROR(LOGGER, "目标校验失败: %s", err.c_str());
    }

    const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    RCLCPP_INFO(LOGGER, "抓取结束: success=%s stage=%u (%.1fs) msg=%s", res->success ? "true" : "false",
                res->stage, dt, res->message.c_str());

    busy_ = false;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<GraspNode>();
    node->start();
    // 抓取回调长时间阻塞：多线程执行器 + 独立回调组，
    // 保证抓取期间 TF 订阅、参数服务等仍然可用。
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(LOGGER, "grasp_node 启动失败: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
