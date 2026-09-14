// grasp_types.hpp
//
// 「视觉 → 抓取」契约在 C++ 侧的镜像，以及喂给 planner 的归一化数据结构。
//
// 分层约定：
//   * 本文件是纯数据，**不许** include MoveIt / MTC 头文件（缝要留在 planner 接口那侧）。
//   * VisionTarget  = 线上消息 GraspTargetStamped.msg 的原样镜像（未归一化，帧可能是 "map"）。
//   * GraspRequest  = 归一化之后的 planner 输入（几何一律已是规划帧下的值）。
//   * 两者之间的换算（TF + 查表 + 箱心计算）由适配器负责，见文末注释。
//
// 单位：米、弧度。帧名与机器人的事实依据：
//   MoveIt 规划帧 = base_footprint（URDF 根，SRDF 无 virtual joint，已实测确认）。

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace turtlebot3_manipulation_grasp {

/// 视觉目标（= msg/GraspTargetStamped.msg 的 C++ 镜像，语义完全一致）
struct VisionTarget {
  std::string frame;  ///< point 所在帧。**建议相对底盘**（camera_rgb_optical_frame
                      ///< 或 base_footprint）；不要用 map —— map 那段 TF 含 AMCL
                      ///< 定位误差，而抓取只需要"物体相对底盘"这个相对量。
  builtin_interfaces::msg::Time stamp;  ///< 检测时刻（TF 查询与新鲜度校验用）
  /// 类别名：取值 = 视觉侧 vision_pipeline.ITEM_NAMES 的【原样字符串】
  /// （"apple" / "coke can" / "bowl"，含空格）。本工程不改视觉侧词表，
  /// 所以 objects.yaml 的 key 必须原样照抄，查表用精确匹配。
  std::string class_id;
  double confidence{ 0.0 };             ///< 检测置信度 [0,1]
  /// 物体中心轴 ∩ 支撑面的交点，z = 支撑面高度。
  /// 注意：不是"可见表面上的点"（那会让箱心沿视线偏半个物体厚度）。
  geometry_msgs::msg::Point point;
};

/// 类别 → 碰撞箱尺寸 + 抓取偏置（objects.yaml 里的一行）
struct ObjectModel {
  std::string class_id;      ///< key：与 VisionTarget::class_id 精确匹配（原样字符串，含空格）
  double height{ 0.0 };      ///< 碰撞箱 z
  double depth{ 0.0 };       ///< 碰撞箱 x
  double width{ 0.0 };       ///< 碰撞箱 y
  double grasp_lift{ 0.0 };  ///< 抓取帧沿指尖方向相对箱心的偏置（m）
  /// 本夹爪（fr3_hand，开口 0.08 m）能否夹住。在 objects.yaml 里显式标注：
  /// 规则书要求"从四个物品中任选一个"，所以不可夹的类别必须能提前挡掉，
  /// 免得白白浪费一次抓取机会。
  bool graspable{ true };
};

/// 支撑面（桌面）碰撞箱。enabled=false 时不加入规划场景。
struct SupportSurface {
  bool enabled{ false };
  std::string frame;
  geometry_msgs::msg::Pose pose;  ///< 盒【中心】位姿
  double length{ 0.0 };           ///< x
  double width{ 0.0 };            ///< y
  double thickness{ 0.0 };        ///< z
};

/// 场景里的一个障碍碰撞盒（几何已是规划帧下的值）
struct ObstacleBox {
  std::string id;                       ///< 碰撞体 id（服务内唯一即可，如 "obstacle_1"）
  geometry_msgs::msg::Pose box_center;  ///< 盒中心（规划帧）
  ObjectModel model;                    ///< 尺寸（来自 objects.yaml 查表）
};

/// planner 的全部输入。几何量一律已经是 planning_frame 下的值。
struct GraspRequest {
  std::string planning_frame;                  ///< "base_footprint"
  std::string object_id{ "object" };           ///< 碰撞体 id
  geometry_msgs::msg::Pose object_box_center;  ///< 碰撞箱中心（规划帧）
  ObjectModel object;                          ///< 尺寸/抓取偏置（来自查表）
  SupportSurface support;                      ///< 桌面（来自配置或请求）
  /// 其它识别到的物品：不抓，但要作为障碍物进规划场景
  /// （规则书场景是桌上并排四个物品，只放目标那一个的话 MTC 会撞翻旁边的）。
  std::vector<ObstacleBox> obstacles;
};

/// 失败归因。编排层据此决定「重试 / 换站 / 报错」，抓取节点只负责如实上报。
/// 数值与 srv/GraspFixedObject.srv 响应区的 STAGE_* 常量一一对应，改一处必须改两处。
enum class GraspStage : uint8_t {
  OK = 0,             ///< 成功
  NO_TARGET = 1,      ///< 没有可用目标（视觉没给 / 数据过旧 / target 为空）
  BAD_TARGET = 2,     ///< 目标不合法（不在支撑面上、超出可达范围、类别查不到表…）
  SCENE_FAILED = 3,   ///< 场景没建起来（move_group 未就绪 / TF 失败）
  INIT_FAILED = 4,    ///< 任务建不起来（组名 / 命名状态 / IK 帧等配置问题）
  NO_SOLUTION = 5,    ///< 场景正常但规划无解
  EXEC_FAILED = 6,    ///< 求出解但执行失败（控制器未就绪 / 碰撞中止）
  BUSY = 7,           ///< 已有抓取正在执行（重入保护）
};

/// 一次抓取的结果（回填到服务/动作响应）
struct GraspResult {
  bool success{ false };
  GraspStage stage{ GraspStage::OK };
  std::string message;
};

// ── 下一步（本次未实现）──────────────────────────────────────────────
// 适配器把线上契约变成 planner 输入，顺序是：
//   1) GraspTargetStamped → VisionTarget
//   2) 新鲜度校验：now - stamp <= 阈值，否则 GraspStage::NO_TARGET
//   3) 查表：class_id → ObjectModel（查不到 → GraspStage::BAD_TARGET）
//   4) TF：point 从 VisionTarget.frame → planning_frame（用 stamp 查询）
//      → 得到支撑面上的轴心点
//   5) 箱心 = 轴心点 + (0, 0, height/2)，即 object_box_center
//   6) 目标校验：箱心是否落在 support 的足迹内、z 是否 ≈ 支撑面高度、
//      是否在臂可达范围内 → 否则 GraspStage::BAD_TARGET
// 完成上述之后才构造 GraspRequest 交给 GraspPlanner。   [待实现]

}  // namespace turtlebot3_manipulation_grasp
