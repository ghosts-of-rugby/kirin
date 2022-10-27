#include <Eigen/Geometry>
#include <iostream>
#include <algorithm>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include <magic_enum.hpp>
#include "kirin/frame.hpp"
#include "kirin/machine.hpp"
#include "kirin/utils.hpp"
#include "kirin/robot_joint_state_generator.hpp"

using namespace std::chrono_literals;

namespace {
const double kDepartXYThreshold   = 0.3;
const double kDepartPsiThreshold  = 0.2;
const double kZAutoFinishDistance = 0.005;

}  // namespace

RobotJointStateGenerator::RobotJointStateGenerator(const rclcpp::NodeOptions& options)
    : rclcpp::Node("robot_joint_state_generator", options),
      initial_pos_{
          Eigen::Vector3d(
              machine::kRedInitialPosX, machine::kRedInitialPosY, machine::kRedInitialPosZ),
          machine::kRedInitialPsi},
      pos_(initial_pos_),
      vel_{Eigen::Vector3d::Zero(), 0.0} {
  /* color direction from field */
  is_red_    = declare_parameter("field", "blue") == "red";
  color_dir_ = is_red_ ? -1.0 : 1.0;
  if (!is_red_) {  // if blue, fix initial position
    pos_.point.y()         = -1.0 * pos_.point.y();
    pos_.psi               = -1.0 * pos_.psi;
    initial_pos_.point.y() = -1.0 * initial_pos_.point.y();
    initial_pos_.psi       = -1.0 * initial_pos_.psi;
  }

  /* get parameter */
  velocity_ratio_ = {.x   = declare_parameter("velocity_ratio.x", 0.0),
                     .y   = declare_parameter("velocity_ratio.y", 0.0),
                     .z   = declare_parameter("velocity_ratio.z", 0.0),
                     .psi = declare_parameter("velocity_ratio.psi", 0.0)};
  z_auto_         = {.ratio           = declare_parameter("z_auto.ratio", 2.0),
                     .max_speed       = declare_parameter("z_auto.max_speed", 0.3),
                     .approach_offset = declare_parameter("z_auto.approach_offset", 0.05)};
  planar_auto_    = {.xy_ratio      = declare_parameter("xy_auto.ratio", 2.0),
                     .xy_max_speed  = declare_parameter("xy_auto.max_speed", 0.3),
                     .psi_ratio     = declare_parameter("psi_auto.ratio", 2.0),
                     .psi_max_speed = declare_parameter("psi_auto.max_speed", 0.3)};

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  world_coord_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("world_coord_pose", qos);
  joint_pub_       = create_publisher<sensor_msgs::msg::JointState>("manual_joint", qos);
  set_target_srv_  = create_service<SetTarget>(
      "generator/set_target", std::bind(&RobotJointStateGenerator::HandleSetTarget, this));
  start_z_srv_ = create_service<StartZAuto>(
      "generator/start_z_auto",
      std::bind(&RobotJointStateGenerator::HandleStartZAutoMovement, this));
  start_planar_srv_ = create_service<StartPlanarAuto>(
      "generator/start_planar_auto",
      std::bind(&RobotJointStateGenerator::HandleStartPlanarAutoMovement, this));
  timer_ = create_wall_timer(std::chrono::microseconds(static_cast<int64_t>(loop_ms_ * 1000)),
                             std::bind(&RobotJointStateGenerator::TimerCallback, this));
}

RobotJointStateGenerator::~RobotJointStateGenerator() {}
void RobotJointStateGenerator::HandleSetTarget(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<SetTarget::Request> request,
    std::shared_ptr<SetTarget::Response> response) {
  (void)request_header;  // Lint Tool 対策

  // when auto movement is enabled, do not change target
  if (!planar_auto_.enabled || !z_auto_.enabled) {
    current_target_  = request->target;
    response->result = true;
  } else {
    response->result = false;
  }
}

void RobotJointStateGenerator::HandleStartZAutoMovement(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<StartZAuto::Request> request,
    std::shared_ptr<StartZAuto::Response> response) {
  (void)request_header;  // Lint Tool 対策

  // if auto movement is enabled, return false
  if (z_auto_.enabled) {
    response->result = false;
    return;
  }
  // start z auto movement
  if (request->toggle) {
    z_auto_.state
        = (z_auto_.state == ZAutoState::Approach) ? ZAutoState::Depart : ZAutoState::Approach;
  } else {
    auto z_auto_state = magic_enum::enum_cast<ZAutoState>(request->z_auto_state.value);
    if (z_auto_state.has_value()) {
      z_auto_.state = z_auto_state.value();
    }
  }
  StartZAutoMovement();
  response->result = true;
}

void RobotJointStateGenerator::HandleStartPlanarAutoMovement(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<StartPlanarAuto::Request> request,
    std::shared_ptr<StartPlanarAuto::Response> response) {
  (void)request_header;  // Lint Tool 対策

  // if auto movement is enabled, return false
  if (planar_auto_.enabled) {
    response->result = false;
    return;
  }
  StartPlanarAutoMovement();
  response->result = true;
}

bool RobotJointStateGenerator::IsAllowedToChangeTarget() {
  if (kirin_utils::Contain(current_target_, "depart")) {
    if (xy_distance_.has_value() && psi_distance_.has_value()) {
      return (xy_distance_->norm() <= kDepartXYThreshold
              && psi_distance_.value() <= kDepartPsiThreshold);
    } else {
      return false;
    }
  } else {
    return false;
  }
}

void RobotJointStateGenerator::ReceiveManualInput(const HandPosition& manual_vel) {
  if (!z_auto_.enabled) {                   // z manual
    vel_.point.z() = manual_vel.point.z();  // receive z vel
  }
  if (!planar_auto_.enabled) {  // planar manual
    vel_.point.x() = manual_vel.point.x();
    vel_.point.y() = manual_vel.point.y();
    vel_.psi       = manual_vel.psi;
  }
}

geometry_msgs::msg::Pose RobotJointStateGenerator::GetPose() {
  /* input manual velocity */
  geometry_msgs::msg::Pose pose;

  /* if z auto movement, overwrite z velocity */
  if (z_auto_.enabled) {
    std::string target;
    double offset;
    if (z_auto_.state == ZAutoState::Approach && current_target_ != frame::kDepart) {
      target = current_target_;
      offset = z_auto_.approach_offset;
    } else {
      target = frame::kDepart;
      offset = 0.0;
    }
    auto z_auto_input = GetDistanceBasedZAutoVelocity(target, offset);
    if (z_auto_input.has_value()) {
      auto [z_auto_vel, z_distance] = z_auto_input.value();
      vel_.point.z()                = z_auto_vel;
      if (std::abs(z_distance - kirin_utils::sgn(z_distance) * offset) <= kZAutoFinishDistance) {
        FinishZAutoMovement();
        RCLCPP_INFO(this->get_logger(), "z auto movement finished!");
      }
    }
  }

  /* if planar auto movement, overwrite x y psi velocity */
  if (planar_auto_.enabled) {
    auto planar_auto_input = GetDistanceBasedPlanarAutoVelocity(current_target_);
    if (planar_auto_input.has_value()) {
      auto [plane_auto_vel, plane_distance] = planar_auto_input.value();
      auto [xy_auto_vel, psi_auto_vel]      = plane_auto_vel;
      std::tie(xy_distance_, psi_distance_) = plane_distance;
      vel_.point.x()                        = xy_auto_vel.x();
      vel_.point.y()                        = xy_auto_vel.y();
      vel_.psi                              = psi_auto_vel;
      double xy_threshold                   = (current_target_ == frame::kDepart) ? 0.05 : 0.002;
      double psi_threshold                  = (current_target_ == frame::kDepart) ? 0.1 : 0.03;
      if (xy_distance_->norm() <= xy_threshold
          && std::abs(psi_distance_.value()) <= psi_threshold) {
        FinishPlanarAutoMovement();
        RCLCPP_INFO(this->get_logger(), "planar auto movement finished!");
      }
    }
  } else {
    xy_distance_  = std::nullopt;
    psi_distance_ = std::nullopt;
  }

  pos_.point += vel_.point * loop_ms_ * 0.001;
  pose.position = Eigen::toMsg(pos_.point);

  pos_.psi += vel_.psi * loop_ms_ * 0.001;
  Eigen::Quaterniond quat(Eigen::AngleAxisd(pos_.psi, Eigen::Vector3d::UnitZ()));
  pose.orientation = Eigen::toMsg(quat);

  return std::move(pose);
}

std::optional<std::tuple<double, double>> RobotJointStateGenerator::GetDistanceBasedZAutoVelocity(
    const std::string& target, double offset) {
  auto pose = GetPoseFromTf(frame::kBellowsTop, target);
  if (pose.has_value()) {
    double z_distance  = pose->position.z;
    double auto_z_vel  = z_auto_.ratio * (z_distance - kirin_utils::sgn(z_distance) * offset);
    double input_z_vel = std::clamp(auto_z_vel, -z_auto_.max_speed, z_auto_.max_speed);
    return std::tie(input_z_vel, z_distance);
  } else {
    return std::nullopt;
  }
}

using PlaneTuple = RobotJointStateGenerator::PlaneTuple;
std::optional<std::tuple<PlaneTuple, PlaneTuple>>
RobotJointStateGenerator::GetDistanceBasedPlanarAutoVelocity(const std::string& target) {
  // bellowsから見たtargetの距離を計算
  auto distance         = GetPoseFromTf(frame::kBellowsTop, target);
  // コントローラーの速度入力はロボット座標系でのxyなので入力の回転変換用のPoseを取得
  auto bellows_in_robot = GetPoseFromTf(frame::kFixBase, frame::kBellowsTop);
  if (distance.has_value() && bellows_in_robot.has_value()) {
    Eigen::Vector2d xy_distance;
    {
      auto&& xy_distance_in_bellows = Eigen::Vector2d(distance->position.x, distance->position.y);
      auto [r_tmp, p_tmp, yaw_bellows_in_robot]
          = CalcGeometryQuatToRPY(bellows_in_robot->orientation);
      Eigen::Rotation2Dd rot(yaw_bellows_in_robot);
      xy_distance = rot.matrix() * xy_distance_in_bellows;
    }

    auto [roll, pitch, psi_distance] = CalcGeometryQuatToRPY(distance->orientation);

    auto&& auto_xy_vel  = planar_auto_.xy_ratio * xy_distance;
    double auto_psi_vel = planar_auto_.psi_ratio * psi_distance;
    auto&& input_xy_vel = Eigen::Vector2d(
        std::clamp(auto_xy_vel.x(), -planar_auto_.xy_max_speed, planar_auto_.xy_max_speed),
        std::clamp(auto_xy_vel.y(), -planar_auto_.xy_max_speed, planar_auto_.xy_max_speed));
    double input_psi_vel
        = std::clamp(auto_psi_vel, -planar_auto_.psi_max_speed, planar_auto_.psi_max_speed);
    PlaneTuple vel_tuple      = std::tie(input_xy_vel, input_psi_vel);
    PlaneTuple distance_tuple = std::tie(xy_distance, psi_distance);
    return std::tie(vel_tuple, distance_tuple);
  } else {
    return std::nullopt;
  }
}

using RPYTuple = RobotJointStateGenerator::RPYTuple;
RPYTuple RobotJointStateGenerator::CalcGeometryQuatToRPY(const geometry_msgs::msg::Quaternion& q) {
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  double r, p, y;
  tf2::Matrix3x3(quat).getRPY(r, p, y);
  return {r, p, y};
}

std::optional<geometry_msgs::msg::Pose> RobotJointStateGenerator::GetPoseFromTf(
    const std::string& parent_frame, const std::string& child_frame) {
  return kirin_utils::GetPoseFromTf(this->get_logger(), this->tf_buffer_, parent_frame,
                                    child_frame);
}

void RobotJointStateGenerator::PublishJointState(double l, double phi_offset) {
  auto joint_state  = std::make_unique<sensor_msgs::msg::JointState>();
  joint_state->name = {"theta_joint", "z_joint", "r_joint", "phi_joint", "phi_extend_joint"};

  /*  Offset between joint displacement and absolute position in the joint coordinate system */
  double r_offset = machine::kROffsetCenterToRRoot + machine::kROffsetRRootToPhi;
  double z_offset = initial_pos_.point.z() - machine::kZOffsetInitialDisplacement;

  /* calculate absolute position in the joint coordinate */
  double r     = model::CalcR(l, pos_.point.x(), pos_.point.y(), pos_.psi);
  double theta = model::CalcTheta(l, pos_.point.x(), pos_.point.y(), pos_.psi);
  double phi   = model::CalcPhi(l, pos_.point.x(), pos_.point.y(), pos_.psi);

  double dr     = model::CalcRVel(l, vel_.point.x(), vel_.point.y(), vel_.psi, r, theta, phi);
  double dtheta = model::CalcThetaVel(l, vel_.point.x(), vel_.point.y(), vel_.psi, r, theta, phi);
  double dphi   = model::CalcPhiVel(l, vel_.point.x(), vel_.point.y(), vel_.psi, r, theta, phi);

  /* calculate joint displacement */
  joint_state->position
      = {theta, pos_.point.z() - z_offset, r - r_offset, phi - phi_offset, phi - phi_offset};
  joint_state->velocity = {dtheta, vel_.point.z(), dr, dphi, dphi};
  joint_pub_->publish(std::move(joint_state));
}

std::optional<std::tuple<double, double>> RobotJointStateGenerator::GetBellowsOffset(
    const std::string& bellows_frame) {
  // calculate length from base to current bellows
  auto&& bellows_in_phi_link = GetPoseFromTf(frame::kPhiLink, bellows_frame);
  double l, phi_offset;
  if (bellows_in_phi_link) {
    double x   = bellows_in_phi_link->position.x;
    double y   = bellows_in_phi_link->position.y;
    l          = sqrt(x * x + y * y);
    phi_offset = atan2(y, x);
    return std::tie(l, phi_offset);
  } else {
    return std::nullopt;
  }
}

void RobotJointStateGenerator::TimerCallback() {
  // calculate length from base to current bellows
  auto&& offset_tuple = GetBellowsOffset(frame::kBellowsTop);
  if (!offset_tuple.has_value()) return;
  auto [l, phi_offset] = offset_tuple.value();

  auto input_pose = std::make_unique<geometry_msgs::msg::PoseStamped>();

  input_pose->header.frame_id = frame::kFixBase;
  input_pose->header.stamp    = get_clock()->now();
  input_pose->pose            = GetPose();
  world_coord_pub_->publish(std::move(input_pose));

  PublishJointState(l, phi_offset);
}

bool RobotJointStateGenerator::ValidateAndUpdateTarget() {
  if ((kirin_utils::Contain(current_target_, "pick")
       && kirin_utils::Contain(next_target_, "place"))) {
    current_target_ = frame::kDepart;
    RCLCPP_ERROR(this->get_logger(),
                 "move from pick to place directly is invalid! Go through depart position!");
    return false;
  } else if (kirin_utils::Contain(current_target_, "place")
             && kirin_utils::Contain(next_target_, "pick")) {
    current_target_ = frame::kDepart;
    RCLCPP_ERROR(this->get_logger(),
                 "move from place to pick directly is invalid! Go through depart position!");
    return false;
  } else {
    return true;
  }
}

void RobotJointStateGenerator::StartZAutoMovement() { z_auto_.enabled = true; }
void RobotJointStateGenerator::FinishZAutoMovement() { z_auto_.enabled = false; }
void RobotJointStateGenerator::StartPlanarAutoMovement() { planar_auto_.enabled = true; }
void RobotJointStateGenerator::FinishPlanarAutoMovement() { planar_auto_.enabled = false; }