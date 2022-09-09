#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include <kirin_msgs/srv/toggle_hand_state.hpp>
#include <kirin_msgs/srv/set_air_state.hpp>
#include "kirin/world_coord_manual_controller.hpp"
#include "kirin/frame.hpp"
#include "kirin/machine.hpp"
#include <iostream>

using namespace std::chrono_literals;

WorldCoordManualController::WorldCoordManualController(const std::string& node_name,
                                                       const rclcpp::NodeOptions& options)
    : JoyController(node_name, options),
      pos_(0.546 + 0.05, 0.00, 0.127),
      vel_(0.0, 0.0, 0.0),
      psi_(0.0),
      dpsi_(0.0),
      current_bellows_frame_{frame::kBellowsTop},
      timer_callback_(std::bind(&WorldCoordManualController::TimerCallback, this)) {
  velocity_ratio.x   = declare_parameter("velocity_ratio.x", 0.0);
  velocity_ratio.y   = declare_parameter("velocity_ratio.y", 0.0);
  velocity_ratio.z   = declare_parameter("velocity_ratio.z", 0.0);
  velocity_ratio.psi = declare_parameter("velocity_ratio.psi", 0.0);

  // transform listener
  tf_buffer_          = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  {
    // wait transform published from robot_state_publiser and hand_tool_manager
    // TODO use lifecycle or not
    std::this_thread::sleep_for(300ms);
  }

  this->RegisterButtonPressedCallback(
      Button::A, std::bind(&WorldCoordManualController::ChangeHandStateClientRequest, this));

  this->RegisterButtonPressedCallback(
      Button::X, std::bind(&WorldCoordManualController::ChangePumpStateClientRequest, this));

  // this->RegisterButtonPressedCallback(
  //   Button::X, std::bind(&WorldCoordManualController::ChangeBellows, this));

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  world_coord_pub_     = create_publisher<geometry_msgs::msg::PoseStamped>(input_topic_name_, qos);
  joint_pub_           = create_publisher<sensor_msgs::msg::JointState>("manual_joint", qos);
  current_bellows_pub_ = create_publisher<std_msgs::msg::String>("current_bellows_frame_", qos);
  timer_               = create_wall_timer(std::chrono::milliseconds(loop_ms_), timer_callback_);
  toggle_hand_state_client_
      = create_client<kirin_msgs::srv::ToggleHandState>("tool/toggle_hand_state");
  set_air_state_client_ = create_client<kirin_msgs::srv::SetAirState>("tool/set_air_state");
}

WorldCoordManualController::~WorldCoordManualController() {}

geometry_msgs::msg::Pose WorldCoordManualController::GetManualInput() {
  geometry_msgs::msg::Pose pose;
  vel_ = Eigen::Vector3d(this->GetAxis(JoyController::Axis::LStickY) * -1.0 * velocity_ratio.x,
                         this->GetAxis(JoyController::Axis::LStickX) * 1.0 * velocity_ratio.y,
                         this->GetAxis(JoyController::Axis::RStickX) * 1.0 * velocity_ratio.z);
  pos_ += vel_ * loop_ms_ * 0.001;  // 10ms loop
  pose.position = Eigen::toMsg(pos_);

  double left_trig  = 1.0 - this->GetAxis(JoyController::Axis::LTrigger);
  double right_trig = 1.0 - this->GetAxis(JoyController::Axis::RTrigger);
  dpsi_             = (left_trig - right_trig) * velocity_ratio.psi;
  psi_ += dpsi_ * loop_ms_ * 0.001;
  Eigen::Quaterniond quat(Eigen::AngleAxisd(psi_, Eigen::Vector3d::UnitZ()));
  pose.orientation = Eigen::toMsg(quat);

  return std::move(pose);
}

std::optional<geometry_msgs::msg::Pose> WorldCoordManualController::GetPoseFromTf(
    const std::string& parent_frame, const std::string& child_frame) {
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_->lookupTransform(parent_frame, child_frame, tf2::TimePointZero);
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", parent_frame.c_str(),
                child_frame.c_str(), ex.what());
    return std::nullopt;
  }

  geometry_msgs::msg::Pose pose;
  pose.position.x  = transform_stamped.transform.translation.x;
  pose.position.y  = transform_stamped.transform.translation.y;
  pose.position.z  = transform_stamped.transform.translation.z;
  pose.orientation = transform_stamped.transform.rotation;
  return pose;
}

RPYTuple WorldCoordManualController::CalcGeometryQuatToRPY(
    const geometry_msgs::msg::Quaternion& q) {
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  double r, p, y;
  tf2::Matrix3x3(quat).getRPY(r, p, y);
  return {r, p, y};
}

void WorldCoordManualController::SetCurrentBellows(const std::string& bellows_frame) {
  RCLCPP_INFO(this->get_logger(), "current bellows: '%s' -> '%s'", current_bellows_frame_.c_str(),
              bellows_frame.c_str());

  auto&& next_in_now = GetPoseFromTf(current_bellows_frame_, bellows_frame);
  if (next_in_now) {
    Eigen::Quaterniond quat(Eigen::AngleAxisd(psi_, Eigen::Vector3d::UnitZ()));
    Eigen::Vector3d t(next_in_now.value().position.x, next_in_now.value().position.y,
                      next_in_now.value().position.z);
    RCLCPP_INFO(this->get_logger(), "pos: %3f, %3f, %3f", pos_.x(), pos_.y(), pos_.z());
    pos_ += quat * t;
    RCLCPP_INFO(this->get_logger(), "pos: %3f, %3f, %3f", pos_.x(), pos_.y(), pos_.z());
  }

  current_bellows_frame_ = bellows_frame;
  std_msgs::msg::String bellows_msg;
  bellows_msg.data = "current bellows: " + current_bellows_frame_;
  current_bellows_pub_->publish(std::move(bellows_msg));

  auto&& bellows_in_phi_link = GetPoseFromTf(frame::kPhiLink, current_bellows_frame_);
  double l, phi_offset;
  if (bellows_in_phi_link) {
    double x   = bellows_in_phi_link.value().position.x;
    double y   = bellows_in_phi_link.value().position.y;
    l          = sqrt(x * x + y * y);
    phi_offset = atan2(y, x);
    PublishJointState(l, phi_offset);
  } else {
    return;
  }
}

void WorldCoordManualController::ChangeBellows() {
  if (current_bellows_frame_ == frame::kBellowsTop) SetCurrentBellows(frame::kBellowsLeft);
  else if (current_bellows_frame_ == frame::kBellowsLeft) SetCurrentBellows(frame::kBellowsRight);
  else /* current_bellows_frame == frame::kBellowsRight */ SetCurrentBellows(frame::kBellowsTop);
}

void WorldCoordManualController::PublishJointState(double l, double phi_offset) {
  auto joint_state  = std::make_unique<sensor_msgs::msg::JointState>();
  joint_state->name = {"theta_joint", "z_joint", "r_joint", "phi_joint", "phi_extend_joint"};

  double r_offset = 0.345 + 0.201;
  double z_offset = 0.1;

  static double pre_r     = 0.0;
  static double pre_theta = 0.0;
  static double pre_phi   = 0.0;

  double r     = model::CalcR(l, pos_.x(), pos_.y(), psi_);
  double theta = model::CalcTheta(l, pos_.x(), pos_.y(), psi_);
  double phi   = model::CalcPhi(l, pos_.x(), pos_.y(), psi_);

  /* check */
  double a_dr     = (r - pre_r) / 0.02;
  double a_dtheta = (theta - pre_theta) / 0.02;
  double a_dphi   = (phi - pre_phi) / 0.02;
  pre_r           = r;
  pre_theta       = theta;
  pre_phi         = phi;

  double dr     = model::CalcRVel(l, vel_.x(), vel_.y(), dpsi_, r, theta, phi);
  double dtheta = model::CalcThetaVel(l, vel_.x(), vel_.y(), dpsi_, r, theta, phi);
  double dphi   = model::CalcPhiVel(l, vel_.x(), vel_.y(), dpsi_, r, theta, phi);

  // RCLCPP_INFO(this->get_logger(),
  //             "[apx] dr: %5f, dtheta: %5f, dphi: %5f, [mat] dr: %5f, dtheta: %5f, dphi: %5f",
  //             a_dr, a_dtheta, a_dphi, dr, dtheta, dphi);

  joint_state->position
      = {theta, pos_.z() - z_offset, r - r_offset, phi - phi_offset, phi - phi_offset};
  joint_state->velocity = {dtheta, vel_.z(), dr, dphi, dphi};
  joint_pub_->publish(std::move(joint_state));
}

void WorldCoordManualController::TimerCallback() {
  // calculate length from base to current bellows
  auto&& bellows_in_phi_link = GetPoseFromTf(frame::kPhiLink, current_bellows_frame_);
  double l, phi_offset;
  if (bellows_in_phi_link) {
    double x   = bellows_in_phi_link.value().position.x;
    double y   = bellows_in_phi_link.value().position.y;
    l          = sqrt(x * x + y * y);
    phi_offset = atan2(y, x);
    // RCLCPP_INFO(this->get_logger(), "x: %f, y: %f", x, y);
    // RCLCPP_INFO(this->get_logger(), "l: %f, phi offset: %f", l, phi_offset);
  } else {
    return;
  }

  auto input_pose = std::make_unique<geometry_msgs::msg::PoseStamped>();

  input_pose->header.frame_id = frame::kFixBase;
  input_pose->header.stamp    = get_clock()->now();
  input_pose->pose            = GetManualInput();
  world_coord_pub_->publish(std::move(input_pose));

  PublishJointState(l, phi_offset);
}

void WorldCoordManualController::ChangeHandStateClientRequest() {
  auto request    = std::make_shared<kirin_msgs::srv::ToggleHandState::Request>();
  request->toggle = true;

  using ResponseFuture   = rclcpp::Client<kirin_msgs::srv::ToggleHandState>::SharedFuture;
  auto response_callback = [this](ResponseFuture future) {
    auto response        = future.get();
    this->current_state_ = (response->current_state.value == kirin_msgs::msg::HandState::EXTEND)
                               ? kirin_types::HandState::Extend
                               : kirin_types::HandState::Shrink;
    // RCLCPP_INFO(this->get_logger(), "return : %d", response->current_state.value);
  };

  auto future_result = toggle_hand_state_client_->async_send_request(request, response_callback);
}

void WorldCoordManualController::ChangePumpStateClientRequest() {
  auto request = std::make_shared<kirin_msgs::srv::SetAirState::Request>();

  auto next_state             = !is_air_on;
  request->air_state.left     = next_state;
  request->air_state.right    = next_state;
  request->air_state.top      = next_state;
  request->air_state.ex_left  = (current_state_ == kirin_types::HandState::Extend) && next_state;
  request->air_state.ex_right = (current_state_ == kirin_types::HandState::Extend) && next_state;
  request->air_state.release  = !next_state;

  using ResponseFuture   = rclcpp::Client<kirin_msgs::srv::SetAirState>::SharedFuture;
  auto response_callback = [this, next_state](ResponseFuture future) {
    auto response = future.get();
    if (response->result) this->is_air_on = next_state;
  };

  auto future_result = set_air_state_client_->async_send_request(request, response_callback);
}