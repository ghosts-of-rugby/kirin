#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include <kirin_msgs/srv/toggle_hand_state.hpp>
#include <kirin_msgs/srv/set_air_state.hpp>
#include "kirin/world_coord_manual_controller.hpp"
#include "kirin/frame.hpp"
#include "kirin/machine.hpp"
#include "kirin/utils.hpp"
#include <iostream>
#include <algorithm>

using namespace std::chrono_literals;

WorldCoordManualController::WorldCoordManualController(const std::string& node_name,
                                                       const rclcpp::NodeOptions& options)
    : JoyController(node_name, options),
      initial_pos_(machine::kRedInitialPosX, machine::kRedInitialPosY, machine::kRedInitialPosZ),
      pos_(initial_pos_),
      vel_(0.0, 0.0, 0.0),
      initial_psi_(machine::kRedInitialPsi),
      psi_(initial_psi_),
      dpsi_(0.0),
      current_bellows_frame_{frame::kBellowsTop},
      move_mode_{kirin_types::MoveMode::Manual},
      timer_callback_(std::bind(&WorldCoordManualController::TimerCallback, this)) {
  /* color direction from field */
  is_red_    = declare_parameter("field", "blue") == "red";
  color_dir_ = is_red_ ? -1.0 : 1.0;
  if (!is_red_) {  // if blue, fix initial position
    pos_.y()         = -1.0 * pos_.y();
    psi_             = -1.0 * psi_;
    initial_pos_.y() = -1.0 * initial_pos_.y();
    initial_psi_     = -1.0 * initial_psi_;
  }

  /* get parameter */
  velocity_ratio_normal_ = {.x   = declare_parameter("velocity_ratio.normal.x", 0.0),
                            .y   = declare_parameter("velocity_ratio.normal.y", 0.0),
                            .z   = declare_parameter("velocity_ratio.normal.z", 0.0),
                            .psi = declare_parameter("velocity_ratio.normal.psi", 0.0)};
  velocity_ratio_adjust_ = {.x   = declare_parameter("velocity_ratio.adjust.x", 0.0),
                            .y   = declare_parameter("velocity_ratio.adjust.y", 0.0),
                            .z   = declare_parameter("velocity_ratio.adjust.z", 0.0),
                            .psi = declare_parameter("velocity_ratio.adjust.psi", 0.0)};

  z_auto_      = {.ratio           = declare_parameter("z_auto.ratio", 2.0),
                  .max_speed       = declare_parameter("z_auto.max_speed", 0.3),
                  .approach_offset = declare_parameter("z_auto.approach_offset", 0.05)};
  planar_auto_ = {.xy_ratio      = declare_parameter("xy_auto.ratio", 2.0),
                  .xy_max_speed  = declare_parameter("xy_auto.max_speed", 0.3),
                  .psi_ratio     = declare_parameter("psi_auto.ratio", 2.0),
                  .psi_max_speed = declare_parameter("psi_auto.max_speed", 0.3)};

  // transform listener
  tf_buffer_          = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  {
    // wait transform published from robot_state_publiser and hand_tool_manager
    // TODO use lifecycle or not
    std::this_thread::sleep_for(300ms);
  }

  /* define target arrary */
  pick_target_  = {frame::pick::kShare2, frame::pick::kShare1, frame::kDepart,    frame::pick::k1st,
                   frame::pick::k2nd,    frame::pick::k3rd,    frame::pick::k4th, frame::pick::k5th};
  place_target_ = {frame::kDepart,     frame::place::kShare, frame::place::k1st, frame::place::k2nd,
                   frame::place::k3rd, frame::place::k4th,   frame::place::k5th};

  /* register callback when some actions occur */
  this->RegisterButtonPressedCallback(
      Button::X, std::bind(&WorldCoordManualController::ChangeHandStateClientRequest, this));

  this->RegisterButtonPressedCallback(
      Button::A, std::bind(&WorldCoordManualController::ChangePumpStateClientRequest, this));

  // this->RegisterButtonPressedCallback(
  //     Button::Home, std::bind(&WorldCoordManualController::ModeChangeHandler, this));
  this->RegisterButtonPressedCallback(Button::Start, [this]() -> void {
    // initial auto movement start
    if (initial_auto_ == InitialAuto::Wait) {
      initial_auto_ = InitialAuto::Start;
      RCLCPP_INFO(this->get_logger(), "Start Initial Auto Movement");
    } else if (initial_auto_ == InitialAuto::WaitRapidFinished) {
      initial_auto_ = InitialAuto::RapidFinished;
    } else if (initial_auto_ == InitialAuto::WaitPicked) {
      initial_auto_ = InitialAuto::Picked;
    } else if (initial_auto_ == InitialAuto::WaitPlaceAdjustment) {
      if (!is_air_on_) initial_auto_ = InitialAuto::AdjustmentCompleted;
    }
  });

  this->RegisterButtonPressedCallback(Button::Home, [this]() -> void {
    // if robot is not in auto movement and pump is not on
    if (!planar_auto_.enabled && !z_auto_.enabled) {
      next_target_         = frame::kDepart;
      current_target_      = frame::kDepart;
      z_auto_.enabled      = true;
      z_auto_.state        = ZAutoState::Depart;
      planar_auto_.enabled = true;
      RCLCPP_INFO(this->get_logger(), "Go to depart position");
    }
  });

  this->RegisterButtonPressedCallback(Button::RB, [this]() -> void {
    z_auto_.enabled = true;
    if (!planar_auto_.enabled) {  // disable to change target during planar movement
      // movement from pick to place or from place to pick is invalid
      ValidateAndUpdateTarget();
      current_target_ = next_target_;
    }
    // change z auto state
    if (current_target_ == frame::kDepart) z_auto_.state = ZAutoState::Depart;
    else {
      if (z_auto_.state == ZAutoState::Approach) z_auto_.state = ZAutoState::Depart;
      else z_auto_.state = ZAutoState::Approach;
    }
    RCLCPP_INFO(this->get_logger(), "Z auto movement start");
  });

  this->RegisterButtonPressedCallback(Button::LB, [this]() -> void {
    if (!planar_auto_.enabled || IsAllowedToChangeTarget()) {
      // movement from pick to place or from place to pick is invalid
      ValidateAndUpdateTarget();
      current_target_      = next_target_;
      planar_auto_.enabled = true;
      RCLCPP_INFO(this->get_logger(), "planar auto movement start");
    }
  });

  // crossx and crossy change next target
  this->RegisterAxisChangedCallback(Axis::CrossX, [this](double pre, double new_value) -> void {
    /* execute process when value jumped from 0.0 to -1.0 or 1.0 */
    if (std::abs(new_value) != 1.0) return;
    int input = new_value * -1;
    place_index += input;
    place_index  = std::clamp(place_index, 0, place_max_index);
    next_target_ = place_target_.at(place_index);
    RCLCPP_INFO(this->get_logger(), "curent place index: %d, target name: %s", place_index,
                next_target_.c_str());
    PublishNextTargetMsg(next_target_);
  });

  this->RegisterAxisChangedCallback(Axis::CrossY, [this](double pre, double new_value) -> void {
    /* execute process when value jumped from 0.0 to -1.0 or 1.0 */
    if (std::abs(new_value) != 1.0) return;
    int input = new_value * color_dir_;
    pick_index += input;
    pick_index   = std::clamp(pick_index, 0, pick_max_index);
    next_target_ = pick_target_.at(pick_index);
    RCLCPP_INFO(this->get_logger(), "curent pick index: %d, target name: %s", pick_index,
                next_target_.c_str());
    PublishNextTargetMsg(next_target_);
  });

  // this->RegisterButtonPressedCallback(
  //   Button::X, std::bind(&WorldCoordManualController::ChangeBellows, this));

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  world_coord_pub_     = create_publisher<geometry_msgs::msg::PoseStamped>(input_topic_name_, qos);
  joint_pub_           = create_publisher<sensor_msgs::msg::JointState>("manual_joint", qos);
  current_bellows_pub_ = create_publisher<std_msgs::msg::String>("current_bellows_frame", qos);
  next_target_pub_     = create_publisher<std_msgs::msg::String>("next_target", qos);
  all_state_msg_pub_   = create_publisher<std_msgs::msg::String>("all_state", qos);
  move_mode_pub_       = create_publisher<kirin_msgs::msg::MoveMode>("move_mode", qos);
  timer_               = create_wall_timer(std::chrono::milliseconds(loop_ms_), timer_callback_);
  toggle_hand_state_client_
      = create_client<kirin_msgs::srv::ToggleHandState>("tool/toggle_hand_state");
  set_air_state_client_ = create_client<kirin_msgs::srv::SetAirState>("tool/set_air_state");
  start_rapid_hand_client_ = create_client<kirin_msgs::srv::StartRapidHand>("/start_rapid_hand");

  /* publish initial message */
  PublishBellowsMsg(current_bellows_frame_);
  PublishModeMsg(move_mode_);
  PublishNextTargetMsg(next_target_);
}

WorldCoordManualController::~WorldCoordManualController() {}

bool WorldCoordManualController::IsAllowedToChangeTarget() {
  double depart_xy_threshold  = 0.3;
  double depart_psi_threshold = 0.2;
  if (current_target_ == frame::kDepart) {
    if (xy_distance_.has_value() && psi_distance_.has_value()) {
      return (xy_distance_->norm() <= depart_xy_threshold
              && psi_distance_.value() <= depart_psi_threshold);
    } else {
      return false;
    }
  } else {
    return false;
  }
}

geometry_msgs::msg::Pose WorldCoordManualController::GetManualPose() {
  /* input manual velocity */
  geometry_msgs::msg::Pose pose;
  auto velocity_ratio
      = (z_auto_.state == ZAutoState::Approach) ? velocity_ratio_adjust_ : velocity_ratio_normal_;

  /* get manual velocity */
  vel_ = Eigen::Vector3d(this->GetAxis(JoyController::Axis::LStickX) * 1.0 * velocity_ratio.x,
                         this->GetAxis(JoyController::Axis::LStickY) * 1.0 * velocity_ratio.y,
                         this->GetAxis(JoyController::Axis::RStickX) * 1.0 * velocity_ratio.z);
  double left_trig  = 1.0 - this->GetAxis(JoyController::Axis::LTrigger);
  double right_trig = 1.0 - this->GetAxis(JoyController::Axis::RTrigger);
  dpsi_             = (left_trig - right_trig) * velocity_ratio.psi;

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
    auto z_auto_input = GenerateAutoZVelocity(target, offset);
    if (z_auto_input.has_value()) {
      auto [z_vel, z_distance] = z_auto_input.value();
      // RCLCPP_INFO(this->get_logger(), "z auto input: %f, z_distance: %f", z_vel, z_distance);
      vel_.z()                 = z_vel;
      if (std::abs(z_distance - sgn(z_distance) * offset) <= 0.005) {
        z_auto_.enabled = false;
        RCLCPP_INFO(this->get_logger(), "z auto movement finished!");
      }
    }
  }

  /* if planar auto movement, overwrite x y psi velocity */
  if (planar_auto_.enabled) {
    auto planar_auto_input = GenerateAutoPlaneVelocity(current_target_);
    if (planar_auto_input.has_value()) {
      auto [plane_vel, plane_distance]      = planar_auto_input.value();
      auto [xy_vel, psi_vel]                = plane_vel;
      std::tie(xy_distance_, psi_distance_) = plane_distance;
      // RCLCPP_INFO(this->get_logger(),
      //             "x y psi auto input: [ %f, %f, %f ], x y psi distance: [ %f, %f, %f ]",
      //             xy_vel.x(), xy_vel.y(), psi_vel, xy_distance.x(), xy_distance.y(),
      //             psi_distance);
      vel_.x()                              = xy_vel.x();
      vel_.y()                              = xy_vel.y();
      dpsi_                                 = psi_vel;
      double xy_threshold                   = (current_target_ == frame::kDepart) ? 0.05 : 0.002;
      double psi_threshold                  = (current_target_ == frame::kDepart) ? 0.1 : 0.03;
      if (xy_distance_->norm() <= xy_threshold
          && std::abs(psi_distance_.value()) <= psi_threshold) {
        planar_auto_.enabled = false;
        RCLCPP_INFO(this->get_logger(), "planar auto movement finished!");
      }
    }
  } else {
    xy_distance_  = std::nullopt;
    psi_distance_ = std::nullopt;
  }

  pos_ += vel_ * loop_ms_ * 0.001;
  pose.position = Eigen::toMsg(pos_);

  psi_ += dpsi_ * loop_ms_ * 0.001;
  Eigen::Quaterniond quat(Eigen::AngleAxisd(psi_, Eigen::Vector3d::UnitZ()));
  pose.orientation = Eigen::toMsg(quat);

  return std::move(pose);
}

std::optional<geometry_msgs::msg::Pose> WorldCoordManualController::GetPoseFromTf(
    const std::string& parent_frame, const std::string& child_frame) {
  return kirin_utils::GetPoseFromTf(this->get_logger(), this->tf_buffer_, parent_frame,
                                    child_frame);
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

  /* publish current bellows */
  current_bellows_frame_ = bellows_frame;
  PublishBellowsMsg(current_bellows_frame_);

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

  /*  Offset between joint displacement and absolute position in the joint coordinate system */
  double r_offset = machine::kROffsetCenterToRRoot + machine::kROffsetRRootToPhi;
  double z_offset = initial_pos_.z() - machine::kZOffsetInitialDisplacement;

  /* calculate absolute position in the joint coordinate */
  double r     = model::CalcR(l, pos_.x(), pos_.y(), psi_);
  double theta = model::CalcTheta(l, pos_.x(), pos_.y(), psi_);
  double phi   = model::CalcPhi(l, pos_.x(), pos_.y(), psi_);

  double dr     = model::CalcRVel(l, vel_.x(), vel_.y(), dpsi_, r, theta, phi);
  double dtheta = model::CalcThetaVel(l, vel_.x(), vel_.y(), dpsi_, r, theta, phi);
  double dphi   = model::CalcPhiVel(l, vel_.x(), vel_.y(), dpsi_, r, theta, phi);

  /* calculate joint displacement */
  joint_state->position
      = {theta, pos_.z() - z_offset, r - r_offset, phi - phi_offset, phi - phi_offset};
  joint_state->velocity = {dtheta, vel_.z(), dr, dphi, dphi};
  joint_pub_->publish(std::move(joint_state));
}

void WorldCoordManualController::TimerCallback() {
  /* InitialAutoMovement */
  if (initial_auto_ != InitialAuto::End) {
    InitialAutoMovement();
  }

  // calculate length from base to current bellows
  auto&& bellows_in_phi_link = GetPoseFromTf(frame::kPhiLink, current_bellows_frame_);
  double l, phi_offset;
  if (bellows_in_phi_link) {
    double x   = bellows_in_phi_link.value().position.x;
    double y   = bellows_in_phi_link.value().position.y;
    l          = sqrt(x * x + y * y);
    phi_offset = atan2(y, x);
  } else {
    return;
  }

  auto input_pose = std::make_unique<geometry_msgs::msg::PoseStamped>();

  input_pose->header.frame_id = frame::kFixBase;
  input_pose->header.stamp    = get_clock()->now();
  input_pose->pose            = GetManualPose();
  world_coord_pub_->publish(std::move(input_pose));

  PublishJointState(l, phi_offset);

  // publish all state
  PublishAllStateMsg();
}

void WorldCoordManualController::InitialAutoMovement() {
  switch (initial_auto_) {
    case InitialAuto::Wait: {
      // wait start button pushed
      PublishNextTargetMsg(frame::kInitialDepart);
      return;
    }
    case InitialAuto::Start: {
      StartRapidHandClientRequest();
      next_target_    = frame::kInitialDepart;
      current_target_ = frame::kInitialDepart;
      PublishNextTargetMsg(next_target_);
      planar_auto_.enabled = true;
      z_auto_.enabled      = true;
      initial_auto_        = InitialAuto::GoInitialDepart;
      return;
    }
    case InitialAuto::GoInitialDepart: {
      if (!planar_auto_.enabled) {  // when initial depart reached
        next_target_    = frame::kShareWait;
        current_target_ = frame::kShareWait;
        PublishNextTargetMsg(next_target_);
        planar_auto_.enabled = true;
        initial_auto_        = InitialAuto::GoShareWait;
      }
      return;
    }
    case InitialAuto::GoShareWait: {
      if (!planar_auto_.enabled) {  // when share waith reached
        initial_auto_ = InitialAuto::WaitRapidFinished;
        PublishNextTargetMsg(frame::pick::kShare2);
      }
      return;
    }
    case InitialAuto::WaitRapidFinished: {
      // wait start button pushed
      PublishNextTargetMsg(frame::pick::kShare2);
      return;
    }
    case InitialAuto::RapidFinished: {
      next_target_    = frame::pick::kShare2;
      current_target_ = frame::pick::kShare2;
      PublishNextTargetMsg(next_target_);
      planar_auto_.enabled = true;
      z_auto_.enabled      = true;
      z_auto_.state        = ZAutoState::Approach;
      ChangeHandStateClientRequest();
      initial_auto_ = InitialAuto::GoShare;
      return;
    }
    case InitialAuto::GoShare: {
      if (!planar_auto_.enabled && !z_auto_.enabled) {
        next_target_ = frame::kDepart;
        PublishNextTargetMsg(next_target_);
        ChangePumpStateClientRequest();
        initial_auto_ = InitialAuto::WaitPicked;
      }
      return;
    }
    case InitialAuto::WaitPicked: {
      // wait start button pressed
      return;
    }
    case InitialAuto::Picked: {
      // when start button pressed
      z_auto_.enabled = true;
      z_auto_.state   = ZAutoState::Depart;
      current_target_ = frame::kDepart;
      initial_auto_   = InitialAuto::GoDepartZ;
      return;
    }
    case InitialAuto::GoDepartZ: {
      if (!z_auto_.enabled) {  // if z movement finished, start xy movement
        planar_auto_.enabled = true;
        ChangeHandStateClientRequest();
        initial_auto_ = InitialAuto::GoDepartXY;
      }
      return;
    }
    case InitialAuto::GoDepartXY: {
      if (!planar_auto_.enabled || IsAllowedToChangeTarget()) {  // if planar movement finished, Go place point
        current_target_      = frame::place::kShare;
        next_target_         = frame::place::kShare;
        planar_auto_.enabled = true;
        initial_auto_        = InitialAuto::GoPlaceAbove;
        PublishNextTargetMsg(next_target_);
      }
      return;
    }
    case InitialAuto::GoPlaceAbove: {
      if (!planar_auto_.enabled) {
        z_auto_.enabled = true;
        z_auto_.state   = ZAutoState::Approach;
        initial_auto_   = InitialAuto::GoPlaceHeight;
      }
      return;
    }
    case InitialAuto::GoPlaceHeight: {
      if (!z_auto_.enabled) {
        initial_auto_ = InitialAuto::WaitPlaceAdjustment;
      }
      return;
    }
    case InitialAuto::WaitPlaceAdjustment: {
      // wait button pushed
      return;
    }
    case InitialAuto::AdjustmentCompleted: {
      // when button pushed
      current_target_      = frame::kDepart;
      next_target_         = frame::kDepart;
      planar_auto_.enabled = true;
      z_auto_.enabled      = true;
      z_auto_.state        = ZAutoState::Depart;
      initial_auto_        = InitialAuto::GoStandby;
      PublishNextTargetMsg(next_target_);
      return;
    }
    case InitialAuto::GoStandby: {
      if (!planar_auto_.enabled && !z_auto_.enabled) {
        initial_auto_ = InitialAuto::End;
      }
      return;
    }
    case InitialAuto::End: {
      return;
    }
  }
}

void WorldCoordManualController::ChangeHandStateClientRequest() {
  auto request    = std::make_shared<kirin_msgs::srv::ToggleHandState::Request>();
  request->toggle = true;

  using ResponseFuture   = rclcpp::Client<kirin_msgs::srv::ToggleHandState>::SharedFuture;
  auto response_callback = [this](ResponseFuture future) {
    auto response     = future.get();
    this->hand_state_ = (response->current_state.value == kirin_msgs::msg::HandState::EXTEND)
                            ? kirin_types::HandState::Extend
                            : kirin_types::HandState::Shrink;
    // RCLCPP_INFO(this->get_logger(), "return : %d", response->current_state.value);
  };

  auto future_result = toggle_hand_state_client_->async_send_request(request, response_callback);
}

void WorldCoordManualController::ChangePumpStateClientRequest() {
  auto request = std::make_shared<kirin_msgs::srv::SetAirState::Request>();

  auto next_state             = !is_air_on_;
  request->air_state.left     = next_state;
  request->air_state.right    = next_state;
  request->air_state.top      = next_state;
  request->air_state.ex_left  = (hand_state_ == kirin_types::HandState::Extend) && next_state;
  request->air_state.ex_right = (hand_state_ == kirin_types::HandState::Extend) && next_state;
  request->air_state.release  = !next_state;

  using ResponseFuture   = rclcpp::Client<kirin_msgs::srv::SetAirState>::SharedFuture;
  auto response_callback = [this, next_state](ResponseFuture future) {
    auto response = future.get();
    if (response->result) this->is_air_on_ = next_state;
  };

  auto future_result = set_air_state_client_->async_send_request(request, response_callback);
}

void WorldCoordManualController::StartRapidHandClientRequest() {
  auto request    = std::make_shared<kirin_msgs::srv::StartRapidHand::Request>();
  request->value = true;

  using ResponseFuture   = rclcpp::Client<kirin_msgs::srv::StartRapidHand>::SharedFuture;
  auto response_callback = [this](ResponseFuture future) {
    auto response     = future.get();
    if (!response->result) RCLCPP_ERROR(this->get_logger(), "Failed to Start Rapid Hand!!");
    // RCLCPP_INFO(this->get_logger(), "return : %d", response->current_state.value);
  };

  auto future_result = start_rapid_hand_client_->async_send_request(request, response_callback);

}

void WorldCoordManualController::ModeChangeHandler() {
  using Mode     = kirin_types::MoveMode;
  auto next_mode = (move_mode_ == Mode::Auto) ? Mode::Manual : Mode::Auto;
  RCLCPP_INFO(this->get_logger(), "MoveMode: '%s' -> '%s'",
              magic_enum::enum_name(this->move_mode_).data(),
              magic_enum::enum_name(next_mode).data());
  move_mode_ = next_mode;

  PublishModeMsg(move_mode_);
}

void WorldCoordManualController::PublishBellowsMsg(const std::string& bellows) {
  std_msgs::msg::String bellows_msg;
  bellows_msg.data = bellows;
  current_bellows_pub_->publish(std::move(bellows_msg));
}

void WorldCoordManualController::PublishNextTargetMsg(const std::string& next_target) {
  std_msgs::msg::String next_target_msg;
  next_target_msg.data = next_target;
  next_target_pub_->publish(std::move(next_target_msg));
}

void WorldCoordManualController::PublishModeMsg(const kirin_types::MoveMode& mode) {
  kirin_msgs::msg::MoveMode msg;
  if (mode == kirin_types::MoveMode::Auto) msg.mode = kirin_msgs::msg::MoveMode::AUTO;
  else if (mode == kirin_types::MoveMode::Manual) msg.mode = kirin_msgs::msg::MoveMode::MANUAL;
  else /* mode == kirin_types::MoveMode::Stop) */ msg.mode = kirin_msgs::msg::MoveMode::STOP;
  move_mode_pub_->publish(std::move(msg));
}

void WorldCoordManualController::PublishAllStateMsg() {
  // pump
  std::string pump_msg = "(Button [A] ) AirPumpState: ";
  pump_msg += is_air_on_ ? "ON" : "OFF";

  // hand
  std::string hand_msg = "(Button [X] ) HandState   : ";
  hand_msg += magic_enum::enum_name(hand_state_);

  // zauto
  std::string z_auto_msg = "(Button [RB]) ZAutoState : ";
  z_auto_msg += magic_enum::enum_name(z_auto_.state);

  // initial auto movement
  std::string initial_auto_msg = "Initial Auto: ";
  initial_auto_msg += magic_enum::enum_name(initial_auto_);

  std_msgs::msg::String msg;
  msg.data = pump_msg + "\n" + hand_msg + "\n" + z_auto_msg + "\n\n" + initial_auto_msg;
  all_state_msg_pub_->publish(std::move(msg));
}

std::optional<std::tuple<double, double>> WorldCoordManualController::GenerateAutoZVelocity(
    const std::string& target, double offset) {
  auto pose = GetPoseFromTf(current_bellows_frame_, target);
  if (pose.has_value()) {
    double z_distance  = pose.value().position.z;
    double input_z_vel = z_auto_.ratio * (z_distance - sgn(z_distance) * offset);
    return std::tie(std::clamp(input_z_vel, -z_auto_.max_speed, z_auto_.max_speed), z_distance);
  } else {
    return std::nullopt;
  }
}

using PlaneTuple = WorldCoordManualController::PlaneTuple;
std::optional<std::tuple<PlaneTuple, PlaneTuple>>
WorldCoordManualController::GenerateAutoPlaneVelocity(const std::string& target) {
  // bellowsから見たtargetの距離を計算
  auto distance         = GetPoseFromTf(current_bellows_frame_, target);
  // コントローラーの速度入力はロボット座標系でのxyなので入力の回転変換用のPoseを取得
  auto bellows_in_robot = GetPoseFromTf(frame::kFixBase, current_bellows_frame_);
  if (distance.has_value() && bellows_in_robot.has_value()) {
    Eigen::Vector2d xy_distance;
    {
      auto&& xy_distance_in_bellows
          = Eigen::Vector2d(distance.value().position.x, distance.value().position.y);
      auto [r_tmp, p_tmp, yaw_bellows_in_robot]
          = CalcGeometryQuatToRPY(bellows_in_robot.value().orientation);
      Eigen::Rotation2Dd rot(yaw_bellows_in_robot);
      xy_distance = rot.matrix() * xy_distance_in_bellows;
    }

    auto [roll, pitch, psi_distance] = CalcGeometryQuatToRPY(distance.value().orientation);

    auto&& input_xy_vel   = planar_auto_.xy_ratio * xy_distance;
    double input_psi_vel  = planar_auto_.psi_ratio * psi_distance;
    auto&& clamped_xy_vel = Eigen::Vector2d(
        std::clamp(input_xy_vel.x(), -planar_auto_.xy_max_speed, planar_auto_.xy_max_speed),
        std::clamp(input_xy_vel.y(), -planar_auto_.xy_max_speed, planar_auto_.xy_max_speed));
    double clamped_psi_vel
        = std::clamp(input_psi_vel, -planar_auto_.psi_max_speed, planar_auto_.psi_max_speed);
    PlaneTuple vel_tuple      = std::tie(clamped_xy_vel, clamped_psi_vel);
    PlaneTuple distance_tuple = std::tie(xy_distance, psi_distance);
    return std::tie(vel_tuple, distance_tuple);
  } else {
    return std::nullopt;
  }
}

void WorldCoordManualController::ValidateAndUpdateTarget() {
  if ((kirin_utils::Contain(current_target_, "pick")
       && kirin_utils::Contain(next_target_, "place"))) {
    next_target_ = frame::kDepart;
    RCLCPP_ERROR(this->get_logger(),
                 "move from pick to place directly is invalid! Go through depart position!");
    PublishNextTargetMsg(next_target_);
  } else if (kirin_utils::Contain(current_target_, "place")
             && kirin_utils::Contain(next_target_, "pick")) {
    next_target_ = frame::kDepart;
    RCLCPP_ERROR(this->get_logger(),
                 "move from place to pick directly is invalid! Go through depart position!");
    PublishNextTargetMsg(next_target_);
  }
}
