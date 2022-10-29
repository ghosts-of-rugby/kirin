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
      move_mode_{kirin_types::MoveMode::Manual},
      timer_callback_(std::bind(&WorldCoordManualController::TimerCallback, this)) {
  /* color direction from field */
  is_red_    = declare_parameter("field", "blue") == "red";
  color_dir_ = is_red_ ? -1.0 : 1.0;

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

  this->RegisterButtonPressedCallback(Button::Start, [this]() -> void {
    // initial auto movement start
    switch (initial_auto_) {
      case InitialAuto::Wait: {
        GoNextInitialAuto();
        RCLCPP_INFO(this->get_logger(), "Start Initial Auto Movement");
        break;
      }
      case InitialAuto::WaitRapidFinished:
      case InitialAuto::WaitPicked:
      case InitialAuto::WaitPlaceAdjustment: {
        GoNextInitialAuto();
        break;
      }
      default: {
        break;
      }
    }
  });

  this->RegisterButtonPressedCallback(Button::Home, [this]() -> void {
    if (!SetNextTarget(frame::kDepart)) return;
    StartZAutoMovement(ZAutoState::Depart);
    StartPlanarMovement();
    if (hand_state_ == kirin_types::HandState::Extend) {
      ChangeHandStateClientRequest();
    }
    RCLCPP_INFO(this->get_logger(), "Go to depart position");
  });

  this->RegisterButtonPressedCallback(Button::RB, [this]() -> void {
    ZAutoState next_z_auto_state
        = (z_auto_state == ZAutoState::Approach) ? ZAutoState::Depart : ZAutoState::Approach;
    if (StartZAutoMovement(next_z_auto_state)) {
      RCLCPP_INFO(this->get_logger(), "Z automatic movement start");
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Z automatic movement cannot start or robot is already moving");
    }
  });

  this->RegisterButtonPressedCallback(Button::LB, [this]() -> void {
    if (!SetNextTarget(next_target_)) return;
    if (!kirin_utils::Contain(next_target_, "share")) {
      if (this->hand_state_ == kirin_types::HandState::Extend) {
        ChangeHandStateClientRequest();
      }
    }
    if (StartPlanarMovement()) {
      RCLCPP_INFO(this->get_logger(), "Planar automatic movement start");
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Planar automatic movement cannot start or robot is already moving");
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
  current_bellows_pub_ = create_publisher<std_msgs::msg::String>("current_bellows_frame", qos);
  next_target_pub_     = create_publisher<std_msgs::msg::String>("next_target", qos);
  all_state_msg_pub_   = create_publisher<std_msgs::msg::String>("all_state", qos);
  move_mode_pub_       = create_publisher<kirin_msgs::msg::MoveMode>("move_mode", qos);
  manual_vel_pub_      = create_publisher<kirin_msgs::msg::HandPosition>("manual_velocity", qos);
  z_fin_sub_           = create_subscription<std_msgs::msg::Bool>(
      "z_finished", qos,
      std::bind(&WorldCoordManualController::ZFinishedCallback, this, std::placeholders::_1));
  planar_fin_sub_ = create_subscription<std_msgs::msg::Bool>(
      "planar_finished", qos,
      std::bind(&WorldCoordManualController::PlanarFinishedCallback, this, std::placeholders::_1));
  timer_ = create_wall_timer(std::chrono::milliseconds(loop_ms_), timer_callback_);
  toggle_hand_state_client_
      = create_client<kirin_msgs::srv::ToggleHandState>("tool/toggle_hand_state");
  set_air_state_client_    = create_client<kirin_msgs::srv::SetAirState>("tool/set_air_state");
  start_rapid_hand_client_ = create_client<kirin_msgs::srv::StartRapidHand>("/start_rapid_hand");
  set_target_client_       = create_client<kirin_msgs::srv::SetTarget>("generator/set_target");
  start_planar_auto_client_
      = create_client<kirin_msgs::srv::StartPlanarAutoMovement>("generator/start_planar_auto");
  start_z_auto_client_
      = create_client<kirin_msgs::srv::StartZAutoMovement>("generator/start_z_auto");

  /* publish initial message */
  PublishModeMsg(move_mode_);
  PublishNextTargetMsg(next_target_);
}

WorldCoordManualController::~WorldCoordManualController() {}

void WorldCoordManualController::TimerCallback() {
  /* InitialAutoMovement */
  if (initial_auto_ != InitialAuto::End) {
    InitialAutoMovement();
  }
  PublishManualInput();

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
      PublishNextTargetMsg(frame::kShareWait);
      SetNextTarget(frame::kShareWait);
      StartPlanarMovement();
      StartZAutoMovement(ZAutoState::Approach);
      GoNextInitialAuto();
      return;
    }
    case InitialAuto::WaitRapidFinished: {
      SetNextInitialAutoCallback([this]() -> void {
        SetNextTarget(frame::pick::kShare2);
        PublishNextTargetMsg(frame::pick::kShare2);
        StartPlanarMovement();
        StartZAutoMovement(ZAutoState::Approach);
        ChangeHandStateClientRequest();
      });
      // wait start button pushed
      return;
    }
    case InitialAuto::GoShare: {
      if (AutomaticMovementFinished()) {
        ChangePumpStateClientRequest();
        GoNextInitialAuto();
      }
      return;
    }
    case InitialAuto::WaitPicked: {
      SetNextInitialAutoCallback([this]() -> void { StartZAutoMovement(ZAutoState::Depart); });
      // wait start button pressed
      return;
    }
    case InitialAuto::ZDepart: {
      if (AutomaticMovementFinished()) {
        if (hand_state_ == kirin_types::HandState::Extend) {
          ChangeHandStateClientRequest();
        }
        SetNextTarget(frame::place::kShare);
        PublishNextTargetMsg(frame::place::kShare);
        StartPlanarMovement();
        GoNextInitialAuto();
      }
      return;
    }
    case InitialAuto::GoPlaceAbove: {
      if (AutomaticMovementFinished()) {
        StartZAutoMovement(ZAutoState::Approach);
        GoNextInitialAuto();
      }
      return;
    }
    case InitialAuto::GoPlaceHeight: {
      if (AutomaticMovementFinished()) {
        GoNextInitialAuto();
      }
      return;
    }
    case InitialAuto::WaitPlaceAdjustment: {
      // wait button pushed
      SetNextInitialAutoCallback([this]() -> void {
        next_target_ = frame::kDepart;
        SetNextTarget(frame::kDepart);
        PublishNextTargetMsg(frame::kDepart);
        StartZAutoMovement(ZAutoState::Depart);
        ChangePumpStateClientRequest();
        pick_index = 2;
      });
      return;
    }
    case InitialAuto::AdjustmentCompleted: {
      // when button pushed
      if (AutomaticMovementFinished()) {
        StartPlanarMovement();
        GoNextInitialAuto();
      }
      return;
    }
    case InitialAuto::End: {
      return;
    }
  }
}

void WorldCoordManualController::GoNextInitialAuto() {
  initial_auto_++;
  if (go_next_initial_auto_callback_) {
    go_next_initial_auto_callback_();
    go_next_initial_auto_callback_ = nullptr;
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
  auto request   = std::make_shared<kirin_msgs::srv::StartRapidHand::Request>();
  request->value = true;

  using ResponseFuture   = rclcpp::Client<kirin_msgs::srv::StartRapidHand>::SharedFuture;
  auto response_callback = [this](ResponseFuture future) {
    auto response = future.get();
    if (!response->result) RCLCPP_ERROR(this->get_logger(), "Failed to Start Rapid Hand!!");
    // RCLCPP_INFO(this->get_logger(), "return : %d", response->current_state.value);
  };

  auto future_result = start_rapid_hand_client_->async_send_request(request, response_callback);
}

bool WorldCoordManualController::SetNextTarget(const std::string& next_target) {
  auto request    = std::make_shared<kirin_msgs::srv::SetTarget::Request>();
  request->target = next_target;

  using ResponseFuture   = rclcpp::Client<kirin_msgs::srv::SetTarget>::SharedFuture;
  auto response_callback = [this](ResponseFuture future) {
    auto response = future.get();
    if (response->result) {
    } else {  // failed
      RCLCPP_ERROR(this->get_logger(), "Failed to Start Next Target!!");
    }
  };

  auto future_result = set_target_client_->async_send_request(request, response_callback);
  return true;
}

bool WorldCoordManualController::StartZAutoMovement(const kirin_types::ZAutoState& state) {
  auto request = std::make_shared<kirin_msgs::srv::StartZAutoMovement::Request>();
  kirin_msgs::msg::ZAutoState state_msg;
  state_msg.value       = static_cast<int32_t>(state);
  request->z_auto_state = state_msg;

  using ResponseFuture   = rclcpp::Client<kirin_msgs::srv::StartZAutoMovement>::SharedFuture;
  auto response_callback = [this, state](ResponseFuture future) {
    auto response = future.get();
    if (response->result) {
      this->is_z_moving = true;
      this->z_auto_state = state;
    } else {  // failed
      RCLCPP_ERROR(this->get_logger(), "Failed to Start Z Auto Movement!!");
    }
  };

  auto future_result = start_z_auto_client_->async_send_request(request, response_callback);
  return true;
}

bool WorldCoordManualController::StartPlanarMovement() {
  auto request   = std::make_shared<kirin_msgs::srv::StartPlanarAutoMovement::Request>();
  request->value = true;

  using ResponseFuture   = rclcpp::Client<kirin_msgs::srv::StartPlanarAutoMovement>::SharedFuture;
  auto response_callback = [this](ResponseFuture future) {
    auto response = future.get();
    if (response->result) {
      this->is_planar_moving = true;
    } else {  // failed
      RCLCPP_ERROR(this->get_logger(), "Failed to Start Planar Auto Movement!!");
    }
  };

  auto future_result = start_planar_auto_client_->async_send_request(request, response_callback);
  return true;
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
  z_auto_msg += magic_enum::enum_name(z_auto_state);

  // initial auto movement
  std::string initial_auto_msg = "Initial Auto: ";
  initial_auto_msg += magic_enum::enum_name(initial_auto_);

  std_msgs::msg::String msg;
  msg.data = pump_msg + "\n" + hand_msg + "\n" + z_auto_msg + "\n\n" + initial_auto_msg;
  all_state_msg_pub_->publish(std::move(msg));
}

void WorldCoordManualController::PublishManualInput() {
  auto msg          = std::make_unique<kirin_msgs::msg::HandPosition>();
  msg->point.x      = this->GetAxis(JoyController::Axis::LStickX);
  msg->point.y      = this->GetAxis(JoyController::Axis::LStickY);
  msg->point.z      = this->GetAxis(JoyController::Axis::RStickX);
  double left_trig  = 1.0 - this->GetAxis(JoyController::Axis::LTrigger);
  double right_trig = 1.0 - this->GetAxis(JoyController::Axis::RTrigger);
  msg->psi          = (left_trig - right_trig);
  manual_vel_pub_->publish(std::move(msg));
}

void WorldCoordManualController::PlanarFinishedCallback(
    [[maybe_unused]] const std_msgs::msg::Bool::UniquePtr msg) {
  is_planar_moving = false;
}

void WorldCoordManualController::ZFinishedCallback(const std_msgs::msg::Bool::UniquePtr msg) {
  is_z_moving = false;
}

void WorldCoordManualController::SetNextInitialAutoCallback(const std::function<void()>& callback) {
  go_next_initial_auto_callback_ = callback;
}

bool WorldCoordManualController::AutomaticMovementFinished() {
  return !is_planar_moving && !is_z_moving;
}