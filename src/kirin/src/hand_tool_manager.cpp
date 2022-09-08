#include "kirin/hand_tool_manager.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <magic_enum.hpp>

#include "kirin/frame.hpp"

using namespace std::chrono_literals;

HandToolManager::HandToolManager(const rclcpp::NodeOptions& options)
    : Node("hand_tool_manager", options),
      hand_state_(HandState::Shrink),
      marker_timer_callback_(std::bind(&HandToolManager::MarkerTimerCallback, this)),
      handle_set_hand_state_(std::bind(&HandToolManager::SetHandStateCallback, this,
                                       std::placeholders::_1, std::placeholders::_2,
                                       std::placeholders::_3)),
      handle_toggle_hand_state_(std::bind(&HandToolManager::ToggleHandStateCallback, this,
                                          std::placeholders::_1, std::placeholders::_2,
                                          std::placeholders::_3)),
      handle_set_air_state_(std::bind(&HandToolManager::SetAirStateCallback, this,
                                      std::placeholders::_1, std::placeholders::_2,
                                      std::placeholders::_3)) {
  std::string mesh_directory = "package://kirin/resources/light";
  resource_map_ = {{HandState::Shrink, mesh_directory + "/phi.stl"},
                   {HandState::Extend, mesh_directory + "/phi_extend.stl"}};

  double top_x = 0.05;
  double shrink_d = 0.10;
  double extend_d = 0.14;
  double bellows_z = -0.042;
  bellows_map_ = {{HandState::Shrink,
                   {BellowsPositionTuple{frame::kBellowsTop, {top_x, 0.0}},
                    BellowsPositionTuple{frame::kBellowsLeft, {top_x - shrink_d, shrink_d}},
                    BellowsPositionTuple{frame::kBellowsRight, {top_x - shrink_d, -shrink_d}}}},
                  {HandState::Extend,
                   {BellowsPositionTuple{frame::kBellowsTop, {top_x, 0.0}},
                    BellowsPositionTuple{frame::kBellowsLeft, {top_x, extend_d}},
                    BellowsPositionTuple{frame::kBellowsRight, {top_x, -extend_d}}}}};
  transform_vec_.reserve(3);
  UpdateBellowsTransformVector(this->hand_state_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_broadcaster_->sendTransform(transform_vec_);

  air_map_ = {
      {kirin_type::BellowsName::Top, kirin_type::AirState::Off},
      {kirin_type::BellowsName::Left, kirin_type::AirState::Off},
      {kirin_type::BellowsName::Right, kirin_type::AirState::Off},
      {kirin_type::BellowsName::ExLeft, kirin_type::AirState::Off},
      {kirin_type::BellowsName::ExRight, kirin_type::AirState::Off},
  };

  marker_pub_ = create_publisher<Marker>("hand_marker", rclcpp::SystemDefaultsQoS());
  timer_ = create_wall_timer(10ms, marker_timer_callback_);
  set_hand_srv_ = create_service<SetHandState>("tool/set_hand_state", handle_set_hand_state_);
  toggle_hand_srv_ =
      create_service<ToggleHandState>("tool/toggle_hand_state", handle_toggle_hand_state_);
  set_air_srv_ = create_service<SetAirState>("tool/set_air_state", handle_set_air_state_);
}

void HandToolManager::MarkerTimerCallback() {
  auto marker_msg = std::make_unique<HandToolManager::Marker>();
  auto now = get_clock()->now();
  marker_msg->header.frame_id = frame::kPhiLink;
  marker_msg->header.stamp = now;

  marker_msg->id = 1;
  marker_msg->ns = "tool";
  marker_msg->type = Marker::MESH_RESOURCE;
  marker_msg->action = Marker::ADD;
  marker_msg->mesh_resource = resource_map_.at(hand_state_);

  marker_msg->pose.position.x = 0;
  marker_msg->pose.position.y = 0;
  marker_msg->pose.position.z = 0;
  marker_msg->pose.orientation.w = 1.0;
  marker_msg->pose.orientation.x = 0.0;
  marker_msg->pose.orientation.y = 0.0;
  marker_msg->pose.orientation.z = 0.0;
  marker_msg->scale.x = 1.0;
  marker_msg->scale.y = 1.0;
  marker_msg->scale.z = 1.0;

  marker_msg->color.r = 0.8;
  marker_msg->color.g = 0.3;
  marker_msg->color.b = 0.3;
  marker_msg->color.a = 1.0;

  marker_pub_->publish(std::move(marker_msg));

  // publish bellows tf
  for (auto& t : transform_vec_) t.header.stamp = now;
  tf_broadcaster_->sendTransform(transform_vec_);
}

void HandToolManager::SetHandStateCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                           const std::shared_ptr<SetHandState::Request> request,
                                           std::shared_ptr<SetHandState::Response> response) {
  (void)request_header;  // Lint Tool 対策
  auto next_state = magic_enum::enum_cast<HandState>(request->hand_state.value);

  if (next_state.has_value()) {
    RCLCPP_INFO(this->get_logger(), "ToolState: '%s' -> '%s'",
                magic_enum::enum_name(this->hand_state_).data(),
                magic_enum::enum_name(next_state.value()).data());
    this->hand_state_ = next_state.value();
    response->result = true;
  }
}

void HandToolManager::SetAirStateCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                          const std::shared_ptr<SetAirState::Request> request,
                                          std::shared_ptr<SetAirState::Response> response) {
  using Bellows = kirin_type::BellowsName;
  using Air = kirin_type::AirState;

  (void)request_header;  // Lint Tool 対策

  std::unordered_map<Bellows, Air> next_state{
      {Bellows::Top, static_cast<Air>(request->air_state.top)},
      {Bellows::Left, static_cast<Air>(request->air_state.left)},
      {Bellows::Right, static_cast<Air>(request->air_state.right)},
      {Bellows::ExLeft, static_cast<Air>(request->air_state.ex_left)},
      {Bellows::ExRight, static_cast<Air>(request->air_state.ex_right)},
  };

  bool release = std::all_of(next_state.begin(), next_state.end(), [](std::pair<Bellows, Air> pair){
    return pair.second == Air::Off;
  });

  /* Send Uart to Arduino */
  if(release) {
    /* release */
  } else {
    /* vacuum */
  }

  for(const auto& [key, next_value] : next_state) {
    if(air_map_.at(key) != next_value) {
      RCLCPP_INFO(this->get_logger(), "AirState [%s]: '%s' -> '%s'",
                  magic_enum::enum_name(key).data(),
                  magic_enum::enum_name(air_map_.at(key)).data(),
                  magic_enum::enum_name(next_value).data());
    }
  }

  air_map_ = next_state;

  response->result = true;
}

void HandToolManager::UpdateBellowsTransformVector(HandState hand_state) {
  double bellows_z = -0.042;
  transform_vec_.clear();
  for (const auto& [bellows_frame, pos] : bellows_map_.at(this->hand_state_)) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = frame::kPhiLink;
    t.child_frame_id = bellows_frame;

    t.transform.translation.x = pos.x();
    t.transform.translation.y = pos.y();
    t.transform.translation.z = bellows_z;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;
    transform_vec_.push_back(std::move(t));
  }
}

void HandToolManager::ToggleHandStateCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ToggleHandState::Request> request,
    std::shared_ptr<ToggleHandState::Response> response) {
  (void)request_header;  // Lint Tool 対策
  auto next_state =
      (this->hand_state_ == HandState::Extend ? HandState::Shrink : HandState::Extend);

  RCLCPP_INFO(this->get_logger(), "ToolState: '%s' -> '%s'",
              magic_enum::enum_name(this->hand_state_).data(),
              magic_enum::enum_name(next_state).data());
  this->hand_state_ = next_state;

  // update transform
  UpdateBellowsTransformVector(this->hand_state_);

  response->result = true;
}