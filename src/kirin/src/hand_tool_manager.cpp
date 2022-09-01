
#include <magic_enum.hpp>
#include "kirin/hand_tool_manager.hpp"

using namespace std::chrono_literals;

HandToolManager::HandToolManager()
    : Node("hand_tool_manager"),
      hand_state_(HandState::Shrink),
      timer_callback_(std::bind(&HandToolManager::TimerCallback, this)),
      handle_set_hand_state_(
        std::bind(&HandToolManager::SetHandStateCallback, this,
                  std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)),
      handle_toggle_hand_state_(
        std::bind(&HandToolManager::ToggleHandStateCallback, this,
                  std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)){

  std::string mesh_directory = "package://kirin/resources/light";
  resource_map_ = {
    {HandState::Shrink, mesh_directory+"/phi.stl"},
    {HandState::Extend, mesh_directory+"/phi_extend.stl"}
  };

  marker_pub_ = create_publisher<Marker>("hand_marker", rclcpp::SystemDefaultsQoS());
  timer_ = create_wall_timer(10ms, timer_callback_);
  set_srv_ = create_service<SetHandState>("tool/set_hand_state", handle_set_hand_state_);
  toggle_srv_ = create_service<ToggleHandState>(
                  "tool/toggle_hand_state", handle_toggle_hand_state_);
}

void HandToolManager::TimerCallback() {
  auto marker_msg = std::make_unique<HandToolManager::Marker>();
  marker_msg->header.frame_id = "phi_link_base";
  marker_msg->header.stamp = get_clock()->now();

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

  marker_msg->color.r = 0.3;
  marker_msg->color.g = 0.3;
  marker_msg->color.b = 0.3;
  marker_msg->color.a = 1.0;

  marker_pub_->publish(std::move(marker_msg));
}

void HandToolManager::SetHandStateCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<SetHandState::Request> request,
    std::shared_ptr<SetHandState::Response> response) {
  (void)request_header; // Lint Tool 対策
  auto next_state = magic_enum::enum_cast<HandState>(request->hand_state.value);

  if(next_state.has_value()) {
    RCLCPP_INFO(this->get_logger(), "ToolState: '%s' -> '%s'",
                magic_enum::enum_name(this->hand_state_).data(),
                magic_enum::enum_name(next_state.value()).data());
    this->hand_state_ = next_state.value();
    response->result = true;
  }
}

void HandToolManager::ToggleHandStateCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ToggleHandState::Request> request,
    std::shared_ptr<ToggleHandState::Response> response) {
  
  (void)request_header; // Lint Tool 対策
  auto next_state = (this->hand_state_ == HandState::Extend
                      ? HandState::Shrink : HandState::Extend);

    RCLCPP_INFO(this->get_logger(), "ToolState: '%s' -> '%s'",
                magic_enum::enum_name(this->hand_state_).data(),
                magic_enum::enum_name(next_state).data());
    this->hand_state_ = next_state;
    response->result = true;
}