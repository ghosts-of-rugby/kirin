#include "joy_controller/joy_controller.h"

JoyController::JoyController(const std::string& node_name, const std::string& topic_name)
  : Node(node_name) {
  auto callback = [this](const sensor_msgs::msg::Joy::UniquePtr msg) -> void {
      RCLCPP_INFO(this->get_logger(), "Received!");
      for(int i=0; i<magic_enum::enum_count<Axis>(); i++) {
        axes_.at(i) = msg->axes.at(i);
      }
      for(int i=0; i<magic_enum::enum_count<Button>(); i++) {
        if(msg->buttons.at(i)) buttons_.at(i) = ButtonState::Pressed;
        else buttons_.at(i) = ButtonState::Released;
      }
    };
  
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  sub_ = create_subscription<sensor_msgs::msg::Joy>(topic_name, qos, callback);
}

float JoyController::GetAxis(JoyController::Axis axis) const {
  return axes_.at(static_cast<size_t>(axis));
}

JoyController::ButtonState JoyController::GetButtonState(JoyController::Button button) const {
  return buttons_.at(static_cast<size_t>(button));
}