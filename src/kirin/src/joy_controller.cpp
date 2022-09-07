#include "kirin/joy_controller.hpp"

JoyController::JoyController(const std::string& node_name, const rclcpp::NodeOptions& options)
    : Node(node_name, options),
      joy_callback_(std::bind(&JoyController::JoyTopicCallback, this, std::placeholders::_1)) {
  // Initial Value
  for (auto& e : axes_) e = 0.0;
  axes_.at(static_cast<size_t>(Axis::LTrigger)) = 1.0;
  axes_.at(static_cast<size_t>(Axis::RTrigger)) = 1.0;
  for (auto& e : buttons_) e = ButtonState::Released;

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  sub_ = create_subscription<sensor_msgs::msg::Joy>("/joy", qos, joy_callback_);
}

JoyController::~JoyController() {}

void JoyController::JoyTopicCallback(const sensor_msgs::msg::Joy::UniquePtr msg) {
  for (int i = 0; i < magic_enum::enum_count<Axis>(); i++) {
    axes_.at(i) = msg->axes.at(i);
  }
  for (int i = 0; i < magic_enum::enum_count<Button>(); i++) {
    auto new_state = magic_enum::enum_cast<ButtonState>(msg->buttons.at(i)).value();
    if (buttons_.at(i) == ButtonState::Released && new_state == ButtonState::Pressed) {
      auto pressed_button = magic_enum::enum_cast<Button>(i).value();
      RCLCPP_INFO(this->get_logger(), "Button [ %s ] Pressed!",
                  magic_enum::enum_name(pressed_button).data());
      if (pressed_callback_.at(i)) pressed_callback_.at(i)();
    }
    buttons_.at(i) = new_state;
  }
}

float JoyController::GetAxis(const JoyController::Axis& axis) const {
  return axes_.at(magic_enum::enum_integer(axis));
}

JoyController::ButtonState JoyController::GetButtonState(
    const JoyController::Button& button) const {
  return buttons_.at(magic_enum::enum_integer(button));
}

void JoyController::RegisterButtonPressedCallback(const JoyController::Button& button,
                                                  std::function<void()> callback) {
  pressed_callback_.at(magic_enum::enum_integer(button)) = callback;
}