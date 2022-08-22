#ifndef SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_JOY_CONTROLLER
#define SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_JOY_CONTROLLER


#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <magic_enum.hpp>

#include <sensor_msgs/msg/joy.hpp>

class JoyController : public rclcpp::Node {
 public:
  enum class Axis : int {
    LStickY = 0,
    LStickX = 1,
    LTrigger = 2,
    RStickY = 3,
    RStickX = 4,
    RTrigger = 5,
    CrossY = 6,
    CrossX = 7
  };

  enum class Button : int {
    A = 0,
    B = 1,
    X = 2,
    Y = 3,
    LB = 4,
    RB = 5,
    Back = 6,
    Start = 7,
    Home = 8,
    LStick = 9,
    RStick = 10
  };

  enum class ButtonState {
    Released,
    Pressed
  };

 public:
  explicit JoyController(const std::string& node_name, const std::string& joy_topic = "/joy");
  virtual ~JoyController();
  float GetAxis(Axis axis) const;
  ButtonState GetButtonState(Button button) const;

 private:
  std::array<float, magic_enum::enum_count<Axis>()> axes_;
  std::array<ButtonState, magic_enum::enum_count<Button>()> buttons_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
};

#endif /* SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_JOY_CONTROLLER */
