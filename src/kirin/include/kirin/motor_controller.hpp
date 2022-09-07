#ifndef SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_MOTOR_CONTROLLER
#define SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_MOTOR_CONTROLLER

#include <kirin_msgs/msg/motor_state_vector.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ddt-motor/angle_filter.hpp"
#include "ddt-motor/motor.hpp"
#include "ddt-motor/uart.hpp"

class MotorController : public rclcpp::Node {
 public:
  using MotorStateVector = kirin_msgs::msg::MotorStateVector;
  explicit MotorController(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

 private:
  std::shared_ptr<ddt::Uart> uart;
  ddt::AngleFilter angle_filter_right, angle_filter_left;
  ddt::Motor motor_right, motor_left;
  double motor_right_input = 0.0;
  double motor_left_input = 0.0;

  void MotorStateVectorReceiveCallback(const MotorStateVector::UniquePtr msg);
  std::function<void(const MotorStateVector::UniquePtr)> motor_callback_;
  rclcpp::Subscription<MotorStateVector>::SharedPtr motor_sub_;
  rclcpp::Publisher<MotorStateVector>::SharedPtr current_motor_pub_;
};

#endif /* SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_MOTOR_CONTROLLER */
