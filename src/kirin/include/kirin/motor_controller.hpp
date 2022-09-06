#ifndef SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_MOTOR_CONTROLLER
#define SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_MOTOR_CONTROLLER

#include <rclcpp/rclcpp.hpp>
#include <kirin_msgs/msg/motor_state_vector.hpp>

class MotorController : public rclcpp::Node {
 public:
  using MotorStateVector = kirin_msgs::msg::MotorStateVector;
  explicit MotorController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void MotorStateVectorReceiveCallback(const MotorStateVector::UniquePtr msg);
  std::function<void(const MotorStateVector::UniquePtr)> motor_callback_;
  rclcpp::Subscription<MotorStateVector>::SharedPtr motor_sub_;
  rclcpp::Publisher<MotorStateVector>::SharedPtr current_motor_pub_;
  
};

#endif /* SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_MOTOR_CONTROLLER */
