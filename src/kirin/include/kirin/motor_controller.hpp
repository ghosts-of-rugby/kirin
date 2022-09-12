#ifndef SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_MOTOR_CONTROLLER
#define SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_MOTOR_CONTROLLER

#include <kirin_msgs/msg/motor_state_vector.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "ddt-motor/angle_filter.hpp"
#include "ddt-motor/motor.hpp"
#include "ddt-motor/uart.hpp"

struct ControllerBase {
  ddt::AngleFilter filter;
  std::optional<ddt::Motor::State> state = std::nullopt;
  double angle = 0;
  double velocity = 0;
  virtual double GetInput(double ref_velocity, double ref_angle) = 0;
  void Update(std::optional<ddt::Motor::State> state);
};

struct ControllerVelocityInput : public ControllerBase {
  int dir = 1;
  double Kp;
  double max_speed;
  ControllerVelocityInput(int dir, double Kp, double max_speed);
  double GetInput(double ref_velocity, double ref_angle) override;
};

struct ControllerCurrentInput {};

class MotorController : public rclcpp::Node {
 public:
  using MotorStateVector = kirin_msgs::msg::MotorStateVector;
  explicit MotorController(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  bool use_hardware_;
  std::chrono::milliseconds sleep_time;
  std::optional<ddt::Motor> motor_right, motor_left, motor_theta, motor_z;
  std::optional<ControllerVelocityInput> controller_right, controller_left,
      controller_z;

  void MotorStateVectorReceiveCallback(const MotorStateVector::UniquePtr msg);
  std::function<void(const MotorStateVector::UniquePtr)> motor_callback_;
  rclcpp::Subscription<MotorStateVector>::SharedPtr motor_sub_;
  rclcpp::Publisher<MotorStateVector>::SharedPtr current_motor_pub_;
};

#endif /* SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_MOTOR_CONTROLLER */
