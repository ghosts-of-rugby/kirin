
#include "kirin/motor_controller.hpp"

#include <chrono>

#include "kirin/common_types.hpp"

using namespace std::chrono_literals;  // NOLINT

double clip(double x, double min, double max) {
  if (x < min) {
    return min;
  } else if (x > max) {
    return max;
  }
  return x;
}

MotorController::MotorController(const rclcpp::NodeOptions& options)
    : uart(std::make_shared<ddt::Uart>("/dev/ttyUSB0")),
      angle_filter_right(),
      angle_filter_left(),
      motor_right(uart, 0x06, 5ms),
      motor_left(uart, 0x09, 5ms),
      Node("motor_controller", options),
      motor_callback_(std::bind(&MotorController::MotorStateVectorReceiveCallback, this,
                                std::placeholders::_1)) {
  rclcpp::QoS qos(rclcpp::KeepLast(2));
  motor_sub_ = create_subscription<MotorStateVector>("motor/reference", qos, motor_callback_);
  current_motor_pub_ = create_publisher<MotorStateVector>("motor/current", qos);
}

void MotorController::MotorStateVectorReceiveCallback(const MotorStateVector::UniquePtr msg) {
  auto start = this->get_clock()->now();

  // 各モーターの目標位置と目標速度の構造体
  MotorAngle angle{
      .theta = msg->angle.theta,
      .left = msg->angle.left,    // 7.0
      .right = msg->angle.right,  // -7.0
      .z = msg->angle.z,
  };
  // MotorAngle velcity{
  //   .theta = msg->velocity.theta,
  //   .left = msg->velocity.left,
  //   .right = msg->velocity.right,
  //   .z = msg->velocity.z
  // };

  /* velocityをffにつかいつつ，angleをPID制御 */
  double max_speed = 5.0;

  auto motor_state_right = motor_right.DriveVelocity(motor_right_input);
  auto motor_state_left = motor_left.DriveVelocity(motor_left_input);

  if (motor_state_right.has_value() && motor_state_left.has_value()) {
    double angle_right = angle_filter_right.Update(motor_state_right->angle);
    double angle_left = angle_filter_left.Update(motor_state_left->angle);
    motor_right_input = -5.0 * (angle.right - angle_right);
    motor_right_input = clip(motor_right_input, -max_speed, max_speed);
    motor_left_input = -5.0 * (angle.left - angle_left);
    motor_left_input = clip(motor_left_input, -max_speed, max_speed);
  }

  // 実装

  /* uartを使ってmotorの速度送信処理 */

  // 実装

  /* 受け取ったデータをpublish */
  auto current_motor = std::make_unique<MotorStateVector>();
  current_motor->angle.left = 0.0;
  current_motor->angle.right = 0.0;
  current_motor->angle.theta = 0.0;
  current_motor->angle.z = 0.0;
  current_motor->velocity.left = 0.0;
  current_motor->velocity.right = 0.0;
  current_motor->velocity.theta = 0.0;
  current_motor->velocity.z = 0.0;
  current_motor_pub_->publish(std::move(current_motor));

  /* end */
  auto run_time = this->get_clock()->now() - start;
  auto run_us = run_time.to_chrono<std::chrono::microseconds>();
  RCLCPP_INFO(this->get_logger(), "motor communication run time: %4d [us]", run_us.count());
}