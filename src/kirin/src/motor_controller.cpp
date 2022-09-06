
#include <chrono>
#include "kirin/motor_controller.hpp"
#include "kirin/common_types.hpp"

MotorController::MotorController(const rclcpp::NodeOptions& options)
  : Node("motor_controller", options),
    motor_callback_(
      std::bind(&MotorController::MotorStateVectorReceiveCallback, this, std::placeholders::_1)) {

  rclcpp::QoS qos(rclcpp::KeepLast(2));
  motor_sub_ = create_subscription<MotorStateVector>("motor/reference", qos, motor_callback_);
  current_motor_pub_ = create_publisher<MotorStateVector>("motor/current", qos);
}


void MotorController::MotorStateVectorReceiveCallback(const MotorStateVector::UniquePtr msg) {
  auto start = this->get_clock()->now();

  // 各モーターの目標位置と目標速度の構造体
  MotorAngle angle{
    .theta = msg->angle.theta,
    .left = msg->angle.left,
    .right = msg->angle.right,
    .z = msg->angle.z,
  };
  // MotorAngle velcity{
  //   .theta = msg->velocity.theta,
  //   .left = msg->velocity.left,
  //   .right = msg->velocity.right,
  //   .z = msg->velocity.z
  // };


  /* velocityをffにつかいつつ，angleをPID制御 */





  // 実装 





  /* uartを使ってmotorの速度送信処理 */




  // 実装 




  /* 受け取ったデータをpublish */


  
  /* end */
  auto run_time = this->get_clock()->now() - start;
  auto run_us = run_time.to_chrono<std::chrono::microseconds>();
  // RCLCPP_INFO(this->get_logger(), "motor communication run time: %4d [us]", run_us.count());
}