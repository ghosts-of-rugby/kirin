#include <chrono>
#include "kirin/manual_controller.h"

using namespace std::chrono_literals;

ManualController::ManualController(const std::string& node_name)
  : JoyController(node_name, "/joy") {
  
  auto direct_pub_message = [this]() -> void {
    auto msg = std::make_unique<kirin_msgs::msg::DirectManual>();
    msg->l = GetAxis(JoyController::Axis::LStickX);
    msg->z = GetAxis(JoyController::Axis::RStickX);
    msg->phi = GetAxis(JoyController::Axis::LStickY) * -1.0;
    msg->theta = GetAxis(JoyController::Axis::RStickY) * -1.0;
  
    direct_pub_->publish(std::move(msg));
  };

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  direct_pub_ = create_publisher<kirin_msgs::msg::DirectManual>(direct_pub_topic_name, qos);
  timer_ = create_wall_timer(10ms, direct_pub_message);
}

ManualController::~ManualController() {
}