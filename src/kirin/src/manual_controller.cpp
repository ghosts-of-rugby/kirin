#include "kirin/manual_controller.h"

ManualController::ManualController(const std::string& node_name, const std::string& topic_name)
  : Node(node_name) {
  auto callback = [this](const sensor_msgs::msg::Joy::UniquePtr msg) -> void {
      RCLCPP_INFO(this->get_logger(), "Received!");
    };
  
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(topic_name, qos, callback);
}