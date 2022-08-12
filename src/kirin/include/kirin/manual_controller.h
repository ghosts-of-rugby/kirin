#ifndef SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_MANUAL_CONTROLLER
#define SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_MANUAL_CONTROLLER


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

class ManualController : public rclcpp::Node {
 public:
  explicit ManualController(const std::string& node_name, const std::string& topic_name = "/joy");

 private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

};


#endif /* SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_MANUAL_CONTROLLER */
