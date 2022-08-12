#ifndef SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_MANUAL_CONTROLLER
#define SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_MANUAL_CONTROLLER

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <kirin_msgs/msg/direct_manual.hpp>

#include "kirin/joy_controller.h"

class ManualController : public JoyController {
 public:
  explicit ManualController(const std::string& node_name);
  ~ManualController();

 private:
  std::string joy_topic_name{"/joy"};
  std::string direct_pub_topic_name{"direct_manual"};
  rclcpp::Publisher<kirin_msgs::msg::DirectManual>::SharedPtr direct_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};


#endif /* SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_MANUAL_CONTROLLER */
