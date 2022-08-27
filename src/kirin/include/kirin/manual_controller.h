#ifndef SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_MANUAL_CONTROLLER
#define SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_MANUAL_CONTROLLER

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <kirin_msgs/msg/joint.hpp>

#include "kirin/joy_controller.h"

class ManualController : public JoyController {
 public:
  explicit ManualController(const std::string& node_name);
  ~ManualController();

 private:
  std::string joy_topic_name_{"/joy"};
  std::string direct_pub_topic_name_{"direct_manual"};
  rclcpp::Publisher<kirin_msgs::msg::Joint>::SharedPtr direct_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  float theta_{0.0};
  float z_{0.0};
  float r_{0.0};
  float phi_{0.0};
};


#endif /* SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_MANUAL_CONTROLLER */
