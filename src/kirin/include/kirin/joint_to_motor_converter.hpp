#ifndef SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_JOINT_TO_MOTOR_CONVERTER
#define SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_JOINT_TO_MOTOR_CONVERTER

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <kirin_msgs/msg/motor_state_vector.hpp>

#include "kirin/common_types.hpp"

class JointToMotorConverter : public rclcpp::Node {
 public:
  using JointState = sensor_msgs::msg::JointState;
  using MotorStateVector = kirin_msgs::msg::MotorStateVector;
  explicit JointToMotorConverter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void JointStateCallback(const JointState::UniquePtr msg);
  bool is_red_;
  int color_dir_;
  std::function<void(const JointState::UniquePtr)> joint_callback_;
  rclcpp::Subscription<JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<MotorStateVector>::SharedPtr motor_pub_;
  Eigen::Matrix2d mat_motor_to_joint_;
  Joint initial_joint_;
};

#endif /* SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_JOINT_TO_MOTOR_CONVERTER */
