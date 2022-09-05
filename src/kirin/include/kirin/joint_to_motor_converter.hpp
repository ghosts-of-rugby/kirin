#ifndef SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_JOINT_TO_MOTOR_CONVERTER
#define SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_JOINT_TO_MOTOR_CONVERTER

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <kirin_msgs/msg/motor.hpp>

class JointToMotorConverter : public rclcpp::Node {
 public:
  using JointState = sensor_msgs::msg::JointState;
  using Motor = kirin_msgs::msg::Motor;
  struct Joint {
    double theta; // [rad]
    double z;     // [m]
    double r;     // [m]
    double phi;   // [rad]
  };
  struct MotorAngle { // [rad]
    double theta;
    double left;
    double right;
    double z;
  };
  explicit JointToMotorConverter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  
 private:
  void JointStateCallback(const JointState::UniquePtr msg);
  std::function<void(const JointState::UniquePtr)> joint_callback_;
  rclcpp::Subscription<JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<Motor>::SharedPtr motor_angle_pub_;
  Eigen::Matrix2d mat_motor_to_joint_;
  Joint initial_joint_;

};


#endif /* SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_JOINT_TO_MOTOR_CONVERTER */
