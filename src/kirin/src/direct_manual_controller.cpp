#include <chrono>
#include "kirin/direct_manual_controller.hpp"

using namespace std::chrono_literals;

DirectManualController::DirectManualController(
    const std::string& node_name, const rclcpp::NodeOptions& options)
  : JoyController(node_name, options) {
  
  auto direct_pub_message = [this]() -> void {
    auto manual_msg = std::make_unique<kirin_msgs::msg::Joint>();

    float max_r_vel = 0.2; // [m/s]
    float max_z_vel = 0.02; // [m/s]
    float max_theta_vel = M_PI/4.0/2.0; // [rad/s]
    float max_phi_vel = M_PI/4.0; // [rad/s]
    r_ += GetAxis(JoyController::Axis::LStickX) * max_r_vel * 0.01; // ratio * vel[m/s] * 0.01[s]
    z_ += GetAxis(JoyController::Axis::RStickX) * max_z_vel * 0.01;
    phi_ += GetAxis(JoyController::Axis::LStickY) * max_phi_vel * 0.01;
    theta_ += GetAxis(JoyController::Axis::RStickY) * max_theta_vel * 0.01;

    manual_msg->r = r_;
    manual_msg->z = z_;
    manual_msg->phi = phi_;
    manual_msg->theta = theta_;
  
    direct_pub_->publish(std::move(manual_msg));

    auto msg = std::make_unique<sensor_msgs::msg::JointState>();
    msg->name = {
      "theta_joint", "z_joint", "r_joint", "phi_joint", "phi_extend_joint"
    };
    msg->position = {
      theta_, z_, r_, phi_, phi_
    };
    joint_pub_->publish(std::move(msg));
  };

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  direct_pub_ = create_publisher<kirin_msgs::msg::Joint>(direct_pub_topic_name_, qos);
  joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("manual_joint", qos);
  timer_ = create_wall_timer(10ms, direct_pub_message);
}

DirectManualController::~DirectManualController() {
}