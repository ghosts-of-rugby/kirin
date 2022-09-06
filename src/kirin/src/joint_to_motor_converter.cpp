#include <Eigen/LU>
#include "kirin/machine.hpp"
#include "kirin/joint_to_motor_converter.hpp"


JointToMotorConverter::JointToMotorConverter(const rclcpp::NodeOptions& options)
  : Node("joint_to_motor_converter", options),
    joint_callback_(
      std::bind(&JointToMotorConverter::JointStateCallback, this, std::placeholders::_1)) {

  double R_m = machine::kMotorRadius;
  double R_phi = machine::kPhiRadius;
  mat_motor_to_joint_ << 1./2.*R_m, 1./2.*R_m,
                        -R_m/R_phi, R_m/R_phi;
  
  /* initial joint value when setting */
  initial_joint_ = {
    .theta = 0,
    .z = 0,
    .r = 0,
    .phi = 0
  };

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  joint_sub_ = create_subscription<JointState>("joint_states", qos, joint_callback_);
  motor_pub_ = create_publisher<MotorStateVector>("motor/reference", qos);
}

void JointToMotorConverter::JointStateCallback(const JointState::UniquePtr msg) {
  // joint = {theta, z, r, phi, phi}
  Joint joint{
    .theta = msg->position.at(0),
    .z = msg->position.at(1),
    .r = msg->position.at(2),
    .phi = msg->position.at(3)
  };

  Eigen::Vector2d r_phi_vec{
    joint.r - initial_joint_.r, joint.phi - initial_joint_.phi
  };

  Eigen::Vector2d motor_vec = mat_motor_to_joint_.inverse() * r_phi_vec;
  // RCLCPP_INFO(this->get_logger(), "inverse: %5f, %5f, %5f, %5f",
  //     mat_motor_to_joint_.inverse()(0,0), mat_motor_to_joint_.inverse()(0,1),
  //     mat_motor_to_joint_.inverse()(1,0), mat_motor_to_joint_.inverse()(1,1));
  MotorAngle motor_angle{
    .theta = joint.theta / machine::kThetaGearRatio,
    .left = motor_vec.x(),
    .right = motor_vec.y(),
    // .z = joint.z / machine::kZGearRatio * machine::kZGearRadius
    .z = joint.z / machine::kZRatio
  };

  auto motor_msg = std::make_unique<MotorStateVector>();
  motor_msg->angle.theta = motor_angle.theta;
  motor_msg->angle.left = motor_angle.left;
  motor_msg->angle.right = motor_angle.right;
  motor_msg->angle.z = motor_angle.z;
  motor_pub_->publish(std::move(motor_msg));
}