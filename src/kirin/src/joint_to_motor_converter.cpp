#include <Eigen/LU>
#include "kirin/machine.hpp"
#include "kirin/joint_to_motor_converter.hpp"

JointToMotorConverter::JointToMotorConverter(const rclcpp::NodeOptions& options)
    : Node("joint_to_motor_converter", options),
      joint_callback_(
          std::bind(&JointToMotorConverter::JointStateCallback, this, std::placeholders::_1)) {
  /* color direction from field */
  is_red_    = declare_parameter("field", "blue") == "red";
  color_dir_ = is_red_ ? -1.0 : 1.0;

  /* define convert matrix */
  double R_m   = machine::kMotorRadius;
  double R_phi = machine::kPhiRadius;

  // clang-format off
  mat_motor_to_joint_ << 1./2.*R_m,       1./2.*R_m,
                         1./2.*R_m/R_phi, 1./2.*-R_m/R_phi;
  // clang-format on

  /* initial joint value when setting */
  initial_joint_ = {.theta = 0, .z = 0, .r = 0, .phi = 0};

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  joint_sub_ = create_subscription<JointState>("joint_states", qos, joint_callback_);
  motor_pub_ = create_publisher<MotorStateVector>("motor/stete/reference", qos);
}

void JointToMotorConverter::JointStateCallback(const JointState::UniquePtr msg) {
  if(msg->velocity.size() < 3) return;
  // joint_state = {theta, z, r, phi, phi}
  Joint joint{.theta = msg->position.at(0),
              .z     = msg->position.at(1),
              .r     = msg->position.at(2),
              .phi   = msg->position.at(3)};
  Joint joint_vel{
    .theta = msg->velocity.at(0),
    .z = msg->velocity.at(1),
    .r = msg->velocity.at(2),
    .phi = msg->velocity.at(3)
  };

  Eigen::Vector2d r_phi_pos{joint.r - initial_joint_.r, joint.phi - initial_joint_.phi};
  Eigen::Vector2d r_phi_vel{joint_vel.r, joint_vel.phi};

  /* offset (difference) from initial position and motor (Note: motor angle start from 0.0) */
  double theta_offset = (is_red_) ? machine::kRedInitialPsi : -1.0 * machine::kRedInitialPsi;
  double z_offset = 0.0; // start point offset from z lowest point 
  Eigen::Vector2d motor_pos = mat_motor_to_joint_.inverse() * r_phi_pos;
  Eigen::Vector2d motor_vel = mat_motor_to_joint_.inverse() * r_phi_vel;
  MotorAngle motor_angle{.theta = (joint.theta - theta_offset)/ machine::kThetaGearRatio,
                         .left  = motor_pos.x(),
                         .right = motor_pos.y(),
                         // .z = joint.z / machine::kZGearRatio * machine::kZGearRadius
                         .z     = (joint.z - z_offset) / machine::kZRatio};
  MotorAngle motor_velocity{
    .theta = joint_vel.theta / machine::kThetaGearRatio,
    .left = motor_vel.x(),
    .right = motor_vel.y(),
    .z = joint_vel.z / machine::kZRatio
  };
  // velocity convert
  // MotorAngle velocity;

  auto motor_msg         = std::make_unique<MotorStateVector>();
  motor_msg->angle.theta = motor_angle.theta;
  motor_msg->angle.left  = motor_angle.left;
  motor_msg->angle.right = motor_angle.right;
  motor_msg->angle.z     = motor_angle.z;
  motor_msg->velocity.theta = motor_velocity.theta;
  motor_msg->velocity.left = motor_velocity.left;
  motor_msg->velocity.right = motor_velocity.right;
  motor_msg->velocity.z = motor_velocity.z;
  motor_pub_->publish(std::move(motor_msg));
}