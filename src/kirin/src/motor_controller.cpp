
#include "kirin/motor_controller.hpp"

#include <algorithm>
#include <chrono>

#include "kirin/common_types.hpp"

using namespace std::chrono_literals;  // NOLINT

void ControllerBase::Update(std::optional<ddt::Motor::State> state) {
  this->state = state;
  if (state.has_value()) {
    angle = -filter.Update(state->angle);
    velocity = state->velocity;
  }
}

double ControllerVelocityInput::GetInput(double ref_velocity,
                                         double ref_angle) {
  ref_angle *= dir;
  ref_velocity *= dir;
  double input = ref_velocity + Kp * (ref_angle - angle);
  input = std::clamp(input, -max_speed, max_speed);
  return input;
}

ControllerVelocityInput::ControllerVelocityInput(int dir, double Kp, double max_speed) {
  this->dir = dir;
  this->Kp = Kp;
  this->max_speed = max_speed;
}

MotorController::MotorController(const rclcpp::NodeOptions& options)
    : Node("motor_controller", options),
      motor_callback_(
          std::bind(&MotorController::MotorStateVectorReceiveCallback, this,
                    std::placeholders::_1)) {
  use_hardware_ = declare_parameter("use_hardware", false);

  if (use_hardware_) {
    std::string usb_device_left_right =
        declare_parameter("usb_device.left_right", "");
    std::string usb_device_theta_z =
        declare_parameter("usb_device.theta_z", "");

    auto uart_left_right = std::make_shared<ddt::Uart>(
        "/dev/serial/by-id/" + usb_device_left_right,
        ddt::Uart::BaudRate::B_115200);
    auto uart_theta_z =
        std::make_shared<ddt::Uart>("/dev/serial/by-id/" + usb_device_theta_z,
                                    ddt::Uart::BaudRate::B_115200);

    motor_right =
        ddt::Motor(uart_left_right, declare_parameter("right.id", 0x06));
    motor_left =
        ddt::Motor(uart_left_right, declare_parameter("left.id", 0x09));
    motor_theta = ddt::Motor(uart_theta_z, declare_parameter("theta.id", 0x03));
    motor_z = ddt::Motor(uart_theta_z, declare_parameter("z.id", 0x06));

    controller_right = ControllerVelocityInput(
        declare_parameter("right.dir", 1), declare_parameter("right.Kp", 1.0),
        declare_parameter("right.max_speed", 1.0));
    controller_left = ControllerVelocityInput(
        declare_parameter("left.dir", 1), declare_parameter("left.Kp", 1.0),
        declare_parameter("left.max_speed", 1.0));
    controller_z = ControllerVelocityInput(
        declare_parameter("z.dir", 1), declare_parameter("z.Kp", 1.0),
        declare_parameter("z.max_speed", 1.0));

    // motor_right = std::make_optional<ddt::Motor>();
  }

  rclcpp::QoS qos(rclcpp::KeepLast(2));
  motor_sub_ = create_subscription<MotorStateVector>("motor/reference", qos,
                                                     motor_callback_);
  current_motor_pub_ = create_publisher<MotorStateVector>("motor/current", qos);
}

void MotorController::MotorStateVectorReceiveCallback(
    const MotorStateVector::UniquePtr msg) {
  auto start = this->get_clock()->now();

  // 各モーターの目標位置と目標速度の構造体
  MotorAngle angle{
      .theta = msg->angle.theta,
      .left = msg->angle.left,
      .right = msg->angle.right,
      .z = msg->angle.z,
  };
  MotorAngle velocity{.theta = msg->velocity.theta,
                      .left = msg->velocity.left,
                      .right = msg->velocity.right,
                      .z = msg->velocity.z};

  auto current_motor = std::make_unique<MotorStateVector>();

  if(use_hardware_) {
    current_motor->angle.left = 0;
    current_motor->angle.right = 0;
    current_motor->angle.theta = 0;
    current_motor->angle.z = 0;
    current_motor->velocity.left = 0;
    current_motor->velocity.right = 0;
    current_motor->velocity.theta = 0;
    current_motor->velocity.z = 0;

    double right_velocity =
        controller_right->GetInput(velocity.right, angle.right);
    motor_right->SendVelocityCommand(right_velocity);
    double z_velocity = controller_z->GetInput(velocity.z, angle.z);
    motor_z->SendVelocityCommand(z_velocity);
    std::this_thread::sleep_for(5ms);
    auto state_right = motor_right->ReceiveSpinMotorFeedback();
    auto state_z = motor_z->ReceiveSpinMotorFeedback();
    controller_right->Update(state_right);
    controller_z->Update(state_z);

    double left_velocity = controller_left->GetInput(velocity.left, angle.left);
    motor_left->SendVelocityCommand(left_velocity);
    std::this_thread::sleep_for(5ms);
    auto state_left = motor_left->ReceiveSpinMotorFeedback();
    controller_left->Update(state_left);

    current_motor->angle.right =
        controller_right->angle * controller_right->dir;
    current_motor->velocity.right =
        controller_right->velocity * controller_right->dir;
    current_motor->angle.left = controller_left->angle * controller_left->dir;
    current_motor->velocity.left =
        controller_left->velocity * controller_left->dir;
    current_motor->angle.z = controller_z->angle * controller_z->dir;
    current_motor->velocity.z = controller_z->velocity * controller_z->dir;
  } else {
    current_motor->angle.left = angle.left;
    current_motor->angle.right = angle.right;
    current_motor->angle.theta = angle.theta;
    current_motor->angle.z = angle.z;
    current_motor->velocity.left = velocity.left;
    current_motor->velocity.right = velocity.right;
    current_motor->velocity.theta = velocity.theta;
    current_motor->velocity.z = velocity.theta;
  }

  /* 受け取ったデータをpublish */
  current_motor_pub_->publish(std::move(current_motor));

  /* end */
  auto run_time = this->get_clock()->now() - start;
  auto run_us = run_time.to_chrono<std::chrono::microseconds>();
  if (run_us > 11000us) {
    RCLCPP_WARN(this->get_logger(), "motor communication run time: %4d [us]",
                run_us.count());
  }
}