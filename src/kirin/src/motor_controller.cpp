
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

ControllerVelocityInput::ControllerVelocityInput(int dir, double Kp,
                                                 double max_speed) {
  this->dir = dir;
  this->Kp = Kp;
  this->max_speed = max_speed;
}

ControllerCurrentInput::ControllerCurrentInput(int dir, double max_current,
                                               double Kp_pos, double Kp_vel,
                                               double obs_K, double obs_pole1,
                                               double obs_pole2)
    : dir(dir),
      pre_input(0.0),
      max_current(max_current),
      Kp_pos(Kp_pos),
      Kp_vel(Kp_vel),
      observer(obs_K, obs_pole1, obs_pole2) {}

double ControllerCurrentInput::GetInput(double ref_velocity, double ref_angle) {
  ref_angle *= dir;
  ref_velocity *= dir;
  if (is_first_time) {
    pre_update_time = std::chrono::high_resolution_clock::now();
    is_first_time = false;
  }
  double input = Kp_pos * (ref_angle - angle) +
                 Kp_vel * (ref_velocity - velocity) + this->dist_est;
  input = std::clamp(input, -max_current, max_current);
  pre_input = input;
  return input;
}

void ControllerCurrentInput::Update(std::optional<ddt::Motor::State> state) {
  using second = std::chrono::duration<double>;
  this->state = state;
  if (state.has_value()) {
    angle = -filter.Update(state->angle);
    velocity = state->velocity;
    auto now = std::chrono::high_resolution_clock::now();
    auto dt = std::chrono::duration_cast<second>(now - pre_update_time);
    auto [v_e, d_e] =
        observer.Update(pre_input, velocity, dt);  // 速度・外乱の推定値
    this->vel_est = v_e;
    this->dist_est = d_e;
    pre_update_time = now;
  }
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
    motor_z = ddt::Motor(uart_theta_z, declare_parameter("z.id", 0x06));
    motor_theta = ddt::Motor(uart_theta_z, declare_parameter("theta.id", 0x03));

    motor_theta->SetMode(ddt::Motor::DriveMode::Current);
    
    controller_right = ControllerVelocityInput(
        declare_parameter("right.dir", 1), declare_parameter("right.Kp", 1.0),
        declare_parameter("right.max_speed", 1.0));
    controller_left = ControllerVelocityInput(
        declare_parameter("left.dir", 1), declare_parameter("left.Kp", 1.0),
        declare_parameter("left.max_speed", 1.0));
    controller_z = ControllerVelocityInput(
        declare_parameter("z.dir", 1), declare_parameter("z.Kp", 1.0),
        declare_parameter("z.max_speed", 1.0));
    controller_theta = ControllerCurrentInput(
        declare_parameter("theta.dir", 1),
        declare_parameter("theta.max_current", 3.0),
        declare_parameter("theta.Kp_pos", 0.3),
        declare_parameter("theta.Kp_vel", 0.3),
        declare_parameter("theta.observer.K", 15.7),
        declare_parameter("theta.observer.pole1", -20.0),
        declare_parameter("theta.observer.pole2", -10.0));

    // motor_right = std::make_optional<ddt::Motor>();
  } else {
    RCLCPP_WARN(this->get_logger(), "motor hardware deactivated");
  }

  rclcpp::QoS qos(rclcpp::KeepLast(2));
  motor_sub_ = create_subscription<MotorStateVector>("motor/reference", qos,
                                                     motor_callback_);
  current_motor_pub_ = create_publisher<MotorStateVector>("motor/current", qos);
}

void MotorController::ShowWarning(const std::array<bool, 4>& state_has_value_arr) {
  bool all_has_value = 
    std::all_of(state_has_value_arr.begin(), state_has_value_arr.end(), [](bool has_value) { return has_value; });
  if(all_has_value) return;
  else {
    RCLCPP_WARN(
      this->get_logger(), "motor command status: left [%s], right [%s], z [%s], theta[%s]",
      state_has_value_arr.at(0) ? "  OK!  " : "Failure", 
      state_has_value_arr.at(1) ? "  OK!  " : "Failure", 
      state_has_value_arr.at(2) ? "  OK!  " : "Failure", 
      state_has_value_arr.at(3) ? "  OK!  " : "Failure");
  }
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

  if (use_hardware_) {
    current_motor->angle.left = 0;
    current_motor->angle.right = 0;
    current_motor->angle.theta = 0;
    current_motor->angle.z = 0;
    current_motor->velocity.left = 0;
    current_motor->velocity.right = 0;
    current_motor->velocity.theta = 0;
    current_motor->velocity.z = 0;

    /* process of right and z motor */
    double right_velocity =
        controller_right->GetInput(velocity.right, angle.right);
    motor_right->SendVelocityCommand(right_velocity);
    double z_velocity = controller_z->GetInput(velocity.z, angle.z);
    motor_z->SendVelocityCommand(z_velocity);
    std::this_thread::sleep_for(8ms);
    auto state_right = motor_right->ReceiveSpinMotorFeedback();
    auto state_z = motor_z->ReceiveSpinMotorFeedback();
    controller_right->Update(state_right);
    controller_z->Update(state_z);

    /* process of left and theta motor */
    double left_velocity = controller_left->GetInput(velocity.left, angle.left);
    motor_left->SendVelocityCommand(left_velocity);
    double theta_current =
        controller_theta->GetInput(velocity.theta, angle.theta);
    motor_theta->SendCurrentCommand(theta_current);
    std::this_thread::sleep_for(8ms);
    auto state_left = motor_left->ReceiveSpinMotorFeedback();
    auto state_theta = motor_theta->ReceiveSpinMotorFeedback();
    controller_left->Update(state_left);
    controller_theta->Update(state_theta);

    /* show warning message */
    std::array<bool, 4> state_has_value_arr = 
      {state_left.has_value(), state_right.has_value(), state_z.has_value(), state_theta.has_value()}; 
    ShowWarning(state_has_value_arr);

    /* substitute current motor state */
    current_motor->angle.right =
        controller_right->angle * controller_right->dir;
    current_motor->velocity.right =
        controller_right->velocity * controller_right->dir;

    current_motor->angle.left = controller_left->angle * controller_left->dir;
    current_motor->velocity.left =
        controller_left->velocity * controller_left->dir;

    current_motor->angle.z = controller_z->angle * controller_z->dir;
    current_motor->velocity.z = controller_z->velocity * controller_z->dir;

    current_motor->angle.theta =
        controller_theta->angle * controller_theta->dir;
    current_motor->velocity.theta =
        controller_theta->velocity * controller_theta->dir;
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
  if (run_us > 17000us) {
    RCLCPP_WARN(this->get_logger(), "motor communication run time: %4d [us]",
                run_us.count());
  }
}