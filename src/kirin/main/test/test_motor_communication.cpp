
#include <rclcpp/rclcpp.hpp>
#include "ddt-motor/uart.hpp"
#include "ddt-motor/motor.hpp"

class TestMotorCommunication : public rclcpp::Node {
 public:
  explicit TestMotorCommunication() : Node("motor_controller") {
    auto name = declare_parameter("motor", "");
    auto mode = declare_parameter("mode", "");
    sleep_ms_ = declare_parameter("sleep_time", 5);

    std::string device;
    if (name == "left" || name == "right") {
      device = declare_parameter("usb_device.left_right", "");
    } else if (name == "z" || name == "theta") {
      device = declare_parameter("usb_device.theta_z", "");
    } else {
      RCLCPP_ERROR(this->get_logger(), "motor name is invalid! Choose from left, right, z, theta");
      rclcpp::shutdown();
    }

    if (mode == "observe") {
      callback_ = std::bind(&TestMotorCommunication::DataObserve, this);
    } else if (mode == "current") {
      callback_ = std::bind(&TestMotorCommunication::CurrentInput, this);
    } else if (mode == "velocity") {
      callback_ = std::bind(&TestMotorCommunication::VelocityInput, this);
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "mode name is invalid! Choose from observe, current, velocity");
      rclcpp::shutdown();
    }

    uart_
        = std::make_shared<ddt::Uart>("/dev/serial/by-id/" + device, ddt::Uart::BaudRate::B_115200);
    motor_ = std::make_shared<ddt::Motor>(uart_, declare_parameter(name + ".id", 0x00));
    dir_   = declare_parameter(name + ".dir", 1);

    if (mode == "current") motor_->SetMode(ddt::Motor::DriveMode::Current);

    // motor_theta->SetMode(ddt::Motor::DriveMode::Current);
    timer_ = create_wall_timer(std::chrono::milliseconds(sleep_ms_ + 2), callback_);
  }

 private:
  void DataObserve() {}

  void CurrentInput() {}

  void VelocityInput() {}

  std::shared_ptr<ddt::Uart> uart_;
  std::shared_ptr<ddt::Motor> motor_;
  int dir_;
  int sleep_ms_;

  std::function<void()> callback_;
  rclcpp::TimerBase::SharedPtr timer_;
};