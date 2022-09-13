#ifndef SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_MOTOR_CONTROLLER
#define SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_MOTOR_CONTROLLER

#include <kirin_msgs/msg/motor_state_vector.hpp>
#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "ddt-motor/angle_filter.hpp"
#include "ddt-motor/motor.hpp"
#include "ddt-motor/uart.hpp"
#include "ddt-motor/observer.hpp"

struct ControllerBase {
  ddt::AngleFilter filter;
  std::optional<ddt::Motor::State> state = std::nullopt;
  double angle = 0;
  double velocity = 0;
  virtual double GetInput(double ref_velocity, double ref_angle) = 0;
  void Update(std::optional<ddt::Motor::State> state);
};

struct ControllerVelocityInput : public ControllerBase {
  int dir = 1;
  double Kp;
  double max_speed;
  ControllerVelocityInput(int dir, double Kp, double max_speed);
  double GetInput(double ref_velocity, double ref_angle) override;
};

struct ControllerCurrentInput : public ControllerBase {
  bool is_first_time = true;
  int dir = 1;
  std::chrono::high_resolution_clock::time_point
      pre_update_time;     // 前回の更新時刻
  double pre_input = 0.0;  // 前回の入力
  double vel_est = 0.0;    // 速度の推定値
  double dist_est = 0.0;   //外乱の推定値
  double Kp_pos, Kp_vel;
  double max_current;
  ddt::Observer observer;
  ControllerCurrentInput(int dir,                       //
                         double max_current,            //
                         double Kp_pos, double Kp_vel,  //
                         double obs_K, double obs_pole1, double obs_pole2);
  void Update(std::optional<ddt::Motor::State> state);
  double GetInput(double ref_velocity, double ref_angle) override;
};

struct ControllerCurrentInputWithoutObserver : public ControllerBase {
  int dir = 1;
  double Kp_pos, Kp_vel;
  double max_current;
  ControllerCurrentInputWithoutObserver(int dir,             //
                                        double max_current,  //
                                        double Kp_pos, double Kp_vel);
  double GetInput(double ref_velocity, double ref_angle) override;
};

class MotorController : public rclcpp::Node {
 public:
  using MotorStateVector = kirin_msgs::msg::MotorStateVector;
  explicit MotorController(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  bool use_hardware_;
  std::chrono::milliseconds sleep_time;
  std::optional<ddt::Motor> motor_right, motor_left, motor_theta, motor_z;
  std::optional<ControllerVelocityInput> controller_z;
  std::optional<ControllerCurrentInputWithoutObserver> controller_right,
      controller_left;
  std::optional<ControllerCurrentInput> controller_theta;

  void ShowWarning(const std::array<bool, 4>& state_has_value_arr);
  void MotorStateVectorReceiveCallback(const MotorStateVector::UniquePtr msg);
  std::function<void(const MotorStateVector::UniquePtr)> motor_callback_;
  rclcpp::Subscription<MotorStateVector>::SharedPtr motor_sub_;
  rclcpp::Publisher<MotorStateVector>::SharedPtr current_motor_pub_;
};

#endif /* SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_MOTOR_CONTROLLER */
