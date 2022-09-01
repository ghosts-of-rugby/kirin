#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>
#include <kirin_msgs/srv/toggle_hand_state.hpp>
#include "kirin/world_coord_manual_controller.hpp"
#include "ik_r.h"
#include "ik_phi.h"
#include "ik_theta.h"

WorldCoordManualController::WorldCoordManualController(const std::string& node_name)
  : JoyController(node_name, "/joy"),
    pos(0.3, 0.1, 0.1),
    psi(0.0),
    timer_callback_(std::bind(&WorldCoordManualController::TimerCallback, this)){

  this->RegisterButtonPressedCallback(Button::A, [this]() -> void {
    auto request = std::make_shared<kirin_msgs::srv::ToggleHandState::Request>();
    request->toggle = true;

    using ResponseFuture = rclcpp::Client<kirin_msgs::srv::ToggleHandState>::SharedFuture;
    auto response_callback = [this](ResponseFuture future) {
      auto response = future.get();
      // if(response->result) RCLCPP_INFO(this->get_logger(), "Successfully Hand State Changed");
    };

    auto future_result = toggle_hand_state_client_->async_send_request(
                            request, response_callback);
  });

  using namespace std::chrono_literals;
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  world_coord_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(input_topic_name_, qos);
  joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("manual_joint", qos);
  timer_ = create_wall_timer(10ms, timer_callback_);
  toggle_hand_state_client_ = create_client<kirin_msgs::srv::ToggleHandState>("tool/toggle_hand_state");
}

WorldCoordManualController::~WorldCoordManualController(){}

void WorldCoordManualController::TimerCallback() {
  auto input_pose = std::make_unique<geometry_msgs::msg::PoseStamped>();

  input_pose->header.frame_id = "fix_base";
  input_pose->header.stamp = get_clock()->now();

  Eigen::Vector3d vel(
    this->GetAxis(JoyController::Axis::LStickY)* -1.0 * 0.3,
    this->GetAxis(JoyController::Axis::LStickX)* 1.0 * 0.3,
    this->GetAxis(JoyController::Axis::RStickX)* 1.0 * 0.2
  );
  pos += vel*0.01; // 10ms loop
  input_pose->pose.position = Eigen::toMsg(pos);

  double left_trig = 1.0 - this->GetAxis(JoyController::Axis::LTrigger);
  double right_trig = 1.0 - this->GetAxis(JoyController::Axis::RTrigger);
  double omega = (left_trig - right_trig) * 0.3;
  psi += omega*0.01;
  Eigen::Quaterniond quat(Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ()));
  input_pose->pose.orientation = Eigen::toMsg(quat);

  world_coord_pub_->publish(std::move(input_pose));

  auto joint_state = std::make_unique<sensor_msgs::msg::JointState>();
  joint_state->name = {
    "theta_joint", "z_joint", "r_joint", "phi_joint", "phi_extend_joint"
  };
  // dokokakara mottekuru
  double l = 0.05;
  double r_offset = 0.345 + 0.201;
  double r = CalcR(l, pos.x(), pos.y(), psi);
  double theta = CalcTheta(l, pos.x(), pos.y(), psi);
  double phi = CalcPhi(l, pos.x(), pos.y(), psi);
  joint_state->position = {
    theta, pos.z(), r-r_offset, phi, phi 
  };
  joint_pub_->publish(std::move(joint_state));
}

double WorldCoordManualController::CalcR(double l, double x, double y, double psi) {
  double out[2];
  model::ik_r(l, psi, x, y, out);
  return out[ik_index];
}

double WorldCoordManualController::CalcPhi(double l, double x, double y, double psi) {
  double out[2];
  model::ik_phi(l, psi, x, y, out);
  return out[ik_index];
}

double WorldCoordManualController::CalcTheta(double l, double x, double y, double psi) {
  double out[2];
  model::ik_theta(l, psi, x, y, out);
  return out[ik_index];
}