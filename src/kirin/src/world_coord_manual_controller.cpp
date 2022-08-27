#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>
#include "kirin/world_coord_manual_controller.hpp"

WorldCoordManualController::WorldCoordManualController(const std::string& node_name)
  : JoyController(node_name, "/joy"),
    pos(0.5, 0.5, 1.0),
    theta(0.0),
    timer_callback_(std::bind(&WorldCoordManualController::TimerCallback, this)){
  using namespace std::chrono_literals;
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  world_coord_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(input_topic_name_, qos);
  timer_ = create_wall_timer(10ms, timer_callback_);
}

WorldCoordManualController::~WorldCoordManualController(){}

void WorldCoordManualController::TimerCallback() {
  auto input_pose = std::make_unique<geometry_msgs::msg::PoseStamped>();

  input_pose->header.frame_id = "fix_base";
  input_pose->header.stamp = get_clock()->now();

  Eigen::Vector3d vel(
    this->GetAxis(JoyController::Axis::LStickY)* -1.0 * 0.5,
    this->GetAxis(JoyController::Axis::LStickX)* 1.0 * 0.5,
    this->GetAxis(JoyController::Axis::RStickX)* 1.0 * 0.2
  );
  pos += vel*0.01; // 10ms loop
  input_pose->pose.position = Eigen::toMsg(pos);

  double left_trig = 1.0 - this->GetAxis(JoyController::Axis::LTrigger);
  double right_trig = 1.0 - this->GetAxis(JoyController::Axis::RTrigger);
  double omega = (left_trig - right_trig) * 0.2;
  theta += omega*0.01;
  Eigen::Quaterniond quat(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
  input_pose->pose.orientation = Eigen::toMsg(quat);

  world_coord_pub_->publish(std::move(input_pose));
}