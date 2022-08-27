#ifndef SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_HAND_COORD_CONTROLLER
#define SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_HAND_COORD_CONTROLLER

#include <string>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "kirin/joy_controller.hpp"

class WorldCoordManualController : public JoyController {
 public:
  explicit WorldCoordManualController(const std::string& node_name);
  ~WorldCoordManualController();

 private:
  const std::string joy_topic_name_{"joy"};
  const std::string input_topic_name_{"world_coord_pose"};
  std::function<void()> timer_callback_;
  Eigen::Vector3d pos;
  double theta;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr world_coord_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void TimerCallback();
};


#endif /* SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_HAND_COORD_CONTROLLER */
