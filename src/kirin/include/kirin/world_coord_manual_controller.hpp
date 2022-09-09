#ifndef SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_HAND_COORD_CONTROLLER
#define SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_HAND_COORD_CONTROLLER

#include <string>
#include <optional>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <kirin_msgs/srv/toggle_hand_state.hpp>
#include <kirin_msgs/srv/set_air_state.hpp>
#include "kirin/joy_controller.hpp"

using RPYTuple = std::tuple<double, double, double>;

class WorldCoordManualController : public JoyController {
 public:
  struct VelocityRatio {
    double x;
    double y;
    double z;
    double psi;
  };
  explicit WorldCoordManualController(const std::string& node_name,
                                      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~WorldCoordManualController();

 private:
  const std::string joy_topic_name_{"joy"};
  const std::string input_topic_name_{"world_coord_pose"};
  std::function<void()> timer_callback_;
  Eigen::Vector3d pos_;
  Eigen::Vector3d vel_;
  double psi_;
  double dpsi_;
  int loop_ms_{20};
  std::string current_bellows_frame_;
  VelocityRatio velocity_ratio;
  bool is_air_on{false};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr world_coord_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_bellows_pub_;
  rclcpp::Client<kirin_msgs::srv::ToggleHandState>::SharedPtr toggle_hand_state_client_;
  rclcpp::Client<kirin_msgs::srv::SetAirState>::SharedPtr set_air_state_client_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::TimerBase::SharedPtr timer_;

  inline geometry_msgs::msg::Pose GetManualInput();
  inline std::optional<geometry_msgs::msg::Pose> GetPoseFromTf(const std::string& parent_frame,
                                                               const std::string& child_frame);
  inline RPYTuple CalcGeometryQuatToRPY(const geometry_msgs::msg::Quaternion& quat);
  void SetCurrentBellows(const std::string& bellows_frame);
  void ChangeBellows();
  void PublishJointState(double l, double phi_offset);
  void TimerCallback();
};

#endif /* SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_HAND_COORD_CONTROLLER */
