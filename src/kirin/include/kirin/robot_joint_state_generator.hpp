#ifndef SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_ROBOT_JOINT_STATE_GENERATOR
#define SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_ROBOT_JOINT_STATE_GENERATOR

#include <string>
#include <optional>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <kirin_msgs/msg/hand_position.hpp>
#include <kirin_msgs/srv/set_target.hpp>
#include <kirin_msgs/srv/start_z_auto_movement.hpp>
#include <kirin_msgs/srv/start_planar_auto_movement.hpp>
#include "kirin/common_types.hpp"
#include "kirin/frame.hpp"

class RobotJointStateGenerator : public rclcpp::Node {
 public:
  using JointStateMsg   = sensor_msgs::msg::JointState;
  using StringMsg       = std_msgs::msg::String;
  using SetTarget       = kirin_msgs::srv::SetTarget;
  using StartZAuto      = kirin_msgs::srv::StartZAutoMovement;
  using StartPlanarAuto = kirin_msgs::srv::StartPlanarAutoMovement;
  using PlaneTuple      = std::tuple<Eigen::Vector2d, double>;
  using RPYTuple        = std::tuple<double, double, double>;
  using ZAutoState      = kirin_types::ZAutoState;

  struct VelocityRatio {
    double x;
    double y;
    double z;
    double psi;
  };

  struct ZAuto {
    bool enabled     = false;
    ZAutoState state = ZAutoState::Depart;
    double ratio;
    double max_speed;
    double approach_offset;
  };

  struct PlanarAuto {
    bool enabled = false;
    double xy_ratio;
    double xy_max_speed;
    double psi_ratio;
    double psi_max_speed;
  };

  struct HandPosition {
    Eigen::Vector3d point;
    double psi;
  };

  explicit RobotJointStateGenerator(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~RobotJointStateGenerator();

 private:
  double loop_ms_{20.0};
  bool is_red_;
  int color_dir_;
  HandPosition initial_pos_;
  HandPosition pos_;
  HandPosition vel_;
  VelocityRatio velocity_ratio_;

  std::string next_target_;
  std::string current_target_;

  ZAuto z_auto_;
  PlanarAuto planar_auto_;
  std::optional<Eigen::Vector2d> xy_distance_;
  std::optional<double> psi_distance_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr z_fin_pub_, planar_fin_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr world_coord_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<kirin_msgs::msg::HandPosition>::SharedPtr manual_vel_sub_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::Service<SetTarget>::SharedPtr set_target_srv_;
  rclcpp::Service<StartZAuto>::SharedPtr start_z_srv_;
  rclcpp::Service<StartPlanarAuto>::SharedPtr start_planar_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool IsAllowedToChangeTarget();
  geometry_msgs::msg::Pose GetPose();
  std::optional<std::tuple<double, double>> GetBellowsOffset(const std::string& bellows_frame);
  std::optional<geometry_msgs::msg::Pose> GetPoseFromTf(const std::string& parent_frame,
                                                        const std::string& child_frame);
  std::optional<std::tuple<double, double>> GetDistanceBasedZAutoVelocity(const std::string& target,
                                                                          double offset);
  std::optional<std::tuple<PlaneTuple, PlaneTuple>> GetDistanceBasedPlanarAutoVelocity(
      const std::string& target);
  inline RPYTuple CalcGeometryQuatToRPY(const geometry_msgs::msg::Quaternion& quat);
  bool ValidateAndUpdateTarget();

  inline void StartZAutoMovement();
  inline void FinishZAutoMovement();
  inline void StartPlanarAutoMovement();
  inline void FinishPlanarAutoMovement();

  void PublishJointState(double l, double phi_offset);
  void PublishPlanarFinished();
  void PublishZFinished();

  void HandleSetTarget(const std::shared_ptr<rmw_request_id_t>,
                       const std::shared_ptr<SetTarget::Request>,
                       std::shared_ptr<SetTarget::Response>);
  void HandleStartZAutoMovement(const std::shared_ptr<rmw_request_id_t>,
                                const std::shared_ptr<StartZAuto::Request>,
                                std::shared_ptr<StartZAuto::Response>);
  void HandleStartPlanarAutoMovement(const std::shared_ptr<rmw_request_id_t>,
                                     const std::shared_ptr<StartPlanarAuto::Request>,
                                     std::shared_ptr<StartPlanarAuto::Response>);

  void ReceiveManualInput(const kirin_msgs::msg::HandPosition::UniquePtr manual_vel);
  void TimerCallback();
};

#endif /* SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_ROBOT_JOINT_STATE_GENERATOR */
