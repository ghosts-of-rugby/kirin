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
#include <kirin_msgs/msg/move_mode.hpp>
#include <kirin_msgs/srv/toggle_hand_state.hpp>
#include <kirin_msgs/srv/set_air_state.hpp>
#include "kirin/joy_controller.hpp"
#include "kirin/common_types.hpp"
#include "kirin/frame.hpp"

using RPYTuple = std::tuple<double, double, double>;

template <typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

class WorldCoordManualController : public JoyController {
 public:
  using PlaneTuple = std::tuple<Eigen::Vector2d, double>;
  struct VelocityRatio {
    double x;
    double y;
    double z;
    double psi;
  };

  enum class ZAutoState {
    Approach,
    Depart
  };

  enum class InitialAuto {
    Wait,
    Start,
    GoInitialDepart,
    GoShareWait,
    WaitRapidFinished,
    RapidFinished,
    GoShare,
    WaitPicked,
    Picked,
    GoDepartZ,
    GoDepartXY,
    GoPlaceAbove,
    GoPlaceHeight,
    WaitPlaceAdjustment,
    AdjustmentCompleted,
    GoStandby,
    End
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

  explicit WorldCoordManualController(const std::string& node_name,
                                      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~WorldCoordManualController();

 private:
  bool is_red_;
  int color_dir_;
  const std::string joy_topic_name_{"joy"};
  const std::string input_topic_name_{"world_coord_pose"};
  std::function<void()> timer_callback_;
  Eigen::Vector3d initial_pos_;
  Eigen::Vector3d pos_;
  Eigen::Vector3d vel_;
  double initial_psi_;
  double psi_;
  double dpsi_;
  int loop_ms_{20};
  std::string current_bellows_frame_;
  VelocityRatio velocity_ratio_normal_;
  VelocityRatio velocity_ratio_adjust_;
  InitialAuto initial_auto_{InitialAuto::Wait};
  ZAuto z_auto_;
  PlanarAuto planar_auto_;
  bool is_air_on_{false};

  int pick_index{0};
  const int pick_max_index{frame::pick::kNum};
  int place_index{0};
  const int place_max_index{frame::place::kNum};
  std::array<std::string, frame::pick::kNum + 1> pick_target_;
  std::array<std::string, frame::place::kNum + 1> place_target_;
  std::string next_target_{frame::kDepart};
  std::string current_target_{frame::kDepart};

  kirin_types::HandState hand_state_;
  kirin_types::MoveMode move_mode_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr world_coord_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_bellows_pub_, next_target_pub_,
      all_state_msg_pub_;
  rclcpp::Publisher<kirin_msgs::msg::MoveMode>::SharedPtr move_mode_pub_;
  rclcpp::Client<kirin_msgs::srv::ToggleHandState>::SharedPtr toggle_hand_state_client_;
  rclcpp::Client<kirin_msgs::srv::SetAirState>::SharedPtr set_air_state_client_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::TimerBase::SharedPtr timer_;

  inline geometry_msgs::msg::Pose GetManualPose();
  inline std::optional<geometry_msgs::msg::Pose> GetPoseFromTf(const std::string& parent_frame,
                                                               const std::string& child_frame);
  inline RPYTuple CalcGeometryQuatToRPY(const geometry_msgs::msg::Quaternion& quat);
  void InitialAutoMovement();
  void SetCurrentBellows(const std::string& bellows_frame);
  void ChangeBellows();
  void PublishJointState(double l, double phi_offset);
  void TimerCallback();
  void ChangePumpStateClientRequest();
  void ChangeHandStateClientRequest();
  void ModeChangeHandler();

  std::optional<std::tuple<double, double>> GenerateAutoZVelocity(const std::string& target,
                                                                  double offset);
  std::optional<std::tuple<PlaneTuple, PlaneTuple>> GenerateAutoPlaneVelocity(
      const std::string& target);
  void ValidateAndUpdateTarget();

  void PublishBellowsMsg(const std::string& bellows);
  void PublishModeMsg(const kirin_types::MoveMode& mode);
  void PublishNextTargetMsg(const std::string& next_target);
  void PublishAllStateMsg();
};

#endif /* SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_HAND_COORD_CONTROLLER */
