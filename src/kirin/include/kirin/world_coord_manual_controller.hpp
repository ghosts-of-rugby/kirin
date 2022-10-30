#ifndef SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_HAND_COORD_CONTROLLER
#define SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_HAND_COORD_CONTROLLER

#include <string>
#include <optional>
#include <Eigen/Core>
#include <magic_enum.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <kirin_msgs/msg/hand_position.hpp>
#include <kirin_msgs/msg/move_mode.hpp>
#include <kirin_msgs/srv/toggle_hand_state.hpp>
#include <kirin_msgs/srv/set_air_state.hpp>
#include <kirin_msgs/srv/set_target.hpp>
#include <kirin_msgs/srv/start_rapid_hand.hpp>
#include <kirin_msgs/srv/start_planar_auto_movement.hpp>
#include <kirin_msgs/srv/start_z_auto_movement.hpp>
#include "kirin/joy_controller.hpp"
#include "kirin/common_types.hpp"
#include "kirin/frame.hpp"



class WorldCoordManualController : public JoyController {
 public:
  using ZAutoState = kirin_types::ZAutoState;
  enum class InitialAuto {
    Wait,
    Start,
    WaitRapidFinished,
    GoShare,
    WaitHandOpen,
    WaitPicked,
    ZDepart,
    GoPlaceAbove,
    GoPlaceHeight,
    WaitPlaceAdjustment,
    AdjustmentCompleted,
    End
  };

  friend InitialAuto& operator ++ (InitialAuto& state, int) {
    if (state == InitialAuto::End) return state;
    else {
      int num = static_cast<int>(state);
      state = static_cast<InitialAuto>(num + 1);
      return state;
    }
  }

  explicit WorldCoordManualController(const std::string& node_name,
                                      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~WorldCoordManualController();

 private:
  bool is_red_;
  int color_dir_;
  const std::string joy_topic_name_{"joy"};
  std::function<void()> timer_callback_;
  int loop_ms_{20};
  InitialAuto initial_auto_{InitialAuto::Wait};
  bool is_air_on_{false};
  bool is_z_moving{false};
  bool is_planar_moving{false};
  ZAutoState z_auto_state;

  int pick_index{0};
  const int pick_max_index{frame::pick::kNum};
  int place_index{0};
  const int place_max_index{frame::place::kNum};
  std::array<std::string, frame::pick::kNum + 1> pick_target_;
  std::array<std::string, frame::place::kNum + 1> place_target_;
  std::string next_target_{frame::kDepart};
  std::optional<std::string> current_target_;

  kirin_types::HandState hand_state_;
  kirin_types::MoveMode move_mode_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_bellows_pub_, next_target_pub_,
      all_state_msg_pub_;
  rclcpp::Publisher<kirin_msgs::msg::MoveMode>::SharedPtr move_mode_pub_;
  rclcpp::Publisher<kirin_msgs::msg::HandPosition>::SharedPtr manual_vel_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr planar_fin_sub_, z_fin_sub_;
  rclcpp::Client<kirin_msgs::srv::ToggleHandState>::SharedPtr toggle_hand_state_client_;
  rclcpp::Client<kirin_msgs::srv::SetAirState>::SharedPtr set_air_state_client_;
  rclcpp::Client<kirin_msgs::srv::SetTarget>::SharedPtr set_target_client_;
  rclcpp::Client<kirin_msgs::srv::StartRapidHand>::SharedPtr start_rapid_hand_client_;
  rclcpp::Client<kirin_msgs::srv::StartZAutoMovement>::SharedPtr start_z_auto_client_;
  rclcpp::Client<kirin_msgs::srv::StartPlanarAutoMovement>::SharedPtr start_planar_auto_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::function<void()> go_next_initial_auto_callback_;

  void InitialAutoMovement();
  void InitialAutoMovementPlan();
  void GoNextInitialAuto();
  void SetNextInitialAutoCallback(const std::function<void()>& callback);
  void TimerCallback();
  void ChangePumpStateClientRequest();
  void ChangeHandStateClientRequest();
  void StartRapidHandClientRequest();
  void ModeChangeHandler();
  bool AutomaticMovementFinished();

  void PlanarFinishedCallback([[ maybe_unused ]] const std_msgs::msg::Bool::UniquePtr msg);
  void ZFinishedCallback([[ maybe_unused ]] const std_msgs::msg::Bool::UniquePtr msg);

  void PublishBellowsMsg(const std::string& bellows);
  void PublishModeMsg(const kirin_types::MoveMode& mode);
  void PublishNextTargetMsg(const std::string& next_target);
  void PublishAllStateMsg();
  void PublishManualInput();

  bool SetNextTarget(const std::string& next_target);
  bool StartZAutoMovement(const kirin_types::ZAutoState& state);
  bool StartPlanarMovement();
};

#endif /* SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_HAND_COORD_CONTROLLER */
