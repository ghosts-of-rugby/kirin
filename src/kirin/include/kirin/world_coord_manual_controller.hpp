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
#include <kirin_msgs/srv/start_rapid_hand.hpp>
#include "kirin/joy_controller.hpp"
#include "kirin/common_types.hpp"
#include "kirin/frame.hpp"



class WorldCoordManualController : public JoyController {
 public:
  using ZAutoState = kirin_types::ZAutoState;
  enum class InitialAuto {
    Wait,
    Start,
    WaitGoShareWait,
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
  ZAutoState z_auto_state;

  int pick_index{0};
  const int pick_max_index{frame::pick::kNum};
  int place_index{0};
  const int place_max_index{frame::place::kNum};
  std::array<std::string, frame::pick::kNum + 1> pick_target_;
  std::array<std::string, frame::place::kNum + 1> place_target_;
  std::string next_target_{frame::kDepart};

  kirin_types::HandState hand_state_;
  kirin_types::MoveMode move_mode_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_bellows_pub_, next_target_pub_,
      all_state_msg_pub_;
  rclcpp::Publisher<kirin_msgs::msg::MoveMode>::SharedPtr move_mode_pub_;
  rclcpp::Client<kirin_msgs::srv::ToggleHandState>::SharedPtr toggle_hand_state_client_;
  rclcpp::Client<kirin_msgs::srv::SetAirState>::SharedPtr set_air_state_client_;
  rclcpp::Client<kirin_msgs::srv::StartRapidHand>::SharedPtr start_rapid_hand_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  void InitialAutoMovement();
  void InitialAutoMovementPlan();
  void TimerCallback();
  void ChangePumpStateClientRequest();
  void ChangeHandStateClientRequest();
  void StartRapidHandClientRequest();
  void ModeChangeHandler();

  void PublishBellowsMsg(const std::string& bellows);
  void PublishModeMsg(const kirin_types::MoveMode& mode);
  void PublishNextTargetMsg(const std::string& next_target);
  void PublishAllStateMsg();

  bool SetNextTarget(const std::string& next_target);
  bool StartZAutoMovement(const kirin_types::ZAutoState& state);
  bool StartPlanarMovement();
};

#endif /* SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_HAND_COORD_CONTROLLER */
