#ifndef SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_HAND_TOOL_MANAGER
#define SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_HAND_TOOL_MANAGER

#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Core>
#include <functional>
#include <kirin_msgs/srv/set_air_state.hpp>
#include <kirin_msgs/srv/set_hand_state.hpp>
#include <kirin_msgs/srv/toggle_hand_state.hpp>
#include <kirin_msgs/srv/set_color_led.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tuple>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include "kirin/common_types.hpp"
#include "ddt-motor/uart.hpp"

using HandState = kirin_types::HandState;
using LED = kirin_types::ColorLED;

class HandToolManager : public rclcpp::Node {
 public:
  using Marker               = visualization_msgs::msg::Marker;
  using SetHandState         = kirin_msgs::srv::SetHandState;
  using ToggleHandState      = kirin_msgs::srv::ToggleHandState;
  using SetAirState          = kirin_msgs::srv::SetAirState;
  using BellowsPositionTuple = std::tuple<std::string, Eigen::Vector2d>;
  explicit HandToolManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void MarkerTimerCallback();
  void SetHandStateCallback(const std::shared_ptr<rmw_request_id_t>,
                            const std::shared_ptr<SetHandState::Request>,
                            std::shared_ptr<SetHandState::Response>);
  void ToggleHandStateCallback(const std::shared_ptr<rmw_request_id_t>,
                               const std::shared_ptr<ToggleHandState::Request>,
                               std::shared_ptr<ToggleHandState::Response>);
  void SetAirStateCallback(const std::shared_ptr<rmw_request_id_t>,
                           const std::shared_ptr<SetAirState::Request>,
                           std::shared_ptr<SetAirState::Response>);
  void UpdateBellowsTransformVector(HandState hand_state);
  bool SendDataToArduino(uint8_t data,
                         const std::chrono::milliseconds& timeout
                         = std::chrono::milliseconds(1000));
  void SetColorLED(const kirin_types::ColorLED& color_led);
  void UpdateColorLED();

  bool use_hardware_;
  bool is_air_on_{false};
  std::shared_ptr<ddt::Uart> pump_arduino_uart_;
  HandState hand_state_;
  std::unordered_map<HandState, std::string> resource_map_;
  std::unordered_map<HandState, std::array<BellowsPositionTuple, 3>> bellows_map_;
  std::unordered_map<kirin_types::BellowsName, kirin_types::AirState> air_map_;
  std::vector<geometry_msgs::msg::TransformStamped> transform_vec_;

  std::function<void()> marker_timer_callback_;
  std::function<void(const std::shared_ptr<rmw_request_id_t>,
                     const std::shared_ptr<SetHandState::Request>,
                     std::shared_ptr<SetHandState::Response>)>
      handle_set_hand_state_;
  std::function<void(const std::shared_ptr<rmw_request_id_t>,
                     const std::shared_ptr<ToggleHandState::Request>,
                     std::shared_ptr<ToggleHandState::Response>)>
      handle_toggle_hand_state_;
  std::function<void(const std::shared_ptr<rmw_request_id_t>,
                     const std::shared_ptr<SetAirState::Request>,
                     std::shared_ptr<SetAirState::Response>)>
      handle_set_air_state_;

  rclcpp::Publisher<Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<kirin_msgs::srv::SetHandState>::SharedPtr set_hand_srv_;
  rclcpp::Service<kirin_msgs::srv::ToggleHandState>::SharedPtr toggle_hand_srv_;
  rclcpp::Service<kirin_msgs::srv::SetAirState>::SharedPtr set_air_srv_;
  rclcpp::Client<kirin_msgs::srv::SetColorLED>::SharedPtr set_color_client_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

#endif /* SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_HAND_TOOL_MANAGER */
