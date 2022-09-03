#ifndef SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_HAND_TOOL_MANAGER
#define SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_HAND_TOOL_MANAGER

#include <functional>
#include <string>
#include <tuple>
#include <vector>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <kirin_msgs/srv/set_hand_state.hpp>
#include <kirin_msgs/srv/toggle_hand_state.hpp>

enum class HandState {
  Shrink=0,
  Extend=1
};

class HandToolManager: public rclcpp::Node {
 public:
  using Marker = visualization_msgs::msg::Marker;
  using SetHandState = kirin_msgs::srv::SetHandState;
  using ToggleHandState = kirin_msgs::srv::ToggleHandState;
  using BellowsPositionTuple = std::tuple<std::string, Eigen::Vector2d>;
  explicit HandToolManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void TimerCallback();
  void SetHandStateCallback(const std::shared_ptr<rmw_request_id_t>,
                            const std::shared_ptr<SetHandState::Request>,
                            std::shared_ptr<SetHandState::Response>);
  void ToggleHandStateCallback(const std::shared_ptr<rmw_request_id_t>,
                               const std::shared_ptr<ToggleHandState::Request>,
                               std::shared_ptr<ToggleHandState::Response>);
  void UpdateBellowsTransformVector(HandState hand_state);
  HandState hand_state_;
  std::unordered_map<HandState, std::string> resource_map_;
  std::unordered_map<HandState, std::array<BellowsPositionTuple, 3>> bellows_map_;
  std::vector<geometry_msgs::msg::TransformStamped> transform_vec_;
  std::function<void()> timer_callback_;
  std::function<void(const std::shared_ptr<rmw_request_id_t>,
                     const std::shared_ptr<SetHandState::Request>,
                     std::shared_ptr<SetHandState::Response>)> handle_set_hand_state_;
  std::function<void(const std::shared_ptr<rmw_request_id_t>,
                     const std::shared_ptr<ToggleHandState::Request>,
                     std::shared_ptr<ToggleHandState::Response>)> handle_toggle_hand_state_;
  rclcpp::Publisher<Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<kirin_msgs::srv::SetHandState>::SharedPtr set_srv_;
  rclcpp::Service<kirin_msgs::srv::ToggleHandState>::SharedPtr toggle_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};

#endif /* SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_HAND_TOOL_MANAGER */
