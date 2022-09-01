#ifndef SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_HAND_TOOL_MANAGER
#define SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_HAND_TOOL_MANAGER

#include <functional>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <kirin_msgs/srv/set_hand_state.hpp>

enum class HandState {
  Shrink=0,
  Expand=1
};

class HandToolManager: public rclcpp::Node {
 public:
  using Marker = visualization_msgs::msg::Marker;
  using SetHandState = kirin_msgs::srv::SetHandState;
  explicit HandToolManager();

 private:
  void TimerCallback();
  void SetHandStateCallback(const std::shared_ptr<rmw_request_id_t>,
                            const std::shared_ptr<SetHandState::Request>,
                            std::shared_ptr<SetHandState::Response>);
  HandState hand_state_;
  std::unordered_map<HandState, std::string> resource_map_; 
  std::function<void()> timer_callback_;
  std::function<void(const std::shared_ptr<rmw_request_id_t>,
                     const std::shared_ptr<SetHandState::Request>,
                     std::shared_ptr<SetHandState::Response>)> handle_set_hand_state_;
  rclcpp::Publisher<Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<kirin_msgs::srv::SetHandState>::SharedPtr hand_state_sub_;

};

#endif /* SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_HAND_TOOL_MANAGER */
