
#include <rclcpp/rclcpp.hpp>
#include "kirin/hand_tool_manager.hpp"

int main(int argc, char* argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  // auto options = rclcpp::NodeOptions().use_intra_process_comms(true);

  rclcpp::executors::SingleThreadedExecutor exec;
  auto hand_tool_manager_node = std::make_shared<HandToolManager>();

  exec.add_node(hand_tool_manager_node);
  exec.spin();
  rclcpp::shutdown();
}