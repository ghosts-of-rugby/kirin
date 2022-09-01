#include <memory>
#include "kirin/direct_manual_controller.hpp"
#include "kirin/world_coord_manual_controller.hpp"
#include "kirin/hand_tool_manager.hpp"

int main(int argc, char* argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  // auto options = rclcpp::NodeOptions().use_intra_process_comms(true);

  // rclcpp::executors::MultiThreadedExecutor exec;
  rclcpp::executors::MultiThreadedExecutor exec;
  // auto manual_controller_node = std::make_shared<DirectManualController>("manual_controller");
  auto world_coord_manual_node = std::make_shared<WorldCoordManualController>("world_coord_controller");
  auto hand_tool_manager_node = std::make_shared<HandToolManager>();

  exec.add_node(world_coord_manual_node);
  exec.add_node(hand_tool_manager_node);

  exec.spin();

  rclcpp::shutdown();
}