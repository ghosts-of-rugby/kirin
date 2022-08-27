#include <memory>
#include "kirin/direct_manual_controller.hpp"
#include "kirin/world_coord_manual_controller.hpp"

int main(int argc, char* argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec;
  auto manual_controller_node = std::make_shared<DirectManualController>("manual_controller");
  auto world_coord_manual_node = std::make_shared<WorldCoordManualController>("world_coord_controller");

  exec.add_node(world_coord_manual_node);

  exec.spin();

  rclcpp::shutdown();
}