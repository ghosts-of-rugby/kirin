#include <memory>
#include "kirin/manual_controller.h"

int main(int argc, char* argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec;
  auto manual_controller_node = std::make_shared<ManualController>("manual_controller");
  exec.add_node(manual_controller_node);

  exec.spin();

  rclcpp::shutdown();
}