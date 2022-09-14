#include <rclcpp/rclcpp.hpp>
#include "kirin/motor_controller.hpp"

int main(int argc, char* argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  // auto options = rclcpp::NodeOptions().use_intra_process_comms(true);

  rclcpp::executors::SingleThreadedExecutor exec;
  auto motor_controller_node = std::make_shared<MotorController>();

  exec.add_node(motor_controller_node);

  exec.spin();

  rclcpp::shutdown();
}