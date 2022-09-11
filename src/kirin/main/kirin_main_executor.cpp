#include <memory>
#include "kirin/direct_manual_controller.hpp"
#include "kirin/world_coord_manual_controller.hpp"
#include "kirin/hand_tool_manager.hpp"
#include "kirin/joint_to_motor_converter.hpp"
#include "kirin/motor_controller.hpp"

int main(int argc, char* argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  // auto options = rclcpp::NodeOptions().use_intra_process_comms(true);

  // rclcpp::executors::MultiThreadedExecutor exec;
  rclcpp::executors::MultiThreadedExecutor exec;
  // auto manual_controller_node = std::make_shared<DirectManualController>("manual_controller");
  auto world_coord_manual_node =
      std::make_shared<WorldCoordManualController>("world_coord_controller");
  auto hand_tool_manager_node = std::make_shared<HandToolManager>();
  auto joint_to_motor_converter_node = std::make_shared<JointToMotorConverter>();
  auto motor_controller_node = std::make_shared<MotorController>();

  exec.add_node(world_coord_manual_node);
  exec.add_node(hand_tool_manager_node);
  exec.add_node(joint_to_motor_converter_node);
  exec.add_node(motor_controller_node);

  exec.spin();

  rclcpp::shutdown();
}