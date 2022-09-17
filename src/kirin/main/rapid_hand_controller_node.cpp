
#include <rclcpp/rclcpp.hpp>
#include <ddt-motor/uart.hpp>
#include <kirin_msgs/srv/start_rapid_hand.hpp>

using namespace std::chrono_literals;

class RapidHandControllerNode : public rclcpp::Node {
 public:
  using Rapid = kirin_msgs::srv::StartRapidHand;
  explicit RapidHandControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("rapid_hand_controller_node", options) {
    use_hardware_ = declare_parameter("use_hardware", false);
    if (use_hardware_) {
      std::string rapid_usb = declare_parameter("usb_device.rapid_arduino", "");
      uart_                 = std::make_shared<ddt::Uart>("/dev/serial/by-id/" + rapid_usb,
                                          ddt::Uart::BaudRate::B_115200);
    } else {
      RCLCPP_WARN(this->get_logger(), "hardware deactivated");
    }

    rapid_hand_srv_ = create_service<kirin_msgs::srv::StartRapidHand>(
        "start_rapid_hand",
        [this](const std::shared_ptr<rmw_request_id_t> request_header,
               const std::shared_ptr<kirin_msgs::srv::StartRapidHand::Request> request,
               std::shared_ptr<kirin_msgs::srv::StartRapidHand::Response> response) -> void {
          uint8_t data = 'S';
          bool success = SendDataToArduino(data, 1000ms);
          if (success) {
            started_ = true;
            RCLCPP_INFO(this->get_logger(), "rapid hand movement started");
          }
          response->result = success;
        });

    timer_ = create_wall_timer(100ms, [this]() -> void {
      if(!use_hardware_) return;
      if(started_) {
        auto receive = uart_->Receive();
        if (receive.size() == 0) return;
        else {
          if (receive.at(0) == 'F') {
            RCLCPP_INFO(this->get_logger(), "rapid hand movement finished");
          }
        }
      }
    });
  }

  bool SendDataToArduino(uint8_t data, const std::chrono::milliseconds& timeout) {
    if (!use_hardware_) return true;

    auto start = this->get_clock()->now();

    while (rclcpp::ok()) {
      auto now = this->get_clock()->now();
      if (now - start > timeout) {
        RCLCPP_WARN(this->get_logger(),
                    "Sending data to rapid hand arduino is terminated by timeout");
        return false;
      }

      uart_->Send({data});
      std::this_thread::sleep_for(500us);
      auto rececive = uart_->Receive();

      if (rececive.size() == 0) {
        RCLCPP_WARN(this->get_logger(), "receive size is 0");
        std::this_thread::sleep_for(100ms);
        continue;
      }
      if (rececive.at(0) == data) {
        // RCLCPP_INFO(this->get_logger(), "Successfully data returned");
        return true;
      } else {
        RCLCPP_WARN(this->get_logger(), "Wrong data returned");
        std::this_thread::sleep_for(100ms);
        continue;
      }
    }
    return true;
  }

 private:
  bool use_hardware_;
  bool started_{false};
  std::shared_ptr<ddt::Uart> uart_;
  rclcpp::Service<kirin_msgs::srv::StartRapidHand>::SharedPtr rapid_hand_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  // auto options = rclcpp::NodeOptions().use_intra_process_comms(true);
  rclcpp::executors::SingleThreadedExecutor exec;
  auto rapid_hand_controller_node = std::make_shared<RapidHandControllerNode>();
  exec.add_node(rapid_hand_controller_node);
  exec.spin();
  rclcpp::shutdown();
}