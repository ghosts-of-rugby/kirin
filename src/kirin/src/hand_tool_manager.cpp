#include "kirin/hand_tool_manager.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <magic_enum.hpp>

#include "kirin/frame.hpp"
#include "kirin/machine.hpp"

using namespace std::chrono_literals;  // NOLINT

HandToolManager::HandToolManager(const rclcpp::NodeOptions& options)
    : Node("hand_tool_manager", options),
      // arduino_uart_("/dev/ttyACM0", ddt::Uart::BaudRate::B_115200),
      hand_state_(HandState::Shrink),
      marker_timer_callback_(std::bind(&HandToolManager::MarkerTimerCallback, this)),
      handle_set_hand_state_(std::bind(&HandToolManager::SetHandStateCallback,
                                       this,
                                       std::placeholders::_1,
                                       std::placeholders::_2,
                                       std::placeholders::_3)),
      handle_toggle_hand_state_(std::bind(&HandToolManager::ToggleHandStateCallback,
                                          this,
                                          std::placeholders::_1,
                                          std::placeholders::_2,
                                          std::placeholders::_3)),
      handle_set_air_state_(std::bind(&HandToolManager::SetAirStateCallback,
                                      this,
                                      std::placeholders::_1,
                                      std::placeholders::_2,
                                      std::placeholders::_3)) {
  /* open arduino */
  use_hardware_ = declare_parameter("use_hardware", false);
  if (use_hardware_) {
    std::string pump_usb = declare_parameter("usb_device.pump_arduino", "");
    pump_arduino_uart_
        = std::make_shared<ddt::Uart>("/dev/serial/by-id/" + pump_usb, ddt::Uart::BaudRate::B_115200);
    
    {
      pump_arduino_uart_->Send({0x00});
      std::this_thread::sleep_for(10ms);
      auto receive = pump_arduino_uart_->Receive();
      if (receive.size() == 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to Receive Data from Arduino");
        RCLCPP_INFO(this->get_logger(), "Reopen Arduino Port");
        pump_arduino_uart_->Close();
        std::this_thread::sleep_for(100ms);
        pump_arduino_uart_->Open();
        pump_arduino_uart_->Send({0x00});
        std::this_thread::sleep_for(10ms);
        auto receive = pump_arduino_uart_->Receive();
        if (receive.size() == 0) {
          RCLCPP_ERROR(this->get_logger(), "Second Trial to Connect Arduino is Failed");
          rclcpp::shutdown();
        } else {
          RCLCPP_INFO(this->get_logger(), "Successfully Connected");
        }
      } else {
        RCLCPP_INFO(this->get_logger(), "Successfully Connected");
      }
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "hardware deactivated");
  }

  std::string mesh_directory = "package://kirin/resources/light";

  resource_map_ = {
      {HandState::Shrink, mesh_directory + "/phi.stl"       },
      {HandState::Extend, mesh_directory + "/phi_extend.stl"}
  };

  using namespace machine::bellows;  // NOLINT
  bellows_map_ = {
      {HandState::Shrink,
       {BellowsPositionTuple{frame::kBellowsTop, {kTop_X, 0.0}},
        BellowsPositionTuple{frame::kBellowsLeft, {kTop_X - kShrink_D, kShrink_D}},
        BellowsPositionTuple{frame::kBellowsRight, {kTop_X - kShrink_D, -kShrink_D}}}},
      {HandState::Extend,
       {BellowsPositionTuple{frame::kBellowsTop, {kTop_X, 0.0}},
        BellowsPositionTuple{frame::kBellowsLeft, {kTop_X, kExtend_D}},
        BellowsPositionTuple{frame::kBellowsRight, {kTop_X, -kExtend_D}}}            }
  };
  transform_vec_.reserve(5);
  UpdateBellowsTransformVector(this->hand_state_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_broadcaster_->sendTransform(transform_vec_);

  air_map_ = {
      {kirin_types::BellowsName::Top,     kirin_types::AirState::Off},
      {kirin_types::BellowsName::Left,    kirin_types::AirState::Off},
      {kirin_types::BellowsName::Right,   kirin_types::AirState::Off},
      {kirin_types::BellowsName::ExLeft,  kirin_types::AirState::Off},
      {kirin_types::BellowsName::ExRight, kirin_types::AirState::Off},
  };

  marker_pub_   = create_publisher<Marker>("hand_marker", rclcpp::QoS(rclcpp::KeepLast(5)));
  timer_        = create_wall_timer(10ms, marker_timer_callback_);
  set_hand_srv_ = create_service<SetHandState>("tool/set_hand_state", handle_set_hand_state_);
  toggle_hand_srv_
      = create_service<ToggleHandState>("tool/toggle_hand_state", handle_toggle_hand_state_);
  set_air_srv_ = create_service<SetAirState>("tool/set_air_state", handle_set_air_state_);
  set_color_client_ = create_client<kirin_msgs::srv::SetColorLED>("/set_color_led");
}

void HandToolManager::MarkerTimerCallback() {
  auto marker_msg             = std::make_unique<HandToolManager::Marker>();
  auto now                    = get_clock()->now();
  marker_msg->header.frame_id = frame::kPhiLink;
  marker_msg->header.stamp    = now;

  marker_msg->id            = 1;
  marker_msg->ns            = "tool";
  marker_msg->type          = Marker::MESH_RESOURCE;
  marker_msg->action        = Marker::ADD;
  marker_msg->mesh_resource = resource_map_.at(hand_state_);

  marker_msg->pose.position.x    = 0;
  marker_msg->pose.position.y    = 0;
  marker_msg->pose.position.z    = 0;
  marker_msg->pose.orientation.w = 1.0;
  marker_msg->pose.orientation.x = 0.0;
  marker_msg->pose.orientation.y = 0.0;
  marker_msg->pose.orientation.z = 0.0;
  marker_msg->scale.x            = 1.0;
  marker_msg->scale.y            = 1.0;
  marker_msg->scale.z            = 1.0;

  marker_msg->color.r = 0.8;
  marker_msg->color.g = 0.3;
  marker_msg->color.b = 0.3;
  marker_msg->color.a = 1.0;

  marker_pub_->publish(std::move(marker_msg));

  // publish bellows tf
  for (auto& t : transform_vec_) t.header.stamp = now;
  tf_broadcaster_->sendTransform(transform_vec_);
}

void HandToolManager::SetHandStateCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                           const std::shared_ptr<SetHandState::Request> request,
                                           std::shared_ptr<SetHandState::Response> response) {
  (void)request_header;  // Lint Tool ??????
  auto next_state = magic_enum::enum_cast<HandState>(request->hand_state.value);

  if (next_state.has_value()) {
    /* send data to arduino */
    uint8_t data = 0x00;
    uint8_t mode = 0x01;
    data |= (static_cast<bool>(next_state) << 6);
    data |= (mode << 7);
    bool success = SendDataToArduino(data);

    RCLCPP_INFO(this->get_logger(), "ToolState: '%s' -> '%s'",
                magic_enum::enum_name(this->hand_state_).data(),
                magic_enum::enum_name(next_state.value()).data());
    this->hand_state_ = next_state.value();
    UpdateColorLED();
    // update transform
    UpdateBellowsTransformVector(this->hand_state_);
    response->result = true;
  }
}

void HandToolManager::ToggleHandStateCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ToggleHandState::Request> request,
    std::shared_ptr<ToggleHandState::Response> response) {
  (void)request_header;  // Lint Tool ??????
  auto next_state
      = (this->hand_state_ == HandState::Extend ? HandState::Shrink : HandState::Extend);

  /* send data to arduino */
  uint8_t data = 0x00;
  uint8_t mode = 0x01;
  data |= (static_cast<bool>(next_state) << 6);
  data |= (mode << 7);
  bool success = SendDataToArduino(data);

  if (!success) {  // send failed
    response->success = false;
    return;
  }

  RCLCPP_INFO(this->get_logger(), "ToolState: '%s' -> '%s'",
              magic_enum::enum_name(this->hand_state_).data(),
              magic_enum::enum_name(next_state).data());
  this->hand_state_ = next_state;
  UpdateColorLED();

  // update transform
  UpdateBellowsTransformVector(this->hand_state_);

  kirin_msgs::msg::HandState ret;
  ret.value               = static_cast<int>(this->hand_state_);
  response->current_state = ret;
  response->success       = true;
}

void HandToolManager::SetAirStateCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                          const std::shared_ptr<SetAirState::Request> request,
                                          std::shared_ptr<SetAirState::Response> response) {
  using Bellows = kirin_types::BellowsName;
  using Air     = kirin_types::AirState;

  (void)request_header;  // Lint Tool ??????

  std::unordered_map<Bellows, Air> next_state{
      {Bellows::Top,     static_cast<Air>(request->air_state.top)     },
      {Bellows::Left,    static_cast<Air>(request->air_state.left)    },
      {Bellows::Right,   static_cast<Air>(request->air_state.right)   },
      {Bellows::ExLeft,  static_cast<Air>(request->air_state.ex_left) },
      {Bellows::ExRight, static_cast<Air>(request->air_state.ex_right)},
  };
  bool release = request->air_state.release;

  /* Send Uart to Arduino */
  uint8_t data = 0x00;
  uint8_t mode = 0x00;
  data |= (static_cast<bool>(next_state.at(Bellows::ExLeft)) << 0);
  data |= (static_cast<bool>(next_state.at(Bellows::Left)) << 1);
  data |= (static_cast<bool>(next_state.at(Bellows::Top)) << 2);
  data |= (static_cast<bool>(next_state.at(Bellows::Right)) << 3);
  data |= (static_cast<bool>(next_state.at(Bellows::ExRight)) << 4);
  data |= (static_cast<bool>(release) << 5);
  data |= (mode << 7);
  bool success = SendDataToArduino(data);

  if (success) {
    if (release) RCLCPP_INFO(this->get_logger(), "Air Released!");

    for (const auto& [key, next_value] : next_state) {
      if (air_map_.at(key) != next_value) {
        RCLCPP_INFO(this->get_logger(), "AirState [%s]: '%s' -> '%s'",
                    magic_enum::enum_name(key).data(),
                    magic_enum::enum_name(air_map_.at(key)).data(),
                    magic_enum::enum_name(next_value).data());
      }
    }
    this->air_map_ = next_state;
    is_air_on_ = !release;
  }
  UpdateColorLED();
  response->result = success;
}

bool HandToolManager::SendDataToArduino(uint8_t data, const std::chrono::milliseconds& timeout) {
  if (!use_hardware_) return true;

  auto start = this->get_clock()->now();

  while (rclcpp::ok()) {
    auto now = this->get_clock()->now();
    if (now - start > timeout) {
      RCLCPP_WARN(this->get_logger(), "Sending data to arduino is failed");
      return false;
    }

    pump_arduino_uart_->Send({data});
    std::this_thread::sleep_for(500us);
    auto rececive = pump_arduino_uart_->Receive();

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

void HandToolManager::UpdateBellowsTransformVector(HandState hand_state) {
  transform_vec_.clear();
  for (const auto& [bellows_frame, pos] : bellows_map_.at(this->hand_state_)) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp    = this->get_clock()->now();
    t.header.frame_id = frame::kPhiLink;
    t.child_frame_id  = bellows_frame;

    t.transform.translation.x = pos.x();
    t.transform.translation.y = pos.y();
    t.transform.translation.z = machine::bellows::kZ;
    t.transform.rotation.x    = 0.0;
    t.transform.rotation.y    = 0.0;
    t.transform.rotation.z    = 0.0;
    t.transform.rotation.w    = 1.0;
    transform_vec_.push_back(std::move(t));
  }
}

void HandToolManager::SetColorLED(const kirin_types::ColorLED& color_led) {
  auto request   = std::make_shared<kirin_msgs::srv::SetColorLED::Request>();
  kirin_msgs::msg::ColorLED msg;
  msg.value = static_cast<int32_t>(color_led);
  request->color = msg;

  using ResponseFuture   = rclcpp::Client<kirin_msgs::srv::SetColorLED>::SharedFuture;
  auto response_callback = [this, color_led](ResponseFuture future) {
    auto response = future.get();
    if (response->result) {
      RCLCPP_INFO(this->get_logger(), "Current Color: [%s]",
                  magic_enum::enum_name(color_led).data());
    } else {  // failed
      RCLCPP_ERROR(this->get_logger(), "Color change failed!");
    }
  };

  auto future_result = set_color_client_->async_send_request(request, response_callback);
}

void HandToolManager::UpdateColorLED() {
  LED led = LED::Cyan;
  if (hand_state_ == HandState::Shrink) {
    if (is_air_on_) led = LED::Green;
    else led = LED::Cyan;
  } else {
    if (is_air_on_) led = LED::Yellow;
    else led = LED::Pink;
  }
  SetColorLED(led);
}