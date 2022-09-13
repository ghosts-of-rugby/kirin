#include <memory>
#include <chrono>
#include <string>
#include <time.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <kirin_msgs/msg/motor_state_vector.hpp>
#include "kirin/common_types.hpp"

class JskVisualizeNode : public rclcpp::Node {
 public:
  explicit JskVisualizeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("jsk_visualize_node", options),
        joint_callback_(std::bind(&JskVisualizeNode::JointCallback, this, std::placeholders::_1)),
        timer_callback_(std::bind(&JskVisualizeNode::TimerCallback, this)) {
    using namespace std::chrono_literals;  // NOLINT
    rclcpp::QoS qos(rclcpp::KeepLast(10));

    /* define subscription */
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", qos,
        std::bind(&JskVisualizeNode::JointCallback, this, std::placeholders::_1));
    ref_motor_sub_ = create_subscription<kirin_msgs::msg::MotorStateVector>(
        "motor/reference", qos,
        std::bind(&JskVisualizeNode::MotorCallback, this, "reference", std::placeholders::_1));
    cur_motor_sub_ = create_subscription<kirin_msgs::msg::MotorStateVector>(
        "motor/current", qos,
        std::bind(&JskVisualizeNode::MotorCallback, this, "current", std::placeholders::_1));

    /* show joint state */
    {
      using JointName = kirin_types::JointName;
      using Float32   = std_msgs::msg::Float32;
      ref_joint_pub_  = {
           {JointName::Theta, create_publisher<Float32>("rviz/theta", qos)},
           {JointName::Phi,   create_publisher<Float32>("rviz/phi",   qos)},
           {JointName::Z,     create_publisher<Float32>("rviz/z",     qos)},
           {JointName::R,     create_publisher<Float32>("rviz/r",     qos)}
      };
    }

    /* show motor angle */
    {
      using MotorName = kirin_types::MotorName;
      using Float32   = std_msgs::msg::Float32;
      ref_motor_pub_  = {
           {MotorName::Left,  create_publisher<Float32>("rviz/motor/left/reference",  qos)},
           {MotorName::Right, create_publisher<Float32>("rviz/motor/right/reference", qos)},
           {MotorName::Theta, create_publisher<Float32>("rviz/motor/theta/reference", qos)},
           {MotorName::Z,     create_publisher<Float32>("rviz/motor/z/reference",     qos)}
      };
      cur_motor_pub_ = {
          {MotorName::Left,  create_publisher<Float32>("rviz/motor/left/current",  qos)},
          {MotorName::Right, create_publisher<Float32>("rviz/motor/right/current", qos)},
          {MotorName::Theta, create_publisher<Float32>("rviz/motor/theta/current", qos)},
          {MotorName::Z,     create_publisher<Float32>("rviz/motor/z/current",     qos)}
      };
    }

    /* show date */
    date_string_pub_ = create_publisher<std_msgs::msg::String>("rviz/date", qos);

    /* regularly update */
    timer_ = create_wall_timer(100ms, timer_callback_);
  }

 private:
  float Rad2Deg(float rad) { return rad * 180.0 / M_PI; }

  std::string GetNowDateString() {
    auto now = get_clock()->now();
    time_t t = now.seconds();
    tm* ptm  = localtime(&t);
    char buf[128];
    strftime(buf, sizeof(buf), "%Y/%m/%d %H:%M:%S", ptm);
    return std::string(buf);
  }

  void TimerCallback() {
    auto date_msg  = std::make_unique<std_msgs::msg::String>();
    date_msg->data = GetNowDateString();
    date_string_pub_->publish(std::move(date_msg));
  }

  void JointCallback(const sensor_msgs::msg::JointState::UniquePtr msg) {
    // joint = {theta, z, r, phi, phi}
    float theta = msg->position.at(0);
    float z     = msg->position.at(1);
    float r     = msg->position.at(2);
    float phi   = msg->position.at(3);

    using Joint     = kirin_types::JointName;
    auto theta_msg  = std::make_unique<std_msgs::msg::Float32>();
    theta_msg->data = Rad2Deg(theta);
    ref_joint_pub_.at(Joint::Theta)->publish(std::move(theta_msg));

    auto z_msg  = std::make_unique<std_msgs::msg::Float32>();
    z_msg->data = z;
    ref_joint_pub_.at(Joint::Z)->publish(std::move(z_msg));

    auto r_msg     = std::make_unique<std_msgs::msg::Float32>();
    float r_offset = 0.345 + 0.201;
    r_msg->data    = r + r_offset;
    ref_joint_pub_.at(Joint::R)->publish(std::move(r_msg));

    auto phi_msg  = std::make_unique<std_msgs::msg::Float32>();
    phi_msg->data = Rad2Deg(phi);
    ref_joint_pub_.at(Joint::Phi)->publish(std::move(phi_msg));
  }

  void MotorCallback(const std::string& target,
                     const kirin_msgs::msg::MotorStateVector::UniquePtr msg) {
    const auto& motor_pub = (target == "reference") ? this->ref_motor_pub_ : this->cur_motor_pub_;
    using Motor           = kirin_types::MotorName;
    auto theta_msg        = std::make_unique<std_msgs::msg::Float32>();
    theta_msg->data       = Rad2Deg(msg->angle.theta);
    motor_pub.at(Motor::Theta)->publish(std::move(theta_msg));

    auto left_msg  = std::make_unique<std_msgs::msg::Float32>();
    left_msg->data = Rad2Deg(msg->angle.left);
    motor_pub.at(Motor::Left)->publish(std::move(left_msg));

    auto right_msg  = std::make_unique<std_msgs::msg::Float32>();
    right_msg->data = Rad2Deg(msg->angle.right);
    motor_pub.at(Motor::Right)->publish(std::move(right_msg));

    auto z_msg  = std::make_unique<std_msgs::msg::Float32>();
    z_msg->data = Rad2Deg(msg->angle.z);
    motor_pub.at(Motor::Z)->publish(std::move(z_msg));
  }

  std::function<void(const sensor_msgs::msg::JointState::UniquePtr)> joint_callback_;
  std::function<void(const kirin_msgs::msg::MotorStateVector::UniquePtr)> motor_callback_;
  std::function<void()> timer_callback_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<kirin_msgs::msg::MotorStateVector>::SharedPtr ref_motor_sub_;
  rclcpp::Subscription<kirin_msgs::msg::MotorStateVector>::SharedPtr cur_motor_sub_;
  using F32Publisher = rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr;
  std::unordered_map<kirin_types::JointName, F32Publisher> ref_joint_pub_;
  std::unordered_map<kirin_types::MotorName, F32Publisher> ref_motor_pub_;
  std::unordered_map<kirin_types::MotorName, F32Publisher> cur_motor_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr date_string_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  auto node = std::make_shared<JskVisualizeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}