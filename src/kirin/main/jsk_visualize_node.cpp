#include <memory>
#include <chrono>
#include <string>
#include <time.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <kirin_msgs/msg/motor_state_vector.hpp>
#include "kirin/common_types.hpp"
#include "kirin/utils.hpp"
#include "kirin/frame.hpp"

class JskVisualizeNode : public rclcpp::Node {
 public:
  explicit JskVisualizeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("jsk_visualize_node", options),
        timer_callback_(std::bind(&JskVisualizeNode::TimerCallback, this)) {
    using namespace std::chrono_literals;  // NOLINT
    rclcpp::QoS qos(rclcpp::KeepLast(10));

    // transform listener
    tf_buffer_          = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    /* define subscription */
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", qos,
        std::bind(&JskVisualizeNode::JointCallback, this, std::placeholders::_1));
    next_target_sub_ = create_subscription<std_msgs::msg::String>(
        "next_target", qos,
        std::bind(&JskVisualizeNode::NextTargetCallback, this, std::placeholders::_1));
    ref_motor_callback_
        = std::bind(&JskVisualizeNode::MotorCallback, this, "reference", std::placeholders::_1);
    cur_motor_callback_
        = std::bind(&JskVisualizeNode::MotorCallback, this, "current", std::placeholders::_1);
    ref_motor_sub_ = create_subscription<kirin_msgs::msg::MotorStateVector>(
        "motor/stete/reference", qos, ref_motor_callback_);
    cur_motor_sub_ = create_subscription<kirin_msgs::msg::MotorStateVector>(
        "motor/state/current", qos, cur_motor_callback_);

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

    /* show next target */
    next_target_string_pub_
        = create_publisher<std_msgs::msg::String>("rviz/next_target/string", qos);
    next_target_pose_pub_
        = create_publisher<geometry_msgs::msg::PoseStamped>("rviz/next_target/pose", qos);

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

  void NextTargetCallback(const std_msgs::msg::String::UniquePtr msg) {
    std_msgs::msg::String show_msg;
    show_msg.data = "[Next Target Frame]: " + msg->data;
    next_target_string_pub_->publish(std::move(show_msg));

    auto next_target_pose = kirin_utils::GetPoseFromTf(this->get_logger(), this->tf_buffer_,
                                                       frame::kBaseLink, msg->data);
    if (next_target_pose.has_value()) {
      auto pose_msg             = std::make_unique<geometry_msgs::msg::PoseStamped>();
      pose_msg->header.frame_id = frame::kBaseLink;
      pose_msg->header.stamp    = this->get_clock()->now();
      pose_msg->pose            = next_target_pose.value();
      next_target_pose_pub_->publish(std::move(pose_msg));
    }
  }

  std::function<void(const kirin_msgs::msg::MotorStateVector::UniquePtr)> ref_motor_callback_,
      cur_motor_callback_;
  std::function<void()> timer_callback_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<kirin_msgs::msg::MotorStateVector>::SharedPtr ref_motor_sub_, cur_motor_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr next_target_sub_;
  using F32Publisher = rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr;
  std::unordered_map<kirin_types::JointName, F32Publisher> ref_joint_pub_;
  std::unordered_map<kirin_types::MotorName, F32Publisher> ref_motor_pub_, cur_motor_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr date_string_pub_, next_target_string_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr next_target_pose_pub_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
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