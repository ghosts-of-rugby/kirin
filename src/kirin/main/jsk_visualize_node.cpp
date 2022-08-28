#include <memory>
#include <chrono>
#include <string>
#include <time.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class JskVisualizeNode: public rclcpp::Node {
 public:
  explicit JskVisualizeNode()
    : Node("jsk_visualize_node"),
      joint_callback_(std::bind(&JskVisualizeNode::JointCallback, this, std::placeholders::_1)),
      timer_callback_(std::bind(&JskVisualizeNode::TimerCallback, this)){

      using namespace std::chrono_literals;
      rclcpp::QoS qos(rclcpp::KeepLast(10));
      joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("joint_states", qos, joint_callback_);
      theta_pie_chart_pub_ = create_publisher<std_msgs::msg::Float32>("rviz/theta", qos);
      phi_pie_chart_pub_ = create_publisher<std_msgs::msg::Float32>("rviz/phi", qos);
      z_gauge_pub_ = create_publisher<std_msgs::msg::Float32>("rviz/z", qos);
      r_gauge_pub_ = create_publisher<std_msgs::msg::Float32>("rviz/r", qos);
      date_string_pub_ = create_publisher<std_msgs::msg::String>("rviz/date", qos);
      timer_ = create_wall_timer(10ms, timer_callback_);
  }

 private:
  float Rad2Deg(float rad) { return rad * 180.0 / M_PI; }
  std::string GetNowDateString(){
    auto now = get_clock()->now();
    time_t t = now.seconds();
    tm* ptm = localtime(&t);
    char buf[128];
    strftime(buf, sizeof(buf), "%Y/%m/%d %H:%M:%S", ptm);
    return std::string(buf);
  }

  void TimerCallback() {
    auto date_msg = std::make_unique<std_msgs::msg::String>();
    date_msg->data = GetNowDateString();
    date_string_pub_->publish(std::move(date_msg));
  }

  void JointCallback(const sensor_msgs::msg::JointState::UniquePtr msg) {
    // joint = {theta, z, r, phi, phi}
    float theta = msg->position.at(0);
    float z = msg->position.at(1);
    float r = msg->position.at(2);
    float phi = msg->position.at(3);
    
    auto theta_msg = std::make_unique<std_msgs::msg::Float32>();
    theta_msg->data = Rad2Deg(theta);
    theta_pie_chart_pub_->publish(std::move(theta_msg));

    auto z_msg = std::make_unique<std_msgs::msg::Float32>();
    z_msg->data = z;
    z_gauge_pub_->publish(std::move(z_msg));

    auto r_msg = std::make_unique<std_msgs::msg::Float32>();
    float r_offset = 0.345 + 0.201;
    r_msg->data = r + r_offset;
    r_gauge_pub_->publish(std::move(r_msg));

    auto phi_msg = std::make_unique<std_msgs::msg::Float32>();
    phi_msg->data = Rad2Deg(phi);
    phi_pie_chart_pub_->publish(std::move(phi_msg));
  }


  std::function<void(const sensor_msgs::msg::JointState::UniquePtr)> joint_callback_;
  std::function<void()> timer_callback_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr theta_pie_chart_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr phi_pie_chart_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr z_gauge_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr r_gauge_pub_;
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