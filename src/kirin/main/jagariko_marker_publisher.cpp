#include <array>
#include <tuple>
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

#include "kirin/frame.hpp"

class JagarikoMarkersPublisher : public rclcpp::Node {
 public:
  using Marker      = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  static constexpr size_t OUR_JAGARIKO_NUM   = 16;
  static constexpr size_t SHARE_JAGARIKO_NUM = 9;
  static constexpr size_t JAGARIKO_NUM       = OUR_JAGARIKO_NUM + SHARE_JAGARIKO_NUM;
  static constexpr size_t BLOCK_NUM          = (OUR_JAGARIKO_NUM - 1) / 3;

  explicit JagarikoMarkersPublisher(const std::string& topic_name,
                                    const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("jagariko_marker_publisher", options) {
    /* calculate jagariko position */
    // our jaga
    double distance = 0.2;
    double top_x    = -1.0 * 0.4;  // if red
    double top_y    = distance * 2.5;
    for (int l = 0; l < BLOCK_NUM; l++) {
      jaga_poses_.at(3 * l + 0) = {top_x, top_y - l * distance};
      jaga_poses_.at(3 * l + 1) = {top_x - distance / 2.0, top_y - (l + 0.5) * distance};
      jaga_poses_.at(3 * l + 2) = {top_x + distance / 2.0, top_y - (l + 0.5) * distance};
    }
    jaga_poses_.at(OUR_JAGARIKO_NUM - 1) = {top_x, top_y - BLOCK_NUM * distance};

    // share jaga
    double share_distance = 0.14;
    double share_top_x    = 0.0;
    double share_top_y    = double(SHARE_JAGARIKO_NUM - 1) / 2.0 * share_distance;
    for (int l = 0; l < SHARE_JAGARIKO_NUM; l++) {
      share_jaga_poses_.at(l) = {share_top_x, share_top_y - l * share_distance};
    }

    /* create pick target transform */
    // out field pick target
    std::vector<std::tuple<int, std::string>> target = {
        {3,  frame::pick::k1st},
        {6,  frame::pick::k2nd},
        {9,  frame::pick::k3rd},
        {12, frame::pick::k4th},
        {15, frame::pick::k5th}
    };
    for (const auto& [idx, frame] : target) {
      auto [x, y] = jaga_poses_.at(idx);
      double z    = 0.08;
      double yaw  = -M_PI_2;  // parent_frame: field(base_link)
      transform_vec_.push_back(CreateJagarikoTransform(x, y, z, yaw, frame));
    }
    // share field target
    {
      auto [x, y]    = share_jaga_poses_.at(2);
      double share_z = 0.035;
      double z       = 0.08 + share_z;
      double yaw     = 0.0;
      transform_vec_.push_back(CreateJagarikoTransform(x, y, z, yaw, frame::pick::kShare));
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_broadcaster_->sendTransform(transform_vec_);

    using namespace std::chrono_literals;
    marker_pub_   = create_publisher<MarkerArray>(topic_name, rclcpp::SystemDefaultsQoS());
    marker_timer_ = create_wall_timer(
        50ms, std::bind(&JagarikoMarkersPublisher::PublishJagarikoMarker, this));
    transform_timer_
        = create_wall_timer(10ms, std::bind(&JagarikoMarkersPublisher::PublishTransform, this));
  }

 private:
  geometry_msgs::msg::TransformStamped CreateJagarikoTransform(
      double x, double y, double z, double yaw, const std::string& frame_name) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp    = this->get_clock()->now();
    t.header.frame_id = frame::kBaseLink;
    t.child_frame_id  = frame_name;

    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = z;
    Eigen::Quaterniond quat(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    t.transform.rotation = Eigen::toMsg(quat);
    return std::move(t);
  }

  void PublishTransform() {
    for (auto& t : transform_vec_) t.header.stamp = this->get_clock()->now();
    tf_broadcaster_->sendTransform(transform_vec_);
  }

  void PublishJagarikoMarker() {
    auto markers_msg = std::make_unique<MarkerArray>();

    for (int i = 0; i < OUR_JAGARIKO_NUM; i++) {
      auto [x, y] = this->jaga_poses_.at(i);
      markers_msg->markers.push_back(this->CreateJagarikoMarker(i, x, y, 0.0));
    }

    for (int i = 0; i < SHARE_JAGARIKO_NUM; i++) {
      auto [x, y]    = this->share_jaga_poses_.at(i);
      double share_z = 0.035;
      markers_msg->markers.push_back(this->CreateJagarikoMarker(100 + i, x, y, share_z));
    }

    this->marker_pub_->publish(std::move(markers_msg));
  }

  Marker CreateJagarikoMarker(int id, double x, double y, double z) {
    Marker marker;
    marker.header.frame_id = frame::kBaseLink;
    marker.header.stamp    = get_clock()->now();

    marker.id            = id;
    marker.ns            = "jagariko";
    marker.type          = Marker::MESH_RESOURCE;
    marker.action        = Marker::ADD;
    marker.mesh_resource = "package://kirin/resources/jagariko.stl";

    marker.pose.position.x    = x;
    marker.pose.position.y    = y;
    marker.pose.position.z    = z;
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 0.0;
    marker.color.g = 0.9;
    marker.color.b = 0.0;
    marker.color.a = 0.8;

    return marker;
  }

  std::array<std::tuple<double, double>, OUR_JAGARIKO_NUM> jaga_poses_;
  std::array<std::tuple<double, double>, SHARE_JAGARIKO_NUM> share_jaga_poses_;
  std::vector<geometry_msgs::msg::TransformStamped> transform_vec_;

  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr transform_timer_;
};

int main(int argc, char* argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  auto node = std::make_shared<JagarikoMarkersPublisher>("jagariko_markers");
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
