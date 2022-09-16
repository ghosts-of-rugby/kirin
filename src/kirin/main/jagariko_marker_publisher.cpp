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
  static constexpr size_t PLACE_POSE_NUM     = 6;
  static constexpr size_t JAGARIKO_NUM       = OUR_JAGARIKO_NUM + SHARE_JAGARIKO_NUM;
  static constexpr size_t BLOCK_NUM          = (OUR_JAGARIKO_NUM - 1) / 3;

  explicit JagarikoMarkersPublisher(const std::string& topic_name,
                                    const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("jagariko_marker_publisher", options) {
    /* color direction from field */
    bool is_red   = declare_parameter("field", "blue") == "red";
    int color_dir = is_red ? -1.0 : 1.0;

    /* calculate jagariko position */
    // our jaga
    double distance = 0.2;
    double top_x    = color_dir * 0.4;
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

    /* calculate place position */
    // base is place point of rapid hand
    {
      double base_x          = color_dir * 0.874;
      double base_y          = 0.65;
      double x_distance      = 0.1;
      double jagariko_height = 0.08;
      double yaw_odd         = (is_red) ? 0.0 : M_PI;
      double yaw_even        = (is_red) ? -M_PI : 0.0;
      for (int l = 0; l < PLACE_POSE_NUM - 1; l++) {
        double yaw         = (l % 2 == 1) ? yaw_odd : yaw_even;
        place_poses_.at(l) = {base_x + x_distance * (l + 1) * color_dir, base_y, 0.0, yaw};
      }
      place_poses_.at(PLACE_POSE_NUM - 1)
          = {base_x + x_distance * 3 * color_dir, base_y, jagariko_height, 0.0};
    }

    /* create pick target transform */
    // out field pick target
    {
      std::vector<std::tuple<int, std::string>> target = {
          {2,  frame::pick::k1st},
          {6,  frame::pick::k2nd},
          {9,  frame::pick::k3rd},
          {12, frame::pick::k4th},
          {15, frame::pick::k5th}
      };
      for (const auto& [idx, frame_name] : target) {
        auto [x, y] = jaga_poses_.at(idx);
        double z    = 0.08;
        // parent_frame: field(base_link)
        double yaw  = (frame_name == frame::pick::k1st) ? 0.0 : -1.0 * M_PI_2;
        transform_vec_.push_back(CreateTargetTransform(x, y, z, yaw, frame_name));
      }
    }
    // share field target
    {
      std::vector<std::tuple<int, std::string>> target = {
          {2, frame::pick::kShare1},
          {6, frame::pick::kShare2}
      };
      for (const auto& [idx, frame_name] : target) {
        auto [x, y]    = share_jaga_poses_.at(idx);
        double share_z = 0.035;
        double z       = 0.08 + share_z;
        double yaw     = (is_red) ? 0.0 : M_PI;
        transform_vec_.push_back(CreateTargetTransform(x, y, z, yaw, frame_name));
      }
    }
    // share wait target
    {
      double wait_x_offset = -0.2;
      double wait_z_offset = 0.03;
      transform_vec_.push_back(CreateTargetTransform(wait_x_offset, 0.0, wait_z_offset, 0.0,
                                                     frame::kShareWait, frame::pick::kShare2));
    }
    // place target
    {
      std::vector<std::tuple<int, std::string>> target = {
          {0, frame::place::kShare},
          {1, frame::place::k1st  },
          {2, frame::place::k2nd  },
          {3, frame::place::k3rd  },
          {4, frame::place::k4th  },
          {5, frame::place::k5th  },
      };
      for (const auto& [idx, frame_name] : target) {
        auto [x, y, offset, yaw] = place_poses_.at(idx);
        double z                 = 0.08 + offset;
        transform_vec_.push_back(CreateTargetTransform(x, y, z, yaw, frame_name));
      }
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
  geometry_msgs::msg::TransformStamped CreateTargetTransform(double x,
                                                             double y,
                                                             double z,
                                                             double yaw,
                                                             const std::string& frame_name,
                                                             const std::string& parent_frame
                                                             = frame::kBaseLink) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp    = this->get_clock()->now();
    t.header.frame_id = parent_frame;
    t.child_frame_id  = frame_name;

    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = z;
    Eigen::Quaterniond quat(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    t.transform.rotation = Eigen::toMsg(quat);
    return std::move(t);
  }

  void PublishTransform() {
    auto now = this->get_clock()->now();
    for (auto& t : transform_vec_) t.header.stamp = now;
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
  std::array<std::tuple<double, double, double, double>, PLACE_POSE_NUM> place_poses_;
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
