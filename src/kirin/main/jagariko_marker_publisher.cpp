#include <array>
#include <tuple>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

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
    // our jaga
    float distance = 0.2;
    float top_x    = -1.0 * 0.4;  // if red
    float top_y    = distance * 2.5;
    for (int l = 0; l < BLOCK_NUM; l++) {
      jaga_poses_.at(3 * l + 0) = {top_x, top_y - l * distance};
      jaga_poses_.at(3 * l + 1) = {top_x - distance / 2.0, top_y - (l + 0.5) * distance};
      jaga_poses_.at(3 * l + 2) = {top_x + distance / 2.0, top_y - (l + 0.5) * distance};
    }
    jaga_poses_.at(OUR_JAGARIKO_NUM - 1) = {top_x, top_y - BLOCK_NUM * distance};

    // share jaga
    float share_distance = 0.14;
    float share_top_x    = 0.0;
    float share_top_y    = float(SHARE_JAGARIKO_NUM - 1) / 2.0 * share_distance;
    for (int l = 0; l < SHARE_JAGARIKO_NUM; l++) {
      share_jaga_poses_.at(l) = {share_top_x, share_top_y - l * share_distance};
    }

    using namespace std::chrono_literals;
    marker_pub_   = create_publisher<MarkerArray>(topic_name, rclcpp::SystemDefaultsQoS());
    marker_timer_ = create_wall_timer(
        50ms, std::bind(&JagarikoMarkersPublisher::PublishJagarikoMarker, this));
  }

 private:
  void PublishJagarikoMarker() {
    auto markers_msg = std::make_unique<MarkerArray>();

    for (int i = 0; i < OUR_JAGARIKO_NUM; i++) {
      auto [x, y] = this->jaga_poses_.at(i);
      markers_msg->markers.push_back(this->CreateJagarikoMarker(i, x, y, 0.0));
    }

    for (int i = 0; i < SHARE_JAGARIKO_NUM; i++) {
      auto [x, y]   = this->share_jaga_poses_.at(i);
      float share_z = 0.035;
      markers_msg->markers.push_back(this->CreateJagarikoMarker(100 + i, x, y, share_z));
    }

    this->marker_pub_->publish(std::move(markers_msg));
  }

  Marker CreateJagarikoMarker(int id, float x, float y, float z) {
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

  std::array<std::tuple<float, float>, OUR_JAGARIKO_NUM> jaga_poses_;
  std::array<std::tuple<float, float>, SHARE_JAGARIKO_NUM> share_jaga_poses_;

  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
};

int main(int argc, char* argv[]) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  auto node = std::make_shared<JagarikoMarkersPublisher>("jagariko_markers");
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
