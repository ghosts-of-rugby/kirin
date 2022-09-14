
#include <rclcpp/rclcpp.hpp>
#include "kirin/utils.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace kirin_utils {

std::optional<geometry_msgs::msg::Pose> GetPoseFromTf(const rclcpp::Logger& node_logger,
                                                      std::shared_ptr<tf2_ros::Buffer> buffer_ptr,
                                                      const std::string& parent_frame,
                                                      const std::string& child_frame) {
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = buffer_ptr->lookupTransform(parent_frame, child_frame, tf2::TimePointZero);
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(node_logger, "Could not transform %s to %s: %s", parent_frame.c_str(),
                child_frame.c_str(), ex.what());
    return std::nullopt;
  }

  geometry_msgs::msg::Pose pose;
  pose.position.x  = transform_stamped.transform.translation.x;
  pose.position.y  = transform_stamped.transform.translation.y;
  pose.position.z  = transform_stamped.transform.translation.z;
  pose.orientation = transform_stamped.transform.rotation;
  return pose;
}

}  // namespace kirin_utils