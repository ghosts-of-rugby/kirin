#ifndef SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_UTILS
#define SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_UTILS

#include <rclcpp/logger.hpp>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose.hpp>
#include <optional>
#include <string>
#include <memory>

namespace kirin_utils {

std::optional<geometry_msgs::msg::Pose> GetPoseFromTf(const rclcpp::Logger& node_logger,
                                                      std::shared_ptr<tf2_ros::Buffer> buffer_ptr,
                                                      const std::string& parent_name,
                                                      const std::string& child_name);

}

#endif /* SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_UTILS */
