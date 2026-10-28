#pragma once

#ifdef SCAN_WITH_ROS2

#include "mid360_suspended_object_detection/msg/largest_obstacle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "scan_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>

namespace scan_web {
class RosObstaclePublisher final : public rclcpp::Node {
public:
  explicit RosObstaclePublisher(Service & service)
  : Node("mid360_scan_web"), service_(service) {
    const bool enabled = declare_parameter<bool>("publish_obstacle", true);
    const auto topic = declare_parameter<std::string>(
      "obstacle_topic", "/mid360_suspended_object_detection/largest_obstacle");
    if (!enabled) {
      service_.publication.configure(false, get_node_topics_interface()->resolve_topic_name(topic));
      RCLCPP_INFO(get_logger(), "Obstacle ROS publishing is disabled");
      return;
    }
    publisher_ = create_publisher<mid360_suspended_object_detection::msg::LargestObstacle>(
      topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());
    service_.publication.configure(true, publisher_->get_topic_name());
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] {
      try {publish_snapshot();}
      catch (const std::exception & error) {
        service_.publication.failed(error.what());
        RCLCPP_ERROR(get_logger(), "Obstacle publication failed: %s", error.what());
      }
    });
    RCLCPP_INFO(get_logger(), "Publishing largest ROI obstacle on %s", publisher_->get_topic_name());
  }

private:
  static std::uint32_t bounded_count(std::size_t count) {
    return static_cast<std::uint32_t>(std::min<std::size_t>(count, std::numeric_limits<std::uint32_t>::max()));
  }

  void publish_snapshot() {
    using Message = mid360_suspended_object_detection::msg::LargestObstacle;
    const auto snapshot = service_.obstacle_snapshot();
    Message message;
    message.header.stamp = now();
    message.header.frame_id = snapshot.frame.empty() ? service_.frame : snapshot.frame;
    message.data_valid = snapshot.data_valid;
    message.status = snapshot.status;
    message.error = snapshot.error;
    message.receive_age_s = snapshot.receive_age_s;
    message.result_age_s = snapshot.result_age_s;
    message.detected = snapshot.data_valid && snapshot.largest.has_value();
    message.candidate_count = bounded_count(snapshot.data_valid ? snapshot.candidate_count : 0);
    if (message.detected) {
      const auto & obstacle = *snapshot.largest;
      const auto center = obstacle.center();
      message.point_count = bounded_count(obstacle.count);
      message.center.x = center.first;
      message.center.y = center.second;
      message.distance_x = center.first;
      message.distance_y = center.second;
      message.center_distance = std::hypot(center.first, center.second);
      message.nearest_point.x = obstacle.nearest.first;
      message.nearest_point.y = obstacle.nearest.second;
      message.distance = obstacle.distance;
      message.width = obstacle.width();
      message.depth = obstacle.depth();
      message.area = obstacle.area();
    }
    const auto point = [](const geometry_msgs::msg::Point & p) {
      return json::object{{"x", p.x}, {"y", p.y}, {"z", p.z}};
    };
    json::object payload{
      {"header", json::object{{"stamp", json::object{{"sec", message.header.stamp.sec}, {"nanosec", message.header.stamp.nanosec}}},
                              {"frame_id", message.header.frame_id}}},
      {"data_valid", message.data_valid}, {"detected", message.detected},
      {"status", message.status}, {"error", message.error},
      {"candidate_count", message.candidate_count}, {"point_count", message.point_count},
      {"center", point(message.center)}, {"nearest_point", point(message.nearest_point)},
      {"distance_x", message.distance_x}, {"distance_y", message.distance_y},
      {"center_distance", message.center_distance}, {"distance", message.distance},
      {"width", message.width}, {"depth", message.depth}, {"area", message.area},
      {"receive_age_s", message.receive_age_s}, {"result_age_s", message.result_age_s}};
    publisher_->publish(message);
    service_.publication.published(std::move(payload));
  }

  Service & service_;
  rclcpp::Publisher<mid360_suspended_object_detection::msg::LargestObstacle>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace scan_web

#endif
