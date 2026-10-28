#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <filesystem>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "cluster_detection.hpp"
#include "detector_config.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "interactive_markers/menu_handler.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "mid360_obstacle_avoidance/msg/obstacle_status.hpp"
#include "mid360_obstacle_avoidance/msg/roi_state.hpp"
#include "mid360_obstacle_avoidance/msg/detector_config.hpp"
#include "mid360_obstacle_avoidance/srv/apply_detector_config.hpp"
#include "mid360_obstacle_avoidance/srv/set_roi_boundary.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "roi_yaml.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"

class Mid360ObstacleNode : public rclcpp::Node
{
public:
  Mid360ObstacleNode() : Node("mid360_obstacle_avoidance")
  {
    input_topic_ = parameter<std::string>("input_topic", "/livox/lidar");
    expected_frame_ = parameter<std::string>("expected_frame", "livox_frame");
    min_x_ = parameter("roi_min_x", 0.2);
    max_x_ = parameter("roi_max_x", 1.7);
    min_y_ = parameter("roi_min_y", -0.4);
    max_y_ = parameter("roi_max_y", 0.8);
    min_z_ = parameter("roi_min_z", -0.1);
    max_z_ = parameter("roi_max_z", 0.5);
    startup_roi_ = current_roi();
    cluster_config_.distance = parameter("cluster_distance_threshold", 0.05);
    min_points_ = parameter<int64_t>("min_obstacle_points", 10);
    cluster_config_.min_diagonal = parameter("min_cluster_diagonal", 0.1);
    cluster_config_.min_volume = parameter("min_cluster_volume", 0.0);
    timeout_ = parameter("cloud_timeout", 0.5);
    detection_enabled_ = parameter("detection_enabled_on_start", false);
    enable_roi_editor_ = parameter("enable_roi_editor", false);
    enable_web_editor_ = parameter("enable_web_editor", false);
    roi_config_file_ = parameter<std::string>("roi_config_file", "");
    validate_parameters();
    cluster_config_.min_points = static_cast<std::size_t>(min_points_);
    enabled_since_ = last_cloud_ = Clock::now();

    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    status_pub_ = create_publisher<Status>("~/status", qos);
    stop_pub_ = create_publisher<std_msgs::msg::Bool>("~/stop_required", qos);
    detection_enabled_pub_ = create_publisher<std_msgs::msg::Bool>("~/detection_enabled", qos);
    detection_control_sub_ = create_subscription<std_msgs::msg::Bool>(
      "~/enable_detection", rclcpp::QoS(1).reliable().durability_volatile(),
      [this](std_msgs::msg::Bool::ConstSharedPtr command) {
        set_detection_enabled(command->data);
      });
    status_.header.frame_id = expected_frame_;
    invalidate();
    publish();

    if (editor_enabled()) {
      pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/pointcloud", rclcpp::SensorDataQoS().keep_last(1));
      roi_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        "~/roi_marker", qos);
      roi_state_pub_ = create_publisher<RoiState>("~/roi_state", qos);
      detector_config_pub_ = create_publisher<DetectorConfig>("~/detector_config", qos);
      apply_config_service_ = create_service<ApplyDetectorConfig>("~/apply_detector_config",
        [this](const std::shared_ptr<ApplyDetectorConfig::Request> request,
          std::shared_ptr<ApplyDetectorConfig::Response> response) {
          response->success = apply_detector_config(*request, response->message);
          response->config = config_message();
        });
      set_roi_service_ = create_service<SetRoiBoundary>("~/set_roi_boundary",
        [this](const std::shared_ptr<SetRoiBoundary::Request> request,
          std::shared_ptr<SetRoiBoundary::Response> response) {
          response->success = update_boundary(request->boundary, request->value, response->message);
          response->roi = roi_message();
        });
      save_roi_service_ = create_service<std_srvs::srv::Trigger>("~/save_roi",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          response->success = save_roi(response->message);
        });
      reset_roi_service_ = create_service<std_srvs::srv::Trigger>("~/reset_roi",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          response->success = reset_roi(response->message);
        });
      if (enable_roi_editor_) {
        interactive_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
          std::string(get_fully_qualified_name()) + "/roi_editor", this);
        menu_handler_.insert("Save ROI to YAML", std::bind(
            &Mid360ObstacleNode::save_menu_callback, this, std::placeholders::_1));
        menu_handler_.insert("Reload ROI from startup values", std::bind(
            &Mid360ObstacleNode::reload_menu_callback, this, std::placeholders::_1));
      }
      refresh_editor_markers();
    }

    subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input_topic_, rclcpp::SensorDataQoS().keep_last(1),
      [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr cloud) {process(*cloud);});
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      publish();
    });
    RCLCPP_INFO(get_logger(),
      "Listening to %s (CustomMsg); cluster distance %.3f m, min diagonal %.3f m%s",
      input_topic_.c_str(), cluster_config_.distance, cluster_config_.min_diagonal,
      enable_roi_editor_ ? "; ROI editor enabled" : "");
    RCLCPP_INFO(get_logger(), "Detection %s; control via %s/enable_detection (Bool)",
      detection_enabled_ ? "enabled" : "paused", get_fully_qualified_name());
  }

private:
  using RoiState = mid360_obstacle_avoidance::msg::RoiState;
  using SetRoiBoundary = mid360_obstacle_avoidance::srv::SetRoiBoundary;
  using DetectorConfig = mid360_obstacle_avoidance::msg::DetectorConfig;
  using ApplyDetectorConfig = mid360_obstacle_avoidance::srv::ApplyDetectorConfig;
  using Settings = mid360_obstacle_avoidance::DetectorSettings;
  bool editor_enabled() const {return enable_roi_editor_ || enable_web_editor_;}

  using Clock = std::chrono::steady_clock;
  using Status = mid360_obstacle_avoidance::msg::ObstacleStatus;
  using Feedback = visualization_msgs::msg::InteractiveMarkerFeedback;
  using RoiBounds = mid360_obstacle_avoidance::RoiBounds;

  template<typename T>
  T parameter(const std::string & name, const T & value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, value, descriptor);
  }

  mid360_obstacle_avoidance::RoiBounds current_roi() const
  {
    return {min_x_, max_x_, min_y_, max_y_, min_z_, max_z_};
  }

  mid360_obstacle_avoidance::RoiBounds locked_roi() const
  {
    std::lock_guard<std::mutex> lock(roi_mutex_);
    return current_roi();
  }

  Settings locked_settings() const
  {
    std::lock_guard<std::mutex> lock(roi_mutex_);
    return {current_roi(), cluster_config_, timeout_};
  }

  bool cloud_timed_out() const
  {
    const auto reference = detection_enabled_ ? std::max(last_cloud_, enabled_since_) : last_cloud_;
    return std::chrono::duration<double>(Clock::now() - reference).count() >=
      locked_settings().timeout;
  }

  void validate_parameters() const
  {
    cluster_config_.validate();
    std::string roi_error;
    if (!mid360_obstacle_avoidance::validate_roi_bounds(current_roi(), roi_error)) {
      throw std::invalid_argument("Invalid configuration: " + roi_error);
    }
    for (const auto value : {min_x_, max_x_, min_y_, max_y_, min_z_, max_z_, timeout_}) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument("Invalid configuration: ROI and timeout must be finite");
      }
    }
    if (input_topic_.empty() || expected_frame_.empty() || min_x_ < 0.0 ||
      timeout_ <= 0.0 ||
      min_points_ < 1 || min_points_ > std::numeric_limits<uint32_t>::max())
    {
      throw std::invalid_argument(
              "Invalid configuration: require nonempty topic/frame, ordered ROI with "
              "at least 0.01 m per axis, timeout > 0 and 1 <= min_obstacle_points <= UINT32_MAX");
    }
    if (editor_enabled() &&
      (roi_config_file_.empty() || !std::filesystem::u8path(roi_config_file_).is_absolute()))
    {
      throw std::invalid_argument(
              "Invalid configuration: an absolute roi_config_file is required for the ROI editor");
    }
    for (const auto bound : {min_x_, max_x_, min_y_, max_y_, min_z_, max_z_}) {
      if (std::abs(bound) > std::numeric_limits<float>::max()) {
        throw std::invalid_argument("Invalid configuration: ROI must fit float32 coordinates");
      }
      mid360_obstacle_avoidance::detail::cell_coordinate(
        static_cast<float>(bound), cluster_config_.distance);
    }
  }

  void clear_detection_result()
  {
    status_.obstacle_detected = false;
    status_.stop_required = false;
    status_.nearest_distance = std::numeric_limits<float>::quiet_NaN();
    status_.obstacle_points = 0;
    status_.stop_zone_points = 0;
  }

  void invalidate()
  {
    status_.data_valid = false;
    clear_detection_result();
  }

  void set_detection_enabled(bool enabled)
  {
    if (enabled == detection_enabled_) {
      return;
    }
    detection_enabled_ = enabled;
    // Preserve input health, but never reuse a result from before the transition.
    // Start a new acquisition window only on false -> true, never on repeated true.
    if (enabled) {
      enabled_since_ = Clock::now();
    }
    clear_detection_result();
    publish();
    RCLCPP_INFO(get_logger(), "Detection %s", enabled ? "enabled; waiting for a fresh cloud" :
      "paused; no stop request");
  }

  void process(const livox_ros_driver2::msg::CustomMsg & cloud)
  {
    const auto processing_start = Clock::now();
    status_.header = cloud.header;
    invalidate();
    if (cloud.header.frame_id != expected_frame_ || cloud.points.empty() ||
      cloud.point_num != cloud.points.size())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Invalid cloud: check expected_frame, point_num and nonempty points; detection %s",
        detection_enabled_ ? "enabled; timeout monitoring active" : "paused");
      publish();
      return;
    }

    const auto settings = locked_settings();
    const auto & roi = settings.roi;
    bool has_valid_return = false;
    std::vector<mid360_obstacle_avoidance::RoiPoint> roi_points;
    std::vector<mid360_obstacle_avoidance::RoiPoint> visual_points;
    if (detection_enabled_) {
      roi_points.reserve(cloud.points.size());
    }
    if (editor_enabled()) {
      visual_points.reserve(cloud.points.size());
    }
    for (const auto & point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
        (point.x == 0.0F && point.y == 0.0F && point.z == 0.0F))
      {
        continue;
      }
      has_valid_return = true;
      const mid360_obstacle_avoidance::RoiPoint value{point.x, point.y, point.z};
      if (editor_enabled()) {
        visual_points.push_back(value);
      }
      if (!detection_enabled_) {
        continue;
      }
      // Match float32 precision of Livox coordinates at inclusive ROI boundaries.
      if (point.x < static_cast<float>(roi.min_x) || point.x > static_cast<float>(roi.max_x) ||
        point.y < static_cast<float>(roi.min_y) || point.y > static_cast<float>(roi.max_y) ||
        point.z < static_cast<float>(roi.min_z) || point.z > static_cast<float>(roi.max_z))
      {
        continue;
      }
      roi_points.push_back(value);
    }

    if (editor_enabled()) {
      publish_pointcloud(cloud.header, visual_points);
    }
    if (has_valid_return) {
      last_cloud_ = processing_start;
      has_valid_cloud_ = true;
      status_.data_valid = true;
      if (detection_enabled_) {
        const auto result = mid360_obstacle_avoidance::detect_clusters(roi_points, settings.cluster);
        status_.nearest_distance = result.nearest;
        status_.obstacle_points = static_cast<uint32_t>(result.accepted_points);
        status_.stop_zone_points = status_.obstacle_points;
        status_.obstacle_detected = result.accepted_points > 0;
        status_.stop_required = status_.obstacle_detected;
      }
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Cloud contains no finite, nonzero returns; detection %s",
        detection_enabled_ ? "enabled; timeout monitoring active" : "paused");
    }
    publish();
    RCLCPP_DEBUG(get_logger(), "cloud_processing_us=%.3f points=%zu editor=%d",
      std::chrono::duration<double, std::micro>(Clock::now() - processing_start).count(),
      cloud.points.size(), editor_enabled());
  }

  void publish_pointcloud(
    const std_msgs::msg::Header & header,
    const std::vector<mid360_obstacle_avoidance::RoiPoint> & points)
  {
    sensor_msgs::msg::PointCloud2 message;
    message.header = header;
    message.header.frame_id = expected_frame_;
    message.height = 1;
    message.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(message);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    if (points.empty()) {
      pointcloud_pub_->publish(message);
      return;
    }
    sensor_msgs::PointCloud2Iterator<float> x(message, "x");
    sensor_msgs::PointCloud2Iterator<float> y(message, "y");
    sensor_msgs::PointCloud2Iterator<float> z(message, "z");
    for (const auto & point : points) {
      *x = point.x;
      *y = point.y;
      *z = point.z;
      ++x;
      ++y;
      ++z;
    }
    pointcloud_pub_->publish(message);
  }

  visualization_msgs::msg::Marker make_roi_marker(const RoiBounds & roi) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = expected_frame_;
    marker.ns = "roi_editor";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = (roi.min_x + roi.max_x) / 2.0;
    marker.pose.position.y = (roi.min_y + roi.max_y) / 2.0;
    marker.pose.position.z = (roi.min_z + roi.max_z) / 2.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = roi.max_x - roi.min_x;
    marker.scale.y = roi.max_y - roi.min_y;
    marker.scale.z = roi.max_z - roi.min_z;
    marker.color.r = 0.1F;
    marker.color.g = 0.8F;
    marker.color.b = 1.0F;
    marker.color.a = 0.22F;
    return marker;
  }

  visualization_msgs::msg::InteractiveMarker make_menu_marker(const RoiBounds & roi) const
  {
    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = expected_frame_;
    marker.name = "roi_editor_menu";
    marker.description = "ROI menu";
    marker.scale = 0.25F;
    marker.pose.position.x = (roi.min_x + roi.max_x) / 2.0;
    marker.pose.position.y = (roi.min_y + roi.max_y) / 2.0;
    marker.pose.position.z = (roi.min_z + roi.max_z) / 2.0;
    marker.pose.orientation.w = 1.0;
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.name = "menu";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MENU;
    control.always_visible = true;
    visualization_msgs::msg::Marker sphere;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.pose.orientation.w = 1.0;
    sphere.scale.x = 0.06;
    sphere.scale.y = 0.06;
    sphere.scale.z = 0.06;
    sphere.color.r = 1.0F;
    sphere.color.g = 0.8F;
    sphere.color.b = 0.1F;
    sphere.color.a = 1.0F;
    control.markers.push_back(sphere);
    marker.controls.push_back(control);
    return marker;
  }

  visualization_msgs::msg::InteractiveMarker make_handle(
    const std::string & name, const geometry_msgs::msg::Point & position,
    bool x_axis, bool y_axis, bool z_axis) const
  {
    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = expected_frame_;
    marker.name = name;
    marker.description = name;
    marker.scale = 0.25F;
    marker.pose.position = position;
    marker.pose.orientation.w = 1.0;
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.name = "move";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
    control.always_visible = true;
    if (x_axis) {
      control.orientation.w = 1.0;
    } else if (y_axis) {
      control.orientation.z = 0.70710678;
      control.orientation.w = 0.70710678;
    } else {
      control.orientation.y = -0.70710678;
      control.orientation.w = 0.70710678;
    }
    visualization_msgs::msg::Marker sphere;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.pose.orientation.w = 1.0;
    sphere.scale.x = 0.08;
    sphere.scale.y = 0.08;
    sphere.scale.z = 0.08;
    sphere.color.r = x_axis ? 1.0F : 0.2F;
    sphere.color.g = y_axis ? 1.0F : 0.2F;
    sphere.color.b = z_axis ? 1.0F : 0.2F;
    sphere.color.a = 1.0F;
    control.markers.push_back(sphere);
    marker.controls.push_back(control);
    return marker;
  }

  void refresh_editor_markers()
  {
    const auto roi = locked_roi();
    roi_marker_pub_->publish(make_roi_marker(roi));
    roi_state_pub_->publish(roi_message());
    detector_config_pub_->publish(config_message());
    if (!interactive_server_) {return;}
    const auto center = make_menu_marker(roi);
    visualization_msgs::msg::InteractiveMarker existing;
    if (interactive_server_->get(center.name, existing)) {
      interactive_server_->setPose(center.name, center.pose, center.header);
    } else {
      interactive_server_->insert(center);
      menu_handler_.apply(*interactive_server_, center.name);
    }
    geometry_msgs::msg::Point point;
    point.x = roi.min_x;
    point.y = (roi.min_y + roi.max_y) / 2.0;
    point.z = (roi.min_z + roi.max_z) / 2.0;
    insert_handle(make_handle("roi_min_x", point, true, false, false));
    point.x = roi.max_x;
    insert_handle(make_handle("roi_max_x", point, true, false, false));
    point.x = (roi.min_x + roi.max_x) / 2.0;
    point.y = roi.min_y;
    insert_handle(make_handle("roi_min_y", point, false, true, false));
    point.y = roi.max_y;
    insert_handle(make_handle("roi_max_y", point, false, true, false));
    point.y = (roi.min_y + roi.max_y) / 2.0;
    point.z = roi.min_z;
    insert_handle(make_handle("roi_min_z", point, false, false, true));
    point.z = roi.max_z;
    insert_handle(make_handle("roi_max_z", point, false, false, true));
    interactive_server_->applyChanges();
  }

  void insert_handle(const visualization_msgs::msg::InteractiveMarker & marker)
  {
    visualization_msgs::msg::InteractiveMarker existing;
    if (interactive_server_->get(marker.name, existing)) {
      interactive_server_->setPose(marker.name, marker.pose, marker.header);
    } else {
      interactive_server_->insert(marker, std::bind(
          &Mid360ObstacleNode::marker_feedback, this, std::placeholders::_1));
    }
  }

  void marker_feedback(const Feedback::ConstSharedPtr feedback)
  {
    if (feedback->event_type != Feedback::POSE_UPDATE &&
      feedback->event_type != Feedback::MOUSE_UP)
    {
      return;
    }
    if (feedback->header.frame_id != expected_frame_) {
      RCLCPP_WARN(get_logger(), "Ignoring ROI feedback in an unexpected frame");
      refresh_editor_markers();
      return;
    }
    const auto & name = feedback->marker_name;
    const auto & position = feedback->pose.position;
    const double coordinate = name == "roi_min_x" || name == "roi_max_x" ? position.x :
      (name == "roi_min_y" || name == "roi_max_y" ? position.y : position.z);
    std::string error;
    if (!update_boundary(name, coordinate, error)) {
      RCLCPP_WARN(get_logger(), "Ignoring invalid ROI feedback: %s", error.c_str());
      refresh_editor_markers();
    }
  }

  RoiState roi_message() const
  {
    const auto settings = locked_settings();
    const auto & roi = settings.roi;
    RoiState message;
    message.header.frame_id = expected_frame_;
    message.header.stamp = now();
    message.min_x = roi.min_x; message.max_x = roi.max_x;
    message.min_y = roi.min_y; message.max_y = roi.max_y;
    message.min_z = roi.min_z; message.max_z = roi.max_z;
    message.cloud_timeout = settings.timeout;
    message.revision = roi_revision_;
    return message;
  }

  DetectorConfig config_message() const
  {
    const auto settings = locked_settings();
    DetectorConfig message;
    message.header.frame_id = expected_frame_;
    message.header.stamp = now();
    message.roi_min_x = settings.roi.min_x; message.roi_max_x = settings.roi.max_x;
    message.roi_min_y = settings.roi.min_y; message.roi_max_y = settings.roi.max_y;
    message.roi_min_z = settings.roi.min_z; message.roi_max_z = settings.roi.max_z;
    message.cluster_distance_threshold = settings.cluster.distance;
    message.min_obstacle_points = static_cast<uint32_t>(settings.cluster.min_points);
    message.min_cluster_diagonal = settings.cluster.min_diagonal;
    message.min_cluster_volume = settings.cluster.min_volume;
    message.cloud_timeout = settings.timeout;
    message.revision = roi_revision_;
    return message;
  }

  bool apply_detector_config(const ApplyDetectorConfig::Request & request, std::string & message)
  {
    // All callbacks use the default mutually exclusive group. Persistence and
    // commit are serialized with ROI edits; frames take one settings snapshot.
    auto candidate = locked_settings();
    candidate.cluster.distance = request.cluster_distance_threshold;
    candidate.cluster.min_points = request.min_obstacle_points;
    candidate.cluster.min_diagonal = request.min_cluster_diagonal;
    candidate.cluster.min_volume = request.min_cluster_volume;
    candidate.timeout = request.cloud_timeout;
    try {
      candidate.validate();
      auto values = mid360_obstacle_avoidance::roi_yaml_values(candidate.roi);
      values.emplace_back("cluster_distance_threshold",
        mid360_obstacle_avoidance::format_yaml_number(candidate.cluster.distance));
      values.emplace_back("min_obstacle_points", std::to_string(candidate.cluster.min_points));
      values.emplace_back("min_cluster_diagonal",
        mid360_obstacle_avoidance::format_yaml_number(candidate.cluster.min_diagonal));
      values.emplace_back("min_cluster_volume",
        mid360_obstacle_avoidance::format_yaml_number(candidate.cluster.min_volume));
      values.emplace_back("cloud_timeout",
        mid360_obstacle_avoidance::format_yaml_number(candidate.timeout));
      if (!mid360_obstacle_avoidance::save_numeric_to_yaml(
          roi_config_file_, values, message, get_fully_qualified_name())) {
        RCLCPP_ERROR(get_logger(), "Cannot apply detector configuration: %s", message.c_str());
        return false;
      }
    } catch (const std::exception & exception) {
      message = exception.what();
      return false;
    }
    bool changed;
    {
      std::lock_guard<std::mutex> lock(roi_mutex_);
      changed = cluster_config_.distance != candidate.cluster.distance ||
        cluster_config_.min_points != candidate.cluster.min_points ||
        cluster_config_.min_diagonal != candidate.cluster.min_diagonal ||
        cluster_config_.min_volume != candidate.cluster.min_volume || timeout_ != candidate.timeout;
      cluster_config_ = candidate.cluster;
      timeout_ = candidate.timeout;
      min_points_ = static_cast<int64_t>(candidate.cluster.min_points);
      if (changed) {++roi_revision_;}
    }
    if (changed) {clear_detection_result();}
    publish();
    refresh_editor_markers();
    message = "Parameters and current ROI saved and applied: " + roi_config_file_;
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
    return true;
  }

  void commit_roi(const RoiBounds & candidate)
  {
    {
      std::lock_guard<std::mutex> lock(roi_mutex_);
      min_x_ = candidate.min_x; max_x_ = candidate.max_x;
      min_y_ = candidate.min_y; max_y_ = candidate.max_y;
      min_z_ = candidate.min_z; max_z_ = candidate.max_z;
      ++roi_revision_;
    }
    // Clear the old result without changing input health. Paused detection stays paused.
    clear_detection_result();
    publish();
    refresh_editor_markers();
  }

  bool update_boundary(const std::string & name, double coordinate, std::string & error)
  {
    auto settings = locked_settings();
    auto & candidate = settings.roi;
    if (!mid360_obstacle_avoidance::move_roi_boundary(candidate, name, coordinate, error)) {
      return false;
    }
    try {
      settings.validate();
    } catch (const std::exception & exception) {
      error = exception.what();
      return false;
    }
    commit_roi(candidate);
    error = "ROI updated in memory (not saved)";
    return true;
  }

  bool save_roi(std::string & message)
  {
    bool success = false;
    try {
      success = mid360_obstacle_avoidance::save_roi_to_yaml(
        roi_config_file_, locked_roi(), message, get_fully_qualified_name());
    } catch (const std::exception & exception) {
      message = exception.what();
    }
    if (success) {
      message = "Saved ROI to " + roi_config_file_;
      RCLCPP_INFO(get_logger(), "%s", message.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to save ROI: %s", message.c_str());
    }
    return success;
  }

  bool reset_roi(std::string & message)
  {
    auto candidate = locked_settings();
    candidate.roi = startup_roi_;
    try {
      candidate.validate();
    } catch (const std::exception & exception) {
      message = exception.what();
      return false;
    }
    commit_roi(startup_roi_);
    message = "Reloaded startup ROI values (not saved)";
    return true;
  }

  void save_menu_callback(const Feedback::ConstSharedPtr)
  {
    std::string message;
    save_roi(message);
  }

  void reload_menu_callback(const Feedback::ConstSharedPtr)
  {
    std::string message;
    if (reset_roi(message)) {
      RCLCPP_INFO(get_logger(), "%s", message.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Cannot reload startup ROI: %s", message.c_str());
    }
  }

  void publish()
  {
    const bool expired = cloud_timed_out();
    if (expired && (has_valid_cloud_ || detection_enabled_)) {
      invalidate();
    }
    status_.stop_required = detection_enabled_ && (status_.obstacle_detected || expired);
    status_pub_->publish(status_);
    std_msgs::msg::Bool stop;
    stop.data = status_.stop_required;
    stop_pub_->publish(stop);
    std_msgs::msg::Bool enabled;
    enabled.data = detection_enabled_;
    detection_enabled_pub_->publish(enabled);
  }

  std::string input_topic_, expected_frame_, roi_config_file_;
  double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_, timeout_;
  bool enable_roi_editor_, enable_web_editor_;
  bool detection_enabled_;
  uint64_t roi_revision_ = 0;
  rclcpp::Publisher<RoiState>::SharedPtr roi_state_pub_;
  rclcpp::Publisher<DetectorConfig>::SharedPtr detector_config_pub_;
  rclcpp::Service<ApplyDetectorConfig>::SharedPtr apply_config_service_;
  rclcpp::Service<SetRoiBoundary>::SharedPtr set_roi_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_roi_service_, reset_roi_service_;
  mid360_obstacle_avoidance::RoiBounds startup_roi_;
  mid360_obstacle_avoidance::ClusterConfig cluster_config_;
  int64_t min_points_;
  Status status_;
  Clock::time_point last_cloud_;
  Clock::time_point enabled_since_;
  bool has_valid_cloud_ = false;
  mutable std::mutex roi_mutex_;
  rclcpp::Publisher<Status>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detection_enabled_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr detection_control_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr roi_marker_pub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_server_;
  interactive_markers::MenuHandler menu_handler_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Mid360ObstacleNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("mid360_obstacle_avoidance"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
