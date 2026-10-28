#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{
struct Point {float x; float y; float z;};
struct Cell {int x; int y; int z; bool operator==(const Cell & o) const {return x == o.x && y == o.y && z == o.z;}};
struct CellHash {
  std::size_t operator()(const Cell & c) const noexcept {
    const auto a = static_cast<std::uint64_t>(static_cast<std::uint32_t>(c.x));
    const auto b = static_cast<std::uint64_t>(static_cast<std::uint32_t>(c.y));
    const auto d = static_cast<std::uint64_t>(static_cast<std::uint32_t>(c.z));
    return static_cast<std::size_t>((a * 73856093ULL) ^ (b * 19349663ULL) ^ (d * 83492791ULL));
  }
};
struct Box {Point min; Point max; std::size_t count{0};};

class UnionFind {
public:
  explicit UnionFind(std::size_t n) : parent_(n), rank_(n, 0) {
    for (std::size_t i = 0; i < n; ++i) parent_[i] = i;
  }
  std::size_t find(std::size_t x) {
    if (parent_[x] != x) parent_[x] = find(parent_[x]);
    return parent_[x];
  }
  void join(std::size_t a, std::size_t b) {
    a = find(a); b = find(b); if (a == b) return;
    if (rank_[a] < rank_[b]) std::swap(a, b);
    parent_[b] = a; if (rank_[a] == rank_[b]) ++rank_[a];
  }
private:
  std::vector<std::size_t> parent_;
  std::vector<unsigned char> rank_;
};
}

class SuspendedObjectNode : public rclcpp::Node
{
public:
  SuspendedObjectNode() : Node("mid360_suspended_object_detection")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/livox/lidar");
    frame_ = declare_parameter<std::string>("expected_frame", "livox_frame");
    min_x_ = declare_parameter("min_x", 0.3); max_x_ = declare_parameter("max_x", 5.0);
    min_y_ = declare_parameter("min_y", -1.0); max_y_ = declare_parameter("max_y", 1.0);
    min_z_ = declare_parameter("min_z", 1.2); max_z_ = declare_parameter("max_z", 3.5);
    distance_ = declare_parameter("cluster_distance", 0.12);
    min_points_ = declare_parameter<int>("min_cluster_points", 8);
    max_points_ = declare_parameter<int>("max_cluster_points", 100000);
    min_width_ = declare_parameter("min_width", 0.08); max_width_ = declare_parameter("max_width", 2.0);
    min_height_ = declare_parameter("min_height", 0.08); max_height_ = declare_parameter("max_height", 2.5);
    min_depth_ = declare_parameter("min_depth", 0.08); max_depth_ = declare_parameter("max_depth", 3.0);
    timeout_ = declare_parameter("cloud_timeout", 1.0);
    validate();
    detected_pub_ = create_publisher<std_msgs::msg::Bool>("~/detected", rclcpp::QoS(10));
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/markers", rclcpp::QoS(10));
    subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input_topic_, rclcpp::SensorDataQoS().keep_last(1),
      [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) {process(*msg);});
    timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {
      if (seen_cloud_ && std::chrono::duration<double>(std::chrono::steady_clock::now() - last_cloud_).count() > timeout_) {
        publish(false, {}); seen_cloud_ = false;
        RCLCPP_WARN(get_logger(), "Point cloud timeout; clearing detection");
      }
    });
    RCLCPP_INFO(get_logger(), "Listening for MID-360 CustomMsg on %s; target ROI x[%.2f, %.2f] y[%.2f, %.2f] z[%.2f, %.2f]",
      input_topic_.c_str(), min_x_, max_x_, min_y_, max_y_, min_z_, max_z_);
  }

private:
  void validate() const
  {
    if (input_topic_.empty() || frame_.empty() || min_x_ < 0 || min_x_ >= max_x_ ||
      min_y_ >= max_y_ || min_z_ >= max_z_ || !std::isfinite(distance_) || distance_ <= 0 ||
      min_points_ < 1 || max_points_ < min_points_ || timeout_ <= 0) {
      throw std::invalid_argument("Invalid topic/frame, ROI, clustering or timeout parameters");
    }
  }
  void process(const livox_ros_driver2::msg::CustomMsg & msg)
  {
    last_cloud_ = std::chrono::steady_clock::now(); seen_cloud_ = true;
    if (msg.header.frame_id != frame_ || msg.point_num != msg.points.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid cloud frame or point_num");
      publish(false, {}); return;
    }
    std::vector<Point> points; points.reserve(msg.points.size());
    for (const auto & p : msg.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || (p.x == 0 && p.y == 0 && p.z == 0)) continue;
      if (p.x >= min_x_ && p.x <= max_x_ && p.y >= min_y_ && p.y <= max_y_ && p.z >= min_z_ && p.z <= max_z_) points.push_back({p.x,p.y,p.z});
    }
    if (points.empty()) {publish(false, {}); return;}
    UnionFind uf(points.size());
    std::unordered_map<Cell, std::vector<std::size_t>, CellHash> grid;
    const auto cell = [this](const Point & p) {return Cell{static_cast<int>(std::floor(p.x / distance_)), static_cast<int>(std::floor(p.y / distance_)), static_cast<int>(std::floor(p.z / distance_))};};
    const float d2 = static_cast<float>(distance_ * distance_);
    for (std::size_t i = 0; i < points.size(); ++i) {
      const Cell c = cell(points[i]);
      for (int dx=-1; dx<=1; ++dx) for (int dy=-1; dy<=1; ++dy) for (int dz=-1; dz<=1; ++dz) {
        const auto it = grid.find({c.x+dx,c.y+dy,c.z+dz}); if (it == grid.end()) continue;
        for (const auto j : it->second) {const float x=points[i].x-points[j].x, y=points[i].y-points[j].y, z=points[i].z-points[j].z; if (x*x+y*y+z*z <= d2) uf.join(i,j);}
      }
      grid[c].push_back(i);
    }
    std::unordered_map<std::size_t, Box> boxes;
    for (std::size_t i=0; i<points.size(); ++i) {
      auto & b=boxes[uf.find(i)]; const auto & p=points[i];
      if (b.count++ == 0) b.min=b.max=p;
      else {b.min.x=std::min(b.min.x,p.x); b.min.y=std::min(b.min.y,p.y); b.min.z=std::min(b.min.z,p.z); b.max.x=std::max(b.max.x,p.x); b.max.y=std::max(b.max.y,p.y); b.max.z=std::max(b.max.z,p.z);}
    }
    std::vector<Box> objects;
    for (const auto & entry : boxes) {const auto & b=entry.second; const float w=b.max.y-b.min.y, h=b.max.z-b.min.z, d=b.max.x-b.min.x; if (b.count >= static_cast<std::size_t>(min_points_) && b.count <= static_cast<std::size_t>(max_points_) && w>=min_width_ && w<=max_width_ && h>=min_height_ && h<=max_height_ && d>=min_depth_ && d<=max_depth_) objects.push_back(b);}
    publish(!objects.empty(), objects, msg.header);
  }
  void publish(bool detected, const std::vector<Box> & boxes, const std_msgs::msg::Header & header = std_msgs::msg::Header())
  {
    std_msgs::msg::Bool state; state.data=detected; detected_pub_->publish(state);
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear; clear.header=header; clear.ns="suspended_objects"; clear.id=0; clear.action=visualization_msgs::msg::Marker::DELETEALL; array.markers.push_back(clear);
    int id=1; for (const auto & b: boxes) {visualization_msgs::msg::Marker m; m.header=header; m.ns="suspended_objects"; m.id=id++; m.type=visualization_msgs::msg::Marker::CUBE; m.action=visualization_msgs::msg::Marker::ADD; m.pose.position.x=(b.min.x+b.max.x)*0.5; m.pose.position.y=(b.min.y+b.max.y)*0.5; m.pose.position.z=(b.min.z+b.max.z)*0.5; m.pose.orientation.w=1; m.scale.x=b.max.x-b.min.x; m.scale.y=b.max.y-b.min.y; m.scale.z=b.max.z-b.min.z; m.color.r=1.0; m.color.g=0.2; m.color.b=0.0; m.color.a=0.65; m.lifetime.sec=static_cast<int32_t>(timeout_); m.lifetime.nanosec=static_cast<uint32_t>((timeout_-m.lifetime.sec)*1e9); array.markers.push_back(m);}
    markers_pub_->publish(array);
  }
  std::string input_topic_, frame_;
  double min_x_,max_x_,min_y_,max_y_,min_z_,max_z_,distance_,min_width_,max_width_,min_height_,max_height_,min_depth_,max_depth_,timeout_;
  int min_points_,max_points_;
  bool seen_cloud_{false}; std::chrono::steady_clock::time_point last_cloud_{};
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detected_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<SuspendedObjectNode>());}
  catch (const std::exception & e) {RCLCPP_FATAL(rclcpp::get_logger("suspended_object_node"), "%s", e.what()); rclcpp::shutdown(); return 1;}
  rclcpp::shutdown(); return 0;
}
