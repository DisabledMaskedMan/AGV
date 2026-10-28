#pragma once

#include <boost/json.hpp>
#include <atomic>
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace scan_web {
namespace json = boost::json;
using Clock = std::chrono::steady_clock;
using Point = std::pair<double, double>;

struct Config {
  double min_x{0.3}, max_x{5}, min_y{-1}, max_y{1};
  double cluster_distance{0.12}, min_width{0.08}, max_width{2};
  double min_depth{0.08}, max_depth{3}, cloud_timeout{1};
  int min_cluster_points{8}, max_cluster_points{100000};
  void validate() const;
  json::object object() const;
  Config updated(const json::value & changes) const;
  static Config load(const std::string & path);
  void save(const std::string & path) const;
};

struct Packet {
  std::string frame, name;
  std::vector<double> ranges;
  double angle_start{0}, increment{0}, range_min{0}, range_max{0};
  json::value offset;
  bool operator==(const Packet & other) const;
};

struct Obstacle {
  double min_x{0}, max_x{0}, min_y{0}, max_y{0};
  Point nearest{0, 0};
  double distance{0};
  std::size_t count{0};
  double width() const {return max_y - min_y;}
  double depth() const {return max_x - min_x;}
  double area() const {return width() * depth();}
  Point center() const {return {(min_x + max_x) / 2, (min_y + max_y) / 2};}
  json::object object() const;
};

struct Detection {
  std::size_t roi_count{0};
  std::vector<Obstacle> objects;
  std::optional<Obstacle> largest() const;
  json::object object() const;
};

// Small immutable view shared by HTTP and ROS; never includes scan coordinates.
struct ObstacleSnapshot {
  std::string frame, status, error;
  bool data_valid{false};
  std::size_t candidate_count{0};
  std::optional<Obstacle> largest;
  double receive_age_s{-1}, result_age_s{-1};
};

// ROS-free telemetry so standalone builds can expose the same HTTP schema.
class PublicationTelemetry {
public:
  void configure(bool enabled, std::string topic);
  void published(json::object message, Clock::time_point now = Clock::now());
  void failed(std::string error);
  json::object object(Clock::time_point now = Clock::now()) const;
private:
  mutable std::mutex mutex_;
  bool supported_{false}, enabled_{false};
  std::string topic_, error_;
  std::uint64_t count_{0};
  std::optional<Clock::time_point> last_;
  std::deque<Clock::time_point> events_;
  json::value message_;
};

std::optional<Packet> parse_packet(const json::value & response, const std::string & frame);
std::vector<Point> project(const Packet & packet, std::vector<Point> & directions,
                           double & cached_start, double & cached_increment);
json::object detect(const std::vector<Point> & points, const Config & config);
Detection detect_objects(const std::vector<Point> & points, const Config & config);

class Service {
public:
  Service(Config config, std::string url, std::string frame, std::string config_path = "");
  ~Service();
  void start();
  void stop();
  void accept(Packet packet);
  void fail(const std::string & error, const std::string & connection = "invalid");
  void connection(const std::string & value, bool reconnect = false);
  Config config() const;
  json::object update(const json::value & changes, bool persist = false);
  std::string state_text() const;
  ObstacleSnapshot obstacle_snapshot() const;
  void process();
  const std::string url, frame;
  PublicationTelemetry publication;

private:
  mutable std::mutex mutex_;
  Config config_;
  std::string config_path_;
  std::shared_ptr<const Packet> packet_;
  struct Analysis {
    std::uint64_t content, config;
    std::string fragment;
    bool detected;
    std::size_t candidate_count;
    std::optional<Obstacle> largest;
    double processing_ms;
    Clock::time_point completed;
  };
  std::shared_ptr<const Analysis> analysis_;
  ObstacleSnapshot snapshot_locked(Clock::time_point now) const;
  std::uint64_t content_{0}, config_version_{0}, generation_{0}, received_{0}, processed_{0}, reconnects_{0};
  std::optional<Clock::time_point> received_at_, changed_at_;
  std::deque<Clock::time_point> receptions_, changes_;
  std::string connection_{"waiting"}, error_;
  std::atomic<bool> stopping_{false};
  std::thread detector_;
  std::vector<Point> directions_;
  double cached_start_{0}, cached_increment_{0};
};
}
