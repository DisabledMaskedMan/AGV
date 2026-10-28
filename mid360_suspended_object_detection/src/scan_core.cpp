#include "scan_core.hpp"

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <unordered_map>
#include <tuple>
#include <filesystem>

namespace scan_web {
namespace {
double number(const json::value & value) {
  if (!value.is_number()) throw std::invalid_argument("字段必须是有限数值");
  const auto result = value.to_number<double>();
  if (!std::isfinite(result)) throw std::invalid_argument("字段必须是有限数值");
  return result;
}
std::string normalized(std::string frame) {
  const auto first = frame.find_first_not_of('/');
  return first == std::string::npos ? "" : frame.substr(first);
}
bool text_is(const json::object & object, const char * key, const char * expected) {
  const auto found = object.if_contains(key);
  return found && found->is_string() && found->as_string() == expected;
}
double seconds(Clock::time_point newer, Clock::time_point older) {
  return std::max(0.0, std::chrono::duration<double>(newer - older).count());
}
double rate(const std::deque<Clock::time_point> & events, Clock::time_point now) {
  auto first = std::find_if(events.begin(), events.end(), [now](auto event) {return seconds(now, event) <= 5;});
  const auto count = std::distance(first, events.end());
  return count < 2 ? 0 : static_cast<double>(count - 1) / std::max(0.001, seconds(now, *first));
}
struct Cell {
  std::int64_t column, row;
  bool operator==(const Cell & other) const {return column == other.column && row == other.row;}
};
struct CellHash {
  std::size_t operator()(const Cell & cell) const {
    return std::hash<std::int64_t>{}(cell.column) ^ (std::hash<std::int64_t>{}(cell.row) << 1);
  }
};
}

void Config::validate() const {
  if (!(min_x < max_x && min_y < max_y && cluster_distance > 0 && cloud_timeout > 0 &&
        min_cluster_points >= 1 && max_cluster_points >= min_cluster_points &&
        min_width >= 0 && max_width >= min_width && min_depth >= 0 && max_depth >= min_depth)) {
    throw std::invalid_argument("ROI、聚类、尺寸或超时参数范围无效");
  }
  for (const auto & field : object()) number(field.value());
}

json::object Config::object() const {
  return {{"min_x", min_x}, {"max_x", max_x}, {"min_y", min_y}, {"max_y", max_y},
    {"cluster_distance", cluster_distance}, {"min_cluster_points", min_cluster_points},
    {"max_cluster_points", max_cluster_points}, {"min_width", min_width}, {"max_width", max_width},
    {"min_depth", min_depth}, {"max_depth", max_depth}, {"cloud_timeout", cloud_timeout}};
}

Config Config::updated(const json::value & changes) const {
  if (!changes.is_object()) throw std::invalid_argument("参数必须是 JSON 对象");
  auto merged = object();
  for (const auto & field : changes.as_object()) {
    if (!merged.contains(field.key())) throw std::invalid_argument("未知参数: " + std::string(field.key()));
    number(field.value());
    if (field.key() == "min_cluster_points" || field.key() == "max_cluster_points") {
      if ((!field.value().is_int64() && !field.value().is_uint64()) ||
          number(field.value()) > std::numeric_limits<int>::max() || number(field.value()) < 1) {
        throw std::invalid_argument("聚类点数必须是有效正整数");
      }
    }
    merged[field.key()] = field.value();
  }
  Config result;
  result.min_x = number(merged.at("min_x")); result.max_x = number(merged.at("max_x"));
  result.min_y = number(merged.at("min_y")); result.max_y = number(merged.at("max_y"));
  result.cluster_distance = number(merged.at("cluster_distance"));
  result.min_cluster_points = static_cast<int>(number(merged.at("min_cluster_points")));
  result.max_cluster_points = static_cast<int>(number(merged.at("max_cluster_points")));
  result.min_width = number(merged.at("min_width")); result.max_width = number(merged.at("max_width"));
  result.min_depth = number(merged.at("min_depth")); result.max_depth = number(merged.at("max_depth"));
  result.cloud_timeout = number(merged.at("cloud_timeout"));
  result.validate();
  return result;
}

Config Config::load(const std::string & path) {
  const auto root = YAML::LoadFile(path)["mid360_suspended_object_detection"]["ros__parameters"];
  if (!root.IsMap()) throw std::invalid_argument("配置缺少 ros__parameters");
  Config initial;
  json::object changes;
  for (const auto & field : initial.object()) {
    const auto name = std::string(field.key());
    if (!root[name]) continue;
    if (name == "min_cluster_points" || name == "max_cluster_points") changes[name] = root[name].as<int>();
    else changes[name] = root[name].as<double>();
  }
  return initial.updated(changes);
}

bool Packet::operator==(const Packet & other) const {
  return frame == other.frame && name == other.name && ranges == other.ranges &&
    angle_start == other.angle_start && increment == other.increment &&
    range_min == other.range_min && range_max == other.range_max && offset == other.offset;
}

std::optional<Packet> parse_packet(const json::value & response, const std::string & frame) {
  if (!response.is_object()) return std::nullopt;
  const auto & envelope = response.as_object();
  if (!text_is(envelope, "channel", "ros") || !text_is(envelope, "op", "publish") ||
      !text_is(envelope, "topic", "/scan")) return std::nullopt;
  const auto message = envelope.if_contains("msg");
  if (!message || !message->is_array()) return std::nullopt;
  for (const auto & value : message->as_array()) {
    if (!value.is_object()) continue;
    const auto & item = value.as_object();
    const auto identifier = item.if_contains("rame_id");
    if (!identifier || !identifier->is_string() || normalized(std::string(identifier->as_string())) != normalized(frame)) continue;
    Packet packet;
    packet.frame = std::string(identifier->as_string());
    if (const auto name = item.if_contains("name"); name && name->is_string()) packet.name = std::string(name->as_string());
    const auto timeout = item.if_contains("timeoutSec");
    if (timeout && number(*timeout) != 0) throw std::invalid_argument("雷达 timeoutSec 异常或报告超时");
    packet.angle_start = number(item.at("angle_start"));
    packet.increment = number(item.at("increment"));
    packet.range_min = number(item.at("range_min")); packet.range_max = number(item.at("range_max"));
    const auto & ranges = item.at("scan");
    if (packet.increment == 0 || packet.range_min < 0 || packet.range_max <= packet.range_min ||
        !ranges.is_array() || ranges.as_array().empty()) throw std::invalid_argument("扫描角度、量程或数组无效");
    if (ranges.as_array().size() > 2000000 || !std::isfinite(packet.angle_start +
        (ranges.as_array().size() - 1) * packet.increment)) throw std::invalid_argument("扫描过大或角度溢出");
    packet.ranges.reserve(ranges.as_array().size());
    for (const auto & sample : ranges.as_array()) {
      double distance = sample.is_number() ? sample.to_number<double>() * 0.001 : 0;
      if (!std::isfinite(distance) || distance <= 0 || distance < packet.range_min || distance > packet.range_max) distance = 0;
      packet.ranges.push_back(distance);
    }
    if (const auto offset = item.if_contains("laserOffset"); offset && !offset->is_null()) {
      if (!offset->is_array() || offset->as_array().size() != 3) throw std::invalid_argument("laserOffset 必须为 [x,y,yaw]");
      for (const auto & coordinate : offset->as_array()) number(coordinate);
      packet.offset = *offset;
    }
    return packet;
  }
  return std::nullopt;
}

std::vector<Point> project(const Packet & packet, std::vector<Point> & directions,
                           double & cached_start, double & cached_increment) {
  if (directions.size() != packet.ranges.size() || cached_start != packet.angle_start || cached_increment != packet.increment) {
    directions.resize(packet.ranges.size());
    cached_start = packet.angle_start; cached_increment = packet.increment;
    for (std::size_t index = 0; index < directions.size(); ++index) {
      const auto angle = packet.angle_start + index * packet.increment;
      directions[index] = {std::cos(angle), std::sin(angle)};
    }
  }
  std::vector<Point> points;
  points.reserve(packet.ranges.size());
  for (std::size_t index = 0; index < packet.ranges.size(); ++index) {
    if (packet.ranges[index] > 0) points.emplace_back(packet.ranges[index] * directions[index].first,
                                                   packet.ranges[index] * directions[index].second);
  }
  return points;
}

json::object Obstacle::object() const {
  const auto midpoint = center();
  return {{"min_x", min_x}, {"max_x", max_x}, {"min_y", min_y}, {"max_y", max_y},
    {"width", width()}, {"depth", depth()}, {"count", count}, {"distance", distance},
    {"center_x", midpoint.first}, {"center_y", midpoint.second}};
}

std::optional<Obstacle> Detection::largest() const {
  if (objects.empty()) return std::nullopt;
  return *std::min_element(objects.begin(), objects.end(), [](const auto & a, const auto & b) {
    return std::make_tuple(-a.area(), a.distance, a.center(), a.min_x, a.max_x, a.min_y, a.max_y, a.count) <
           std::make_tuple(-b.area(), b.distance, b.center(), b.min_x, b.max_x, b.min_y, b.max_y, b.count);
  });
}

json::object Detection::object() const {
  json::array boxes;
  for (const auto & box : objects) boxes.push_back(box.object());
  return {{"roi_count", roi_count}, {"objects", std::move(boxes)}};
}

json::object detect(const std::vector<Point> & points, const Config & config) {
  return detect_objects(points, config).object();
}

Detection detect_objects(const std::vector<Point> & points, const Config & config) {
  std::vector<Point> roi;
  roi.reserve(points.size());
  for (const auto & point : points) {
    if (point.first >= config.min_x && point.first <= config.max_x &&
        point.second >= config.min_y && point.second <= config.max_y) roi.push_back(point);
  }
  std::vector<std::size_t> parents(roi.size());
  std::iota(parents.begin(), parents.end(), 0);
  std::vector<unsigned char> ranks(roi.size(), 0);
  const auto root = [&parents](std::size_t index) {
    while (parents[index] != index) {parents[index] = parents[parents[index]]; index = parents[index];}
    return index;
  };
  std::unordered_map<Cell, std::vector<std::size_t>, CellHash> grid;
  const auto squared = config.cluster_distance * config.cluster_distance;
  const auto cell_index = [&config](double coordinate) {
    const auto value = std::floor(coordinate / config.cluster_distance);
    if (!std::isfinite(value) || std::abs(value) > 9e18) throw std::invalid_argument("聚类距离过小");
    return static_cast<std::int64_t>(value);
  };
  for (std::size_t index = 0; index < roi.size(); ++index) {
    const Cell cell{cell_index(roi[index].first), cell_index(roi[index].second)};
    for (int delta_x = -1; delta_x <= 1; ++delta_x) for (int delta_y = -1; delta_y <= 1; ++delta_y) {
      const auto found = grid.find({cell.column + delta_x, cell.row + delta_y});
      if (found == grid.end()) continue;
      for (const auto neighbor : found->second) {
        const auto difference_x = roi[index].first - roi[neighbor].first;
        const auto difference_y = roi[index].second - roi[neighbor].second;
        if (difference_x * difference_x + difference_y * difference_y > squared) continue;
        auto first = root(index), second = root(neighbor);
        if (first == second) continue;
        if (ranks[first] < ranks[second]) std::swap(first, second);
        parents[second] = first;
        if (ranks[first] == ranks[second]) ++ranks[first];
      }
    }
    grid[cell].push_back(index);
  }
  std::unordered_map<std::size_t, Obstacle> boxes;
  for (std::size_t index = 0; index < roi.size(); ++index) {
    auto & box = boxes[root(index)];
    const auto & point = roi[index];
    if (box.count++ == 0) box = {point.first, point.first, point.second, point.second, point, std::hypot(point.first, point.second), 1};
    else {
      box.min_x = std::min(box.min_x, point.first); box.max_x = std::max(box.max_x, point.first);
      box.min_y = std::min(box.min_y, point.second); box.max_y = std::max(box.max_y, point.second);
      const auto distance = std::hypot(point.first, point.second);
      if (distance < box.distance || (distance == box.distance && point < box.nearest)) {
        box.distance = distance; box.nearest = point;
      }
    }
  }
  Detection result;
  result.roi_count = roi.size();
  for (const auto & entry : boxes) {
    const auto & box = entry.second;
    const auto width = box.max_y - box.min_y, depth = box.max_x - box.min_x;
    if (box.count < static_cast<std::size_t>(config.min_cluster_points) || box.count > static_cast<std::size_t>(config.max_cluster_points) ||
        width < config.min_width || width > config.max_width || depth < config.min_depth || depth > config.max_depth) continue;
    result.objects.push_back(box);
  }
  std::sort(result.objects.begin(), result.objects.end(), [](const auto & first, const auto & second) {
    return std::make_tuple(first.distance, first.center()) < std::make_tuple(second.distance, second.center());
  });
  return result;
}

Service::Service(Config config, std::string source, std::string target, std::string config_path)
: url(std::move(source)), frame(std::move(target)), config_(config),
  config_path_(config_path.empty() ? "" : std::filesystem::canonical(config_path).string()) {config_.validate();}
Service::~Service() {stop();}
void Service::start() {
  detector_ = std::thread([this] {
    while (!stopping_) {
      const auto started = Clock::now();
      try {process();} catch (const std::exception & error) {fail(error.what());}
      std::this_thread::sleep_until(started + std::chrono::milliseconds(100));
    }
  });
}
void Service::stop() {stopping_ = true; if (detector_.joinable()) detector_.join();}
Config Service::config() const {std::lock_guard<std::mutex> guard(mutex_); return config_;}
json::object Service::update(const json::value & changes, bool persist) {
  std::lock_guard<std::mutex> guard(mutex_);
  const auto next = config_.updated(changes);
  if (persist) {
    if (config_path_.empty()) throw std::runtime_error("未配置 YAML 保存路径");
    next.save(config_path_);
  }
  config_ = next; ++config_version_; analysis_.reset();
  return config_.object();
}
void Service::accept(Packet packet) {
  const auto now = Clock::now();
  std::lock_guard<std::mutex> guard(mutex_);
  if (!packet_ || !(packet == *packet_)) {
    packet_ = std::make_shared<Packet>(std::move(packet));
    ++content_; changed_at_ = now; changes_.push_back(now);
  }
  received_at_ = now; ++received_; receptions_.push_back(now); connection_ = "streaming"; error_.clear();
  while (!receptions_.empty() && seconds(now, receptions_.front()) > 5) receptions_.pop_front();
  while (!changes_.empty() && seconds(now, changes_.front()) > 5) changes_.pop_front();
}
void Service::fail(const std::string & error, const std::string & connection) {
  std::lock_guard<std::mutex> guard(mutex_);
  error_ = error; connection_ = connection; packet_.reset(); analysis_.reset(); ++generation_;
}
void Service::connection(const std::string & value, bool reconnect) {
  std::lock_guard<std::mutex> guard(mutex_); connection_ = value; if (reconnect) ++reconnects_;
}
void Service::process() {
  std::shared_ptr<const Packet> packet;
  Config current;
  std::uint64_t content, config_version, generation;
  {
    std::lock_guard<std::mutex> guard(mutex_);
    if (!packet_ || seconds(Clock::now(), *received_at_) > config_.cloud_timeout ||
        (analysis_ && analysis_->content == content_ && analysis_->config == config_version_)) return;
    packet = packet_; current = config_; content = content_; config_version = config_version_; generation = generation_;
  }
  const auto started = Clock::now();
  const auto points = project(*packet, directions_, cached_start_, cached_increment_);
  const auto detection = detect_objects(points, current);
  const auto detected = !detection.objects.empty();
  const auto processing_ms = seconds(Clock::now(), started) * 1000;
  auto result = detection.object();
  json::array coordinates;
  coordinates.reserve(points.size());
  for (const auto & point : points) coordinates.push_back(json::array{point.first, point.second});
  result["points"] = std::move(coordinates); result["valid_count"] = points.size();
  auto fragment = json::serialize(result);
  fragment.erase(fragment.begin()); fragment.pop_back();
  const auto completed = Clock::now();
  auto analysis = std::make_shared<Analysis>(Analysis{content, config_version, std::move(fragment), detected,
    detection.objects.size(), detection.largest(), processing_ms, completed});
  std::lock_guard<std::mutex> guard(mutex_);
  if (packet_ && generation == generation_ && config_version == config_version_) {analysis_ = std::move(analysis); ++processed_;}
}
ObstacleSnapshot Service::snapshot_locked(Clock::time_point now) const {
  ObstacleSnapshot result;
  result.frame = normalized(packet_ ? packet_->frame : frame);
  result.status = connection_; result.error = error_;
  if (received_at_) result.receive_age_s = seconds(now, *received_at_);
  if (analysis_) result.result_age_s = seconds(now, analysis_->completed);
  bool usable = analysis_ && analysis_->config == config_version_;
  if (usable && analysis_->content != content_ && result.result_age_s > config_.cloud_timeout) usable = false;
  if (packet_) {
    if (result.receive_age_s > config_.cloud_timeout) {
      result.status = "timeout"; result.error = "目标扫描接收超时"; usable = false;
    } else result.status = !usable ? "processing" : analysis_->detected ? "detected" : "clear";
  } else usable = false;
  result.data_valid = usable;
  if (usable) {result.candidate_count = analysis_->candidate_count; result.largest = analysis_->largest;}
  return result;
}

ObstacleSnapshot Service::obstacle_snapshot() const {
  std::lock_guard<std::mutex> guard(mutex_);
  return snapshot_locked(Clock::now());
}

std::string Service::state_text() const {
  const auto now = Clock::now();
  json::object state;
  std::shared_ptr<const Analysis> analysis;
  {
    std::lock_guard<std::mutex> guard(mutex_);
    const auto snapshot = snapshot_locked(now);
    const auto & status = snapshot.status;
    const auto & error = snapshot.error;
    analysis = snapshot.data_valid ? analysis_ : nullptr;
    state = {{"status", status}, {"connection", connection_}, {"error", error.empty() ? json::value(nullptr) : json::value(error)},
      {"source_url", url}, {"target_frame", frame}, {"frame", packet_ ? json::value(packet_->frame) : json::value(nullptr)},
      {"name", packet_ ? json::value(packet_->name) : json::value(nullptr)}, {"source_timestamp", nullptr},
      {"distance_unit", "m"}, {"scan_unit", "mm"}, {"laser_offset", packet_ ? packet_->offset : json::value(nullptr)},
      {"scan_count", packet_ ? packet_->ranges.size() : 0}, {"detected", status == "detected"},
      {"largest_obstacle", snapshot.largest ? json::value(snapshot.largest->object()) : json::value(nullptr)},
      {"receive_age_s", received_at_ ? json::value(seconds(now, *received_at_)) : json::value(nullptr)},
      {"content_age_s", changed_at_ ? json::value(seconds(now, *changed_at_)) : json::value(nullptr)},
      {"receive_hz", rate(receptions_, now)}, {"content_hz", rate(changes_, now)}, {"received_count", received_},
      {"content_version", content_}, {"analyzed_content_version", analysis ? json::value(analysis->content) : json::value(nullptr)},
      {"processed_count", processed_}, {"processing_ms", analysis ? analysis->processing_ms : 0},
      {"processed_age_s", analysis ? json::value(seconds(now, analysis->completed)) : json::value(nullptr)},
      {"reconnects", reconnects_}, {"config", config_.object()}, {"backend", "cpp"}};
  }
  state["ros_publication"] = publication.object();
  auto text = json::serialize(state);
  text.pop_back(); text += ',';
  text += analysis ? analysis->fragment : "\"points\":[],\"valid_count\":0,\"roi_count\":0,\"objects\":[]";
  return text + '}';
}
}
