#ifndef MID360_OBSTACLE_AVOIDANCE_CLUSTER_DETECTION_HPP_
#define MID360_OBSTACLE_AVOIDANCE_CLUSTER_DETECTION_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace mid360_obstacle_avoidance
{

struct RoiPoint
{
  float x;
  float y;
  float z;
};

struct ClusterConfig
{
  double distance = 0.10;
  std::size_t min_points = 3;
  double min_diagonal = 0.05;
  double min_volume = 0.0;

  void validate() const
  {
    if (!std::isfinite(distance) || distance <= 0.0 || min_points == 0 ||
      !std::isfinite(min_diagonal) || min_diagonal < 0.0 ||
      !std::isfinite(min_volume) || min_volume < 0.0)
    {
      throw std::invalid_argument(
              "Invalid configuration: finite cluster_distance_threshold > 0, "
              "min_obstacle_points >= 1 and finite nonnegative cluster size thresholds required");
    }
  }
};

struct ClusterResult
{
  std::size_t accepted_points = 0;
  float nearest = std::numeric_limits<float>::infinity();
};

namespace detail
{

struct Cell
{
  int64_t x;
  int64_t y;
  int64_t z;

  bool operator==(const Cell & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct CellHash
{
  std::size_t operator()(const Cell & cell) const noexcept
  {
    auto hash = std::hash<int64_t>{}(cell.x);
    for (const auto value : {cell.y, cell.z}) {
      hash ^= std::hash<int64_t>{}(value) + std::size_t{0x9e3779b9} +
        (hash << 6) + (hash >> 2);
    }
    return hash;
  }
};

// Leave headroom for neighbor offsets and avoid unsafe floating-to-integer casts.
inline int64_t cell_coordinate(double coordinate, double distance)
{
  const double index = std::floor(coordinate / distance);
  if (!std::isfinite(index) || std::abs(index) > 0x1p50) {
    throw std::invalid_argument(
            "Invalid configuration: ROI / cluster_distance_threshold exceeds grid range");
  }
  return static_cast<int64_t>(index);
}

inline Cell cell_for(const RoiPoint & point, double distance)
{
  return {cell_coordinate(point.x, distance), cell_coordinate(point.y, distance),
    cell_coordinate(point.z, distance)};
}

}  // namespace detail

// Input contains only finite, nonzero points already clipped to the ROI.
inline ClusterResult detect_clusters(
  const std::vector<RoiPoint> & points, const ClusterConfig & config)
{
  config.validate();
  std::unordered_map<detail::Cell, std::vector<std::size_t>, detail::CellHash> cells;
  cells.reserve(points.size());
  for (std::size_t index = 0; index < points.size(); ++index) {
    cells[detail::cell_for(points[index], config.distance)].push_back(index);
  }

  ClusterResult result;
  std::vector<bool> visited(points.size(), false);
  std::vector<std::size_t> cluster;
  for (std::size_t seed = 0; seed < points.size(); ++seed) {
    if (visited[seed]) {
      continue;
    }
    cluster.clear();
    cluster.push_back(seed);
    visited[seed] = true;
    RoiPoint minimum = points[seed];
    RoiPoint maximum = points[seed];
    // The growing cluster vector also serves as the breadth-first queue.
    for (std::size_t next = 0; next < cluster.size(); ++next) {
      const auto & point = points[cluster[next]];
      minimum.x = std::min(minimum.x, point.x);
      minimum.y = std::min(minimum.y, point.y);
      minimum.z = std::min(minimum.z, point.z);
      maximum.x = std::max(maximum.x, point.x);
      maximum.y = std::max(maximum.y, point.y);
      maximum.z = std::max(maximum.z, point.z);
      const auto cell = detail::cell_for(point, config.distance);
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            const auto found = cells.find({cell.x + dx, cell.y + dy, cell.z + dz});
            if (found == cells.end()) {
              continue;
            }
            auto & candidates = found->second;
            std::size_t offset = 0;
            while (offset < candidates.size()) {
              const auto candidate = candidates[offset];
              if (!visited[candidate]) {
                const auto & other = points[candidate];
                if (std::hypot(
                    static_cast<double>(point.x) - other.x,
                    static_cast<double>(point.y) - other.y,
                    static_cast<double>(point.z) - other.z) > config.distance)
                {
                  ++offset;
                  continue;
                }
                visited[candidate] = true;
                cluster.push_back(candidate);
              }
              // Remove discovered points so dense cells are not scanned for every queued point.
              candidates[offset] = candidates.back();
              candidates.pop_back();
            }
          }
        }
      }
    }
    if (cluster.size() < config.min_points) {
      continue;
    }
    const double span_x = static_cast<double>(maximum.x) - minimum.x;
    const double span_y = static_cast<double>(maximum.y) - minimum.y;
    const double span_z = static_cast<double>(maximum.z) - minimum.z;
    if (std::hypot(span_x, span_y, span_z) < config.min_diagonal ||
      (config.min_volume > 0.0 && span_x * span_y * span_z < config.min_volume))
    {
      continue;
    }
    result.accepted_points += cluster.size();
    result.nearest = std::min(result.nearest, minimum.x);
  }
  return result;
}

}  // namespace mid360_obstacle_avoidance

#endif  // MID360_OBSTACLE_AVOIDANCE_CLUSTER_DETECTION_HPP_
