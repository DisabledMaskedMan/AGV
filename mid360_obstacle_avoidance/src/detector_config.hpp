#ifndef MID360_OBSTACLE_AVOIDANCE_DETECTOR_CONFIG_HPP_
#define MID360_OBSTACLE_AVOIDANCE_DETECTOR_CONFIG_HPP_

#include <limits>
#include "cluster_detection.hpp"
#include "roi_bounds.hpp"

namespace mid360_obstacle_avoidance
{
struct DetectorSettings
{
  RoiBounds roi;
  ClusterConfig cluster;
  double timeout;

  void validate() const
  {
    cluster.validate();
    std::string error;
    if (!validate_roi_bounds(roi, error)) {
      throw std::invalid_argument("Invalid configuration: " + error);
    }
    if (!std::isfinite(timeout) || timeout <= 0 ||
      cluster.min_points > std::numeric_limits<uint32_t>::max())
    {
      throw std::invalid_argument("Invalid configuration: finite timeout > 0 and uint32 point count required");
    }
    for (const auto value : {roi.min_x, roi.max_x, roi.min_y, roi.max_y, roi.min_z, roi.max_z}) {
      if (std::abs(value) > std::numeric_limits<float>::max()) {
        throw std::invalid_argument("Invalid configuration: ROI must fit float32 coordinates");
      }
      detail::cell_coordinate(static_cast<float>(value), cluster.distance);
    }
  }
};
}  // namespace mid360_obstacle_avoidance
#endif
