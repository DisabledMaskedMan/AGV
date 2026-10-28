#ifndef MID360_OBSTACLE_AVOIDANCE_ROI_BOUNDS_HPP_
#define MID360_OBSTACLE_AVOIDANCE_ROI_BOUNDS_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace mid360_obstacle_avoidance
{

struct RoiBounds
{
  double min_x;
  double max_x;
  double min_y;
  double max_y;
  double min_z;
  double max_z;
};

inline constexpr double kMinRoiExtent = 0.01;

inline bool validate_roi_bounds(const RoiBounds & roi, std::string & error)
{
  for (const auto bound : {roi.min_x, roi.max_x, roi.min_y, roi.max_y, roi.min_z, roi.max_z}) {
    if (!std::isfinite(bound) || std::abs(bound) > std::numeric_limits<float>::max()) {
      error = "ROI coordinates must be finite and fit float32";
      return false;
    }
  }
  if (roi.min_x < 0.0 || roi.max_x - roi.min_x < kMinRoiExtent ||
    roi.max_y - roi.min_y < kMinRoiExtent || roi.max_z - roi.min_z < kMinRoiExtent ||
    static_cast<float>(roi.min_x) >= static_cast<float>(roi.max_x) ||
    static_cast<float>(roi.min_y) >= static_cast<float>(roi.max_y) ||
    static_cast<float>(roi.min_z) >= static_cast<float>(roi.max_z))
  {
    error = "ROI requires min_x >= 0 and at least 0.01 m per axis";
    return false;
  }
  error.clear();
  return true;
}

// Work on a candidate so invalid feedback cannot partially update the live ROI.
inline bool move_roi_boundary(
  RoiBounds & roi, const std::string & name, double coordinate, std::string & error)
{
  if (!std::isfinite(coordinate)) {
    error = "dragged ROI coordinate is not finite";
    return false;
  }
  auto candidate = roi;
  // Round away from the opposite face when subtraction/addition would shrink the
  // represented gap below one centimeter (especially near negative coordinates).
  const auto lower_limit = [](double upper) {
      const double value = upper - kMinRoiExtent;
      return upper - value < kMinRoiExtent ?
             std::nextafter(value, -std::numeric_limits<double>::infinity()) : value;
    };
  const auto upper_limit = [](double lower) {
      const double value = lower + kMinRoiExtent;
      return value - lower < kMinRoiExtent ?
             std::nextafter(value, std::numeric_limits<double>::infinity()) : value;
    };
  if (name == "roi_min_x") {
    candidate.min_x = std::max(0.0, std::min(coordinate, lower_limit(roi.max_x)));
  } else if (name == "roi_max_x") {
    candidate.max_x = std::max(coordinate, upper_limit(roi.min_x));
  } else if (name == "roi_min_y") {
    candidate.min_y = std::min(coordinate, lower_limit(roi.max_y));
  } else if (name == "roi_max_y") {
    candidate.max_y = std::max(coordinate, upper_limit(roi.min_y));
  } else if (name == "roi_min_z") {
    candidate.min_z = std::min(coordinate, lower_limit(roi.max_z));
  } else if (name == "roi_max_z") {
    candidate.max_z = std::max(coordinate, upper_limit(roi.min_z));
  } else {
    error = "unknown ROI boundary";
    return false;
  }
  if (!validate_roi_bounds(candidate, error)) {
    return false;
  }
  roi = candidate;
  return true;
}

}  // namespace mid360_obstacle_avoidance

#endif  // MID360_OBSTACLE_AVOIDANCE_ROI_BOUNDS_HPP_
