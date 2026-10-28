#include "cluster_detection.hpp"

#include <chrono>
#include <iostream>
#include <numeric>
#include <random>
#include <string>

using mid360_obstacle_avoidance::ClusterConfig;
using mid360_obstacle_avoidance::ClusterResult;
using mid360_obstacle_avoidance::RoiPoint;
using mid360_obstacle_avoidance::detect_clusters;

void require(bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void check(
  const std::vector<RoiPoint> & points, const ClusterConfig & config,
  std::size_t count, float nearest, const std::string & name)
{
  const auto actual = detect_clusters(points, config);
  require(actual.accepted_points == count, name + ": accepted count");
  require(actual.nearest == nearest, name + ": nearest distance");
}

// Independent all-pairs graph traversal, used only as a small-input reference.
ClusterResult reference(const std::vector<RoiPoint> & points, const ClusterConfig & config)
{
  std::vector<bool> seen(points.size(), false);
  ClusterResult result;
  for (std::size_t seed = 0; seed < points.size(); ++seed) {
    if (seen[seed]) {
      continue;
    }
    std::vector<std::size_t> component{seed};
    seen[seed] = true;
    for (std::size_t next = 0; next < component.size(); ++next) {
      for (std::size_t other = 0; other < points.size(); ++other) {
        const auto & a = points[component[next]];
        const auto & b = points[other];
        if (!seen[other] && std::hypot(
            static_cast<double>(a.x) - b.x, static_cast<double>(a.y) - b.y,
            static_cast<double>(a.z) - b.z) <= config.distance)
        {
          seen[other] = true;
          component.push_back(other);
        }
      }
    }
    RoiPoint lo = points[seed];
    RoiPoint hi = points[seed];
    for (const auto index : component) {
      const auto & p = points[index];
      lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
      hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
    }
    const double x = static_cast<double>(hi.x) - lo.x;
    const double y = static_cast<double>(hi.y) - lo.y;
    const double z = static_cast<double>(hi.z) - lo.z;
    if (component.size() >= config.min_points && std::hypot(x, y, z) >= config.min_diagonal &&
      (config.min_volume == 0.0 || x * y * z >= config.min_volume))
    {
      result.accepted_points += component.size();
      result.nearest = std::min(result.nearest, lo.x);
    }
  }
  return result;
}

void test_geometry()
{
  const float inf = std::numeric_limits<float>::infinity();
  const ClusterConfig defaults;
  check({}, defaults, 0, inf, "empty ROI");
  check({{0.5F, 0, 0}, {1, 0, 0}, {2, 0, 0}}, defaults, 0, inf, "isolated noise");
  check({{0.5F, 0, 0}, {0.5625F, 0, 0}}, defaults, 0, inf, "too few points");
  check({{0.5F, 0, 0}, {0.51F, 0, 0}, {0.52F, 0, 0}}, defaults,
    0, inf, "too small");
  check(std::vector<RoiPoint>(100, {0.5F, 0, 0}), defaults,
    0, inf, "duplicate coordinates");
  check({{2, 0, 0}, {2, 0.03125F, 0}, {2, 0.0625F, 0}}, defaults,
    3, 2, "flat surface at two meters");
  check({{0.5F, 0, 0}, {0.5F, 0.03125F, 0}, {0.5F, 0.0625F, 0},
      {2, 0, 0}, {2, 0, 0.03125F}, {2, 0, 0.0625F}, {0.25F, 0, 0}},
    defaults, 6, 0.5F, "multiple clusters exclude nearer noise");

  ClusterConfig exact{0.125, 3, 0.25, 0.0};
  std::vector<RoiPoint> chain{{0.5F, -0.125F, 0}, {0.5F, 0, 0}, {0.5F, 0.125F, 0}};
  check(chain, exact, 3, 0.5F, "inclusive radius and diagonal, negative cell boundary");
  std::mt19937 random(360);
  for (int run = 0; run < 20; ++run) {
    std::shuffle(chain.begin(), chain.end(), random);
    check(chain, exact, 3, 0.5F, "permuted chain");
  }
  exact.distance = std::nextafter(0.125, 0.0);
  check(chain, exact, 0, inf, "just outside neighbor radius");
  exact.distance = 0.125;
  exact.min_diagonal = std::nextafter(0.25, 1.0);
  check(chain, exact, 0, inf, "just below diagonal threshold");
  check({{0.5F, 0, 0}, {0.625F, 0.125F, 0}, {0.75F, 0.25F, 0}},
    {0.125, 3, 0.0, 0.0}, 0, inf, "3D distance, not cell membership");
  check(std::vector<RoiPoint>(3, {0.5F, 0, 0}), {0.1, 3, 0.0, 0.0},
    3, 0.5F, "zero diagonal explicitly allowed");

  const std::vector<RoiPoint> box{
    {0.5F, 0, 0}, {0.625F, 0, 0}, {0.625F, 0.125F, 0}, {0.625F, 0.125F, 0.125F}};
  const double volume = 0.125 * 0.125 * 0.125;
  check(box, {0.125, 3, 0.05, volume}, 4, 0.5F, "inclusive volume");
  check(box, {0.125, 3, 0.05, std::nextafter(volume, 1.0)},
    0, inf, "below volume threshold");
  check({{0.5F, 0, 0}, {0.5F, 0.03125F, 0}, {0.5F, 0.0625F, 0}},
    {0.1, 3, 0.05, volume}, 0, inf, "volume rejects plane when enabled");
}

void test_reference()
{
  std::mt19937 random(360);
  std::uniform_int_distribution<int> coordinate(-16, 16);
  for (int run = 0; run < 200; ++run) {
    std::vector<RoiPoint> points;
    for (int i = 0; i < 100; ++i) {
      points.push_back({coordinate(random) / 16.0F, coordinate(random) / 16.0F,
        coordinate(random) / 16.0F});
    }
    const ClusterConfig config{
      run % 2 == 0 ? 0.25 : 0.4, 3, 0.125, run % 3 == 0 ? 0.001 : 0.0};
    const auto expected = reference(points, config);
    for (int permutation = 0; permutation < 3; ++permutation) {
      std::shuffle(points.begin(), points.end(), random);
      const auto actual = detect_clusters(points, config);
      require(actual.accepted_points == expected.accepted_points &&
        actual.nearest == expected.nearest, "random graph reference / input order");
    }
  }
}

void test_validation()
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const auto & config : std::vector<ClusterConfig>{
      {0, 3, 0, 0}, {-0.1, 3, 0, 0}, {nan, 3, 0, 0}, {inf, 3, 0, 0},
      {0.1, 0, 0, 0}, {0.1, 3, -1, 0}, {0.1, 3, nan, 0}, {0.1, 3, inf, 0},
      {0.1, 3, 0, -1}, {0.1, 3, 0, nan}, {0.1, 3, 0, inf}})
  {
    bool rejected = false;
    try {
      detect_clusters({}, config);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    require(rejected, "invalid cluster configuration");
  }
  bool rejected = false;
  try {
    detect_clusters({{1, 0, 0}}, {1e-300, 3, 0.05, 0});
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  require(rejected, "grid range protected against overflow");
}

int main()
{
  try {
    test_geometry();
    test_reference();
    test_validation();
    std::vector<RoiPoint> dense(20000, {0.5F, 0, 0});
    dense.push_back({0.5625F, 0, 0});
    const auto start = std::chrono::steady_clock::now();
    const auto result = detect_clusters(dense, {});
    const double milliseconds = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    require(result.accepted_points == dense.size(), "dense cell traversal");
    std::cout << "All cluster tests passed; synthetic 20001-point dense ROI: " <<
      milliseconds << " ms (not a hardware/real-cloud benchmark)\n";
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
