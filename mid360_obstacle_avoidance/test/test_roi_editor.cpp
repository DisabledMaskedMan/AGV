#include "roi_yaml.hpp"

#include <iostream>
#include <stdexcept>
#include <vector>

using mid360_obstacle_avoidance::RoiBounds;
using mid360_obstacle_avoidance::move_roi_boundary;
using mid360_obstacle_avoidance::rewrite_roi_yaml;
using mid360_obstacle_avoidance::save_roi_to_yaml;

void require(bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

std::string sample_yaml()
{
  return
    "# ROI calibration\n"
    "other_node:\n"
    "  ros__parameters:\n"
    "    roi_min_x: 999.0 # leave this node unchanged\n"
    "mid360_obstacle_avoidance:\n"
    "  ros__parameters:\n"
    "    enable_roi_editor: false\n"
    "    roi_min_x: 0.2 # front boundary\n"
    "    roi_max_x: 1.7\n"
    "    roi_min_y: -0.4\n"
    "    roi_max_y: 0.8\n"
    "    roi_min_z: -0.1\n"
    "    roi_max_z: 0.5\n"
    "    cluster_distance_threshold: 0.05 # preserve noise settings\n"
    "    extra:\n"
    "      roi_min_x: 123.0 # nested key is not the ROI\n"
    "another_node:\n"
    "  ros__parameters:\n"
    "    roi_max_x: 888.0\n";
}

void test_bounds()
{
  const RoiBounds startup{0.2, 1.7, -0.4, 0.8, -0.1, 0.5};
  std::string error;
  for (const auto & name : {"roi_min_x", "roi_max_x", "roi_min_y", "roi_max_y",
      "roi_min_z", "roi_max_z"})
  {
    auto roi = startup;
    require(move_roi_boundary(roi, name, 0.3, error), "move " + std::string(name));
    require(mid360_obstacle_avoidance::validate_roi_bounds(roi, error), "valid moved ROI");
    require(move_roi_boundary(roi, name, 100.0, error), "clamp upper crossing");
    require(mid360_obstacle_avoidance::validate_roi_bounds(roi, error), "valid upper clamp");
    require(move_roi_boundary(roi, name, -100.0, error), "clamp lower crossing");
    require(mid360_obstacle_avoidance::validate_roi_bounds(roi, error), "valid lower clamp");
  }
  auto roi = startup;
  require(move_roi_boundary(roi, "roi_min_x", -1.0, error) && roi.min_x == 0.0,
    "minimum X cannot become negative");
  const auto before = roi;
  for (const double coordinate : {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), 1e100})
  {
    require(!move_roi_boundary(roi, "roi_max_x", coordinate, error), "reject invalid feedback");
    require(roi.max_x == before.max_x && roi.min_x == before.min_x, "no partial update");
  }
  require(!move_roi_boundary(roi, "not_a_handle", 1.0, error), "reject unknown handle");
  require(!mid360_obstacle_avoidance::validate_roi_bounds({0, 0.009, 0, 1, 0, 1}, error),
    "reject extent below one centimeter");
  require(mid360_obstacle_avoidance::validate_roi_bounds({0, 0.01, 0, 0.01, 0, 0.01}, error),
    "inclusive one centimeter extent");
  for (int i = -1000; i <= 1000; ++i) {
    const double upper = static_cast<double>(i) / 1000.0;
    roi = {0.2, 1.7, upper - 1.0, upper, -0.1, 0.5};
    require(move_roi_boundary(roi, "roi_min_y", upper + 1.0, error),
      "decimal upper-face clamp must be reachable");
    require(roi.max_y - roi.min_y >= mid360_obstacle_avoidance::kMinRoiExtent,
      "decimal clamp must never shrink below one centimeter");
    roi = {0.2, 1.7, upper, upper + 1.0, -0.1, 0.5};
    require(move_roi_boundary(roi, "roi_max_y", upper - 1.0, error),
      "decimal lower-face clamp must be reachable");
    require(roi.max_y - roi.min_y >= mid360_obstacle_avoidance::kMinRoiExtent,
      "decimal lower clamp must never shrink below one centimeter");
  }
}

void test_rewrite()
{
  const RoiBounds updated{0.0, 2.0, -1.0, 1.0, -0.25, 0.75};
  std::string output;
  std::string error;
  const auto source = sample_yaml();
  require(rewrite_roi_yaml(source, updated, "mid360_obstacle_avoidance", output, error), error);
  require(output.find("    roi_min_x: 0.0 # front boundary") != std::string::npos,
    "preserve inline comment and float YAML type");
  require(output.find("    roi_max_x: 2.0\n") != std::string::npos, "integer meters stay double");
  require(output.find("    roi_min_x: 999.0 # leave this node unchanged") != std::string::npos,
    "preserve other node");
  require(output.find("      roi_min_x: 123.0 # nested key is not the ROI") != std::string::npos,
    "preserve nested key");
  require(output.find("cluster_distance_threshold: 0.05 # preserve noise settings") !=
    std::string::npos, "preserve clustering settings");
  std::string identical;
  require(rewrite_roi_yaml(output, updated, "/mid360_obstacle_avoidance", identical, error), error);
  require(output == identical, "repeated save is byte-identical");
  std::string crlf;
  for (char value : source) {
    if (value == '\n') {
      crlf += '\r';
    }
    crlf += value;
  }
  crlf.resize(crlf.size() - 2);  // Also preserve absence of a final newline.
  require(rewrite_roi_yaml(crlf, updated, "mid360_obstacle_avoidance", output, error), error);
  require(output.back() != '\n' && output.find("\r\n") != std::string::npos,
    "preserve CRLF and missing final newline");
  for (std::size_t i = 0; i < output.size(); ++i) {
    require(output[i] != '\n' || (i > 0 && output[i - 1] == '\r'), "no newline conversion");
  }
  std::vector<std::string> invalid{
    "# ros__parameters:\nroi_min_x: 0.0\n", "wrong_node:\n  ros__parameters:\n",
    source + source};
  auto missing = source;
  missing.erase(missing.find("    roi_min_z: -0.1\n"), std::string("    roi_min_z: -0.1\n").size());
  invalid.push_back(missing);
  auto duplicate = source;
  duplicate.insert(duplicate.find("    roi_max_x:"), "    roi_min_x: 0.1\n");
  invalid.push_back(duplicate);
  auto alias = source;
  alias.replace(alias.find("roi_max_x: 1.7"), 14, "roi_max_x: *alias");
  invalid.push_back(alias);
  for (const auto & text : invalid) {
    output = "untouched";
    require(!rewrite_roi_yaml(text, updated, "mid360_obstacle_avoidance", output, error),
      "reject missing/ambiguous/unsupported YAML target");
    require(output == "untouched", "rewrite failure leaves output intact");
  }
}

void write_file(const std::filesystem::path & path, const std::string & contents)
{
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  output << contents;
  output.close();
  require(static_cast<bool>(output), "write test fixture");
}

std::string read_file(const std::filesystem::path & path)
{
  std::string output;
  std::string error;
  require(mid360_obstacle_avoidance::read_roi_file(path, output, error), error);
  return output;
}

void test_save(const std::filesystem::path & directory)
{
  const RoiBounds roi{0, 2, -1, 1, -0.25, 0.75};
  const auto target = directory / "obstacle.yaml";
  const auto source = sample_yaml();
  std::string error;
  write_file(target, source);
  require(save_roi_to_yaml(target.u8string(), roi, error), error);
  const auto saved = read_file(target);
  require(saved.find("roi_max_x: 2.0") != std::string::npos, "saved values");
  require(!save_roi_to_yaml((directory / "missing.yaml").u8string(), roi, error),
    "do not create a missing configuration");
  require(!save_roi_to_yaml(directory.u8string(), roi, error), "reject directory");
  write_file(target, "not_the_node:\n  ros__parameters:\n");
  const auto invalid = read_file(target);
  require(!save_roi_to_yaml(target.u8string(), roi, error), "reject malformed node section");
  require(read_file(target) == invalid, "failed rewrite preserves original file");
  write_file(target, source);
  const auto original_permissions = std::filesystem::status(target).permissions();
  std::filesystem::permissions(target, std::filesystem::perms::owner_read);
  require(!save_roi_to_yaml(target.u8string(), roi, error), "reject read-only configuration");
  require(read_file(target) == source, "read-only failure preserves original file");
  std::filesystem::permissions(target, original_permissions);
#ifdef _WIN32
  const auto held = CreateFileW(target.c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr,
      OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
  require(held != INVALID_HANDLE_VALUE, "hold YAML without delete sharing");
  const bool replaced = save_roi_to_yaml(target.u8string(), roi, error);
  CloseHandle(held);
  require(!replaced && read_file(target) == source, "failed replacement never deletes target");
#endif
  std::error_code ec;
  const auto link = directory / "linked.yaml";
  std::filesystem::create_symlink(target, link, ec);
  if (!ec) {
    require(save_roi_to_yaml(link.u8string(), roi, error), error);
    require(std::filesystem::is_symlink(link) && read_file(target).find("roi_max_x: 2.0") !=
      std::string::npos, "symlink-install keeps its link and updates the source");
  }
  for (const auto & entry : std::filesystem::directory_iterator(directory)) {
    require(entry.path().filename().string().find(".roi.tmp.") == std::string::npos,
      "temporary files cleaned after success/failure");
  }
}

void test_full_configuration_yaml()
{
  using namespace mid360_obstacle_avoidance;
  auto source = sample_yaml();
  // Keep the same newline format and unrelated text, including comments.
  const auto newline = source.find("\r\n") == std::string::npos ? "\n" : "\r\n";
  const std::string extras = std::string("    min_obstacle_points: 10 # points") + newline +
    "    min_cluster_diagonal: 0.1" + newline + "    min_cluster_volume: 0.0" + newline +
    "    cloud_timeout: 0.5" + newline;
  // Insert directly in the target parameter block, before the first ROI scalar.
  const auto position = source.find("    roi_min_x", source.find("mid360_obstacle_avoidance:"));
  require(position != std::string::npos, "configuration fixture target");
  source.insert(position, extras);
  auto values = roi_yaml_values({0, 2, -1, 1, -.25, .75});
  values.insert(values.end(), {{"cluster_distance_threshold", "0.125"},
    {"min_obstacle_points", "4294967295"}, {"min_cluster_diagonal", "0.25"},
    {"min_cluster_volume", "0.0"}, {"cloud_timeout", "1.0"}});
  std::string output, error;
  require(rewrite_numeric_yaml(source, values, "mid360_obstacle_avoidance", output, error), error);
  require(output.find("min_obstacle_points: 4294967295 # points") != std::string::npos,
    "uint32 point count stays an integer and comment stays intact");
  // Rewriting to the original scalar strings must round-trip every other byte.
  YamlNumbers originals;
  const auto target_block = source.substr(source.find("mid360_obstacle_avoidance:"));
  for (const auto & value : values) {
    const std::regex pattern("(^|\\n)    " + value.first + ": ([^ #\\r\\n]+)");
    std::smatch match;
    require(std::regex_search(target_block, match, pattern), "find original scalar");
    originals.emplace_back(value.first, match[2].str());
  }
  std::string restored;
  require(rewrite_numeric_yaml(output, originals, "mid360_obstacle_avoidance", restored, error), error);
  require(restored == source, "eleven-field round trip preserves all other bytes");
}

int main()
{
  const auto directory = std::filesystem::temp_directory_path() /
    ("mid360_roi_test_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  try {
    require(std::filesystem::create_directory(directory), "create isolated test directory");
    test_bounds();
    test_rewrite();
    test_save(directory);
    test_full_configuration_yaml();
    std::filesystem::remove_all(directory);
    std::cout << "All ROI boundary and YAML persistence tests passed\n";
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    std::error_code ignored;
    std::filesystem::remove_all(directory, ignored);
    return 1;
  }
  return 0;
}
