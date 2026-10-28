#ifndef MID360_OBSTACLE_AVOIDANCE_ROI_YAML_HPP_
#define MID360_OBSTACLE_AVOIDANCE_ROI_YAML_HPP_

#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <locale>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "roi_bounds.hpp"

namespace mid360_obstacle_avoidance
{

inline std::string format_yaml_number(double value)
{
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
  auto result = stream.str();
  // ROS parameter doubles must remain doubles even when dragged to integer meters.
  if (result.find_first_of(".eE") == std::string::npos) {
    result += ".0";
  }
  return result;
}

// Intentionally support block-style ROS parameter YAML, not arbitrary YAML aliases/flow maps.
// Reject unsupported or ambiguous target entries rather than reserializing unrelated text.
using YamlNumbers = std::vector<std::pair<std::string, std::string>>;

inline YamlNumbers roi_yaml_values(const RoiBounds & bounds)
{
  return {{"roi_min_x", format_yaml_number(bounds.min_x)},
    {"roi_max_x", format_yaml_number(bounds.max_x)},
    {"roi_min_y", format_yaml_number(bounds.min_y)},
    {"roi_max_y", format_yaml_number(bounds.max_y)},
    {"roi_min_z", format_yaml_number(bounds.min_z)},
    {"roi_max_z", format_yaml_number(bounds.max_z)}};
}

inline bool rewrite_numeric_yaml(
  const std::string & source, const YamlNumbers & values, const std::string & node_name,
  std::string & destination, std::string & error)
{
  std::vector<int> occurrences(values.size(), 0);
  const std::regex mapping(R"(^([ ]*)([A-Za-z_~/][A-Za-z0-9_~/]*)([ \t]*:[ \t]*)(.*)$)");
  const std::regex number(R"(^([-+]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][-+]?[0-9]+)?)([ \t]*(?:#.*)?)$)");
  bool in_node = false;
  bool in_parameters = false;
  int node_count = 0;
  int parameters_count = 0;
  std::size_t parameter_indent = 0;
  std::size_t entry_indent = std::string::npos;
  std::string output;
  output.reserve(source.size() + 128);
  std::size_t begin = 0;
  while (begin < source.size()) {
    const auto newline = source.find('\n', begin);
    const auto end = newline == std::string::npos ? source.size() : newline + 1;
    auto content_end = newline == std::string::npos ? source.size() : newline;
    if (content_end > begin && source[content_end - 1] == '\r') {
      --content_end;
    }
    const std::string line = source.substr(begin, content_end - begin);
    const auto first = line.find_first_not_of(" \t");
    std::string replacement = line;
    if (first != std::string::npos && line[first] != '#') {
      std::smatch match;
      const bool is_mapping = std::regex_match(line, match, mapping);
      const std::string key = is_mapping ? match[2].str() : "";
      const std::string value = is_mapping ? match[4].str() : "";
      const bool block = is_mapping && (value.empty() || value[0] == '#');
      if (first == 0) {
        const auto short_name = node_name.substr(node_name.find_last_of('/') + 1);
        in_node = block && (key == node_name || key == short_name || key == "/" + node_name);
        in_parameters = false;
        if (in_node) {
          ++node_count;
        }
      } else if (in_node) {
        if (in_parameters && first <= parameter_indent) {
          in_parameters = false;
        }
        if (block && key == "ros__parameters") {
          ++parameters_count;
          parameter_indent = first;
          entry_indent = std::string::npos;
          in_parameters = true;
        } else if (in_parameters) {
          if (entry_indent == std::string::npos) {
            entry_indent = first;
          }
          if (first == entry_indent && is_mapping) {
            for (std::size_t i = 0; i < values.size(); ++i) {
              if (key != values[i].first) {
                continue;
              }
              std::smatch numeric;
              if (!std::regex_match(value, numeric, number)) {
                error = "unsupported non-numeric ROI YAML entry: " + key;
                return false;
              }
              ++occurrences[i];
              replacement = match[1].str() + key + match[3].str() +
                values[i].second + numeric[2].str();
            }
          }
        }
      }
    }
    output += replacement;
    output.append(source, content_end, end - content_end);
    begin = end;
  }
  if (node_count != 1 || parameters_count != 1) {
    error = "ROI YAML requires exactly one matching node / ros__parameters block";
    return false;
  }
  for (std::size_t i = 0; i < occurrences.size(); ++i) {
    if (occurrences[i] != 1) {
      error = std::string("ROI YAML requires exactly one scalar ") + values[i].first;
      return false;
    }
  }
  destination = std::move(output);
  error.clear();
  return true;
}

inline bool rewrite_roi_yaml(
  const std::string & source, const RoiBounds & bounds, const std::string & node_name,
  std::string & destination, std::string & error)
{
  return validate_roi_bounds(bounds, error) &&
    rewrite_numeric_yaml(source, roi_yaml_values(bounds), node_name, destination, error);
}

inline bool read_roi_file(
  const std::filesystem::path & path, std::string & contents, std::string & error)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    error = "cannot open ROI YAML: " + path.string();
    return false;
  }
  contents.assign(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
  if (input.bad()) {
    error = "cannot read ROI YAML: " + path.string();
    return false;
  }
  return true;
}

inline bool save_numeric_to_yaml(
  const std::string & filename, const YamlNumbers & values, std::string & error,
  const std::string & node_name = "mid360_obstacle_avoidance")
{
  std::error_code ec;
  // Follow symlink-install files so saving updates the original configuration, not its link.
  const auto target = std::filesystem::canonical(std::filesystem::u8path(filename), ec);
  if (ec || !std::filesystem::is_regular_file(target, ec)) {
    error = "ROI YAML must be an existing regular file: " + filename;
    return false;
  }
  std::string source;
  if (!read_roi_file(target, source, error)) {
    return false;
  }
  std::string updated;
  if (!rewrite_numeric_yaml(source, values, node_name, updated, error)) {
    return false;
  }
  const auto permissions = std::filesystem::status(target, ec).permissions();
  if (ec) {
    error = "cannot inspect ROI YAML permissions";
    return false;
  }
  if ((permissions & std::filesystem::perms::owner_write) == std::filesystem::perms::none &&
    (permissions & std::filesystem::perms::group_write) == std::filesystem::perms::none &&
    (permissions & std::filesystem::perms::others_write) == std::filesystem::perms::none)
  {
    error = "ROI YAML is read-only";
    return false;
  }
  auto temporary = target;
  temporary += ".roi.tmp." + std::to_string(
    std::chrono::steady_clock::now().time_since_epoch().count());
  const auto cleanup = [&temporary]() {
      std::error_code ignored;
      std::filesystem::remove(temporary, ignored);
    };
  {
    std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
    if (!output) {
      error = "cannot create temporary ROI YAML";
      return false;
    }
    output.write(updated.data(), static_cast<std::streamsize>(updated.size()));
    output.flush();
    output.close();
    if (!output) {
      error = "cannot write or close temporary ROI YAML";
      cleanup();
      return false;
    }
  }
  std::filesystem::permissions(temporary, permissions, ec);
  if (ec) {
    error = "cannot preserve ROI YAML permissions";
    cleanup();
    return false;
  }
  // Refuse to overwrite concurrent edits to the other parameters.
  std::string current;
  if (!read_roi_file(target, current, error) || current != source) {
    error = "ROI YAML changed while saving; retry after checking the file";
    cleanup();
    return false;
  }
#ifdef _WIN32
  if (!ReplaceFileW(target.c_str(), temporary.c_str(), nullptr, 0, nullptr, nullptr)) {
    error = "cannot replace ROI YAML (Windows error " + std::to_string(GetLastError()) + ")";
    cleanup();
    return false;
  }
#else
  std::filesystem::rename(temporary, target, ec);
  if (ec) {
    error = "cannot atomically replace ROI YAML: " + ec.message();
    cleanup();
    return false;
  }
#endif
  error.clear();
  return true;
}

inline bool save_roi_to_yaml(
  const std::string & filename, const RoiBounds & bounds, std::string & error,
  const std::string & node_name = "mid360_obstacle_avoidance")
{
  return validate_roi_bounds(bounds, error) &&
    save_numeric_to_yaml(filename, roi_yaml_values(bounds), error, node_name);
}

}  // namespace mid360_obstacle_avoidance

#endif  // MID360_OBSTACLE_AVOIDANCE_ROI_YAML_HPP_
