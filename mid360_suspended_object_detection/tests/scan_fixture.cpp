#include "scan_core.hpp"
#include <iostream>

int main() {
  using namespace scan_web;
  std::string line;
  while (std::getline(std::cin, line)) {
    try {
      const auto input = json::parse(line).as_object();
      const auto packet = parse_packet(input.at("message"), "scan_link");
      if (!packet) {std::cout << "null\n"; continue;}
      const auto config = Config{}.updated(input.at("config"));
      std::vector<Point> directions;
      double start = 0, increment = 0;
      const auto points = project(*packet, directions, start, increment);
      auto result = detect(points, config);
      json::array coordinates;
      for (const auto & point : points) coordinates.push_back(json::array{point.first, point.second});
      result["points"] = std::move(coordinates);
      std::cout << json::serialize(result) << '\n';
    } catch (const std::exception & error) {
      std::cout << json::serialize(json::object{{"error", error.what()}}) << '\n';
    }
  }
}
