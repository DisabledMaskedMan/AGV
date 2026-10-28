#include "scan_core.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace scan_web;

void require(bool condition, const char * message) {if (!condition) throw std::runtime_error(message);}
template<class Function> void rejects(Function function) {
  bool rejected = false;
  try {function();} catch (const std::exception &) {rejected = true;}
  require(rejected, "Expected rejection");
}
json::value envelope() {
  return json::parse(R"({"channel":"ros","op":"publish","topic":"/scan","msg":[{"rame_id":"/scan_link","name":"scan","scan":[1000,2000,0,null,true,50001],"angle_start":0,"increment":1.5707963267948966,"range_min":0.03,"range_max":50,"timeoutSec":0,"laserOffset":[0.94,0,-0.017]}]})");
}
int main() {
  try {
    Config config;
    rejects([&] {config.updated(json::parse(R"({"min_cluster_points":1.2})"));});
    rejects([&] {config.updated(json::parse(R"({"min_x":6})"));});
    rejects([&] {config.updated(json::parse(R"({"cloud_timeout":true})"));});
    rejects([&] {config.updated(json::parse(R"({"min_z":1})"));});
    auto packet = parse_packet(envelope(), "scan_link");
    require(packet.has_value() && packet->ranges[0] == 1 && packet->ranges[4] == 0, "Unit conversion/filter");
    std::vector<Point> directions;
    double start = 0, increment = 0;
    const auto points = project(*packet, directions, start, increment);
    require(points.size() == 2 && std::abs(points[1].second - 2) < 1e-10 && points[0].first == 1, "Projection/offset");
    auto unrelated = envelope();
    unrelated.as_object()["msg"].as_array()[0].as_object()["rame_id"] = "scan4_link";
    require(!parse_packet(unrelated, "scan_link"), "Other frame");
    require(!parse_packet(json::parse(R"({"op":"status"})"), "scan_link"), "Status ignored");
    for (const auto & change : {json::object{{"increment", 0}}, json::object{{"timeoutSec", 1}},
                               json::object{{"scan", json::array{}}}, json::object{{"range_max", 0.01}}}) {
      auto invalid = envelope();
      for (const auto & field : change) invalid.as_object()["msg"].as_array()[0].as_object()[field.key()] = field.value();
      rejects([&] {parse_packet(invalid, "scan_link");});
    }
    std::vector<Point> block;
    for (int column = 0; column < 3; ++column) for (int row = 0; row < 3; ++row) block.emplace_back(1 + column * 0.05, -0.05 + row * 0.05);
    auto result = detect(block, config);
    require(result.at("roi_count").to_number<int>() == 9 && result.at("objects").as_array().size() == 1, "Clustering");
    require(detect(block, config.updated(json::object{{"min_width", 0.2}})).at("objects").as_array().empty(), "Size filter");
    require(detect({}, config).at("roi_count").to_number<int>() == 0, "Empty ROI");
    std::vector<Point> candidates;
    for (int x = 0; x < 4; ++x) for (int y = 0; y < 4; ++y) {
      candidates.emplace_back(2 + x * 0.0625, 0.25 + y * 0.0625);
      candidates.emplace_back(3 + x * 0.0625, 0.25 + y * 0.0625);
    }
    for (int x = 0; x < 3; ++x) for (int y = 0; y < 3; ++y) candidates.emplace_back(1 + x * 0.05, -0.2 + y * 0.05);
    auto selected = detect_objects(candidates, config);
    auto largest = selected.largest();
    require(selected.objects.size() == 3 && largest && largest->count == 16, "Largest area selection");
    require(largest->area() == 0.03515625 && largest->nearest == Point{2, 0.25}, "Area tie chooses nearest obstacle");
    require(largest->center() == Point{2.09375, 0.34375},
      "Center XY and signed distances");
    require(std::abs(std::hypot(largest->center().first, largest->center().second) - 2.1217806495959946) < 1e-10,
      "Center distance");
    Detection centered;
    centered.objects = {{-1, 1, -0.5, 0.5, {-1, 0}, 1, 8},
                        {-1, 1, -0.5, 0.5, {1, 0}, 1, 8}};
    require(centered.largest()->center() == Point{0, 0}, "Axis-centered candidate");
    centered.objects = {{-3, -2, -1, 0, {-2, 0}, 2, 8},
                        {2, 3, 0, 1, {2, 0}, 2, 8}};
    require(centered.largest()->center() == Point{-2.5, -0.5}, "Equal area and distance sort by center XY");
    auto cluster_packet = *packet;
    cluster_packet.ranges.clear();
    for (int index = 0; index < 21; ++index) cluster_packet.ranges.push_back(index % 2 ? 2.1 : 2.0);
    cluster_packet.angle_start = -0.05;
    cluster_packet.increment = 0.005;
    Service detected_service(config, "ws://test:6060", "scan_link");
    detected_service.accept(cluster_packet); detected_service.process();
    auto detected_snapshot = detected_service.obstacle_snapshot();
    auto web_selected = json::parse(detected_service.state_text()).as_object();
    const auto expected_box = detected_snapshot.largest->object();
    for (const auto & field : expected_box) {
      const auto & actual = web_selected.at("largest_obstacle").as_object().at(field.key());
      require(std::abs(actual.to_number<double>() - field.value().to_number<double>()) < 1e-12,
        "Web uses the ROS selected obstacle");
    }
    require(web_selected.at("ros_publication").as_object().at("status") == "unsupported", "Non-ROS default telemetry");
    require(detected_snapshot.data_valid && detected_snapshot.status == "detected" &&
      detected_snapshot.candidate_count == 1 && detected_snapshot.largest &&
      detected_snapshot.largest->count == 21, "Detected snapshot");
    const auto detected_center = detected_snapshot.largest->center();
    require(detected_center.first > 2 && std::abs(detected_center.second) < 0.01 &&
      std::abs(detected_snapshot.largest->distance - 2) < 0.01,
      "Center and nearest actual point");
    detected_service.update(json::object{{"min_x", -3}, {"max_x", -1}});
    auto pending_snapshot = detected_service.obstacle_snapshot();
    require(json::parse(detected_service.state_text()).as_object().at("largest_obstacle").is_null(), "ROI update clears red frame");
    require(!pending_snapshot.data_valid && pending_snapshot.status == "processing" &&
      !pending_snapshot.largest && pending_snapshot.candidate_count == 0, "ROI update immediately clears prior detection");
    detected_service.process();
    require(detected_service.obstacle_snapshot().status == "clear", "Updated empty ROI is valid");
    detected_service.accept(cluster_packet);
    auto duplicate_snapshot = detected_service.obstacle_snapshot();
    require(duplicate_snapshot.data_valid && duplicate_snapshot.status == "clear", "Duplicate scan keeps result");
    cluster_packet.ranges[0] = 2.01;
    detected_service.accept(cluster_packet);
    pending_snapshot = detected_service.obstacle_snapshot();
    require(pending_snapshot.data_valid && pending_snapshot.status == "clear",
      "Fresh last-completed result stays available during the next scan analysis");
    Config rear = config.updated(json::object{{"min_x", -3}, {"max_x", -0.3}, {"min_y", -1}, {"max_y", 1}});
    std::vector<Point> negative_points;
    for (int x = 0; x < 3; ++x) for (int y = 0; y < 3; ++y) negative_points.emplace_back(-2 + x * 0.05, -0.2 + y * 0.05);
    const auto negative = detect_objects(negative_points, rear).largest();
    require(negative && negative->center().first < 0 && negative->center().second < 0, "Negative XY directions");
    Service service(config.updated(json::object{{"cloud_timeout", 0.04}}), "ws://test:6060", "scan_link");
    require(json::parse(service.state_text()).as_object().at("status") == "waiting", "Initial state");
    service.accept(*packet); service.process(); service.accept(*packet); service.process();
    auto snapshot = service.obstacle_snapshot();
    require(snapshot.data_valid && snapshot.status == "clear" && !snapshot.largest, "Valid no-obstacle snapshot");
    auto state = json::parse(service.state_text()).as_object();
    require(state.at("received_count").to_number<int>() == 2 && state.at("processed_count").to_number<int>() == 1, "Duplicate deduplication");
    require(state.at("content_version").to_number<int>() == 1, "Duplicate content version");
    std::this_thread::sleep_for(std::chrono::milliseconds(55));
    state = json::parse(service.state_text()).as_object();
    require(state.at("status") == "timeout" && state.at("objects").as_array().empty(), "Timeout clears state");
    snapshot = service.obstacle_snapshot();
    require(!snapshot.data_valid && snapshot.status == "timeout" && !snapshot.largest && snapshot.candidate_count == 0,
      "Timeout clears ROS snapshot");
    require(json::parse(service.state_text()).as_object().at("largest_obstacle").is_null(), "Timeout clears red frame");
    service.accept(*packet); service.update(json::object{{"max_x", 4}}); service.process();
    require(json::parse(service.state_text()).as_object().at("processed_count").to_number<int>() == 2, "Config reanalysis");
    service.fail("lost", "disconnected");
    require(json::parse(service.state_text()).as_object().at("objects").as_array().empty(), "Error clears state");
    snapshot = service.obstacle_snapshot();
    require(!snapshot.data_valid && snapshot.status == "disconnected" && !snapshot.largest, "Disconnect clears ROS snapshot");
    service.accept(*packet); service.process();
    require(json::parse(service.state_text()).as_object().at("status") == "clear", "Recovery");
    std::cout << "scan_core tests passed\n";
  } catch (const std::exception & error) {std::cerr << error.what() << '\n'; return 1;}
}
