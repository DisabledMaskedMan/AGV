#include "scan_core.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

using namespace scan_web;
void check(bool condition) {if (!condition) throw std::runtime_error("Persistence check failed");}
int main() {
  const auto folder = std::filesystem::temp_directory_path() /
    ("scan-config-test-" + std::to_string(Clock::now().time_since_epoch().count()));
  std::filesystem::create_directory(folder);
  try {
    const auto target = folder / "config.yaml", link = folder / "installed.yaml";
    {std::ofstream out(target); out << "mid360_suspended_object_detection:\n  ros__parameters:\n    min_z: 1.2\nother: [1, 2]\n";}
    std::filesystem::create_symlink(target, link);
    Service service(Config::load(link.string()), "ws://test", "scan_link", link.string());
    service.update(json::object{{"min_x", -2}});
    check(Config::load(target.string()).min_x == 0.3);
    service.update(json::object{{"min_y", -3}, {"min_cluster_points", 12}}, true);
    check(Config::load(target.string()).object() == service.config().object());
    check(std::filesystem::is_symlink(link));
    auto document = YAML::LoadFile(target.string());
    check(document["other"].size() == 2 && document["mid360_suspended_object_detection"]["ros__parameters"]["min_z"].as<double>() == 1.2);
    const auto before = service.config().object();
    bool rejected = false;
    try {service.update(json::object{{"min_x", 100}}, true);} catch (...) {rejected = true;}
    check(rejected && service.config().object() == before);
    std::vector<std::thread> workers;
    for (int i = 0; i < 12; ++i) workers.emplace_back([&, i] {service.update(json::object{{"max_x", 5 + i}}, true);});
    for (auto & worker : workers) worker.join();
    check(Config::load(target.string()).object() == service.config().object());
    const auto confirmed = service.config().object();
    // File can still be loaded, but its parent cannot be written: fail after load.
    std::filesystem::permissions(folder, std::filesystem::perms::owner_read | std::filesystem::perms::owner_exec);
    rejected = false;
    try {service.update(json::object{{"min_x", -4}}, true);} catch (...) {rejected = true;}
    std::filesystem::permissions(folder, std::filesystem::perms::owner_all);
    check(rejected && service.config().object() == confirmed && Config::load(target.string()).object() == confirmed);
    for (const auto & entry : std::filesystem::directory_iterator(folder)) check(entry.path().filename().string().rfind(".scan-web-", 0) != 0);
    std::filesystem::remove_all(folder);
    std::cout << "Config persistence tests passed\n";
  } catch (const std::exception & error) {
    std::filesystem::permissions(folder, std::filesystem::perms::owner_all);
    std::filesystem::remove_all(folder);
    std::cerr << error.what() << '\n'; return 1;
  }
}
