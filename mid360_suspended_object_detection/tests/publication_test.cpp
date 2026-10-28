#include "scan_core.hpp"
#include <atomic>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace scan_web;
void check(bool value) {if (!value) throw std::runtime_error("Publication telemetry check failed");}
int main() {
  try {
    PublicationTelemetry cache;
    const auto start = Clock::now();
    check(cache.object(start).at("status") == "unsupported");
    cache.configure(false, "/disabled");
    cache.published(json::object{{"seq", 99}}, start);
    check(cache.object(start).at("status") == "disabled" && cache.object(start).at("message").is_null());
    cache.configure(true, "/remapped/topic");
    check(cache.object(start).at("status") == "waiting" && cache.object(start).at("frequency_hz").is_null());
    check(cache.object(start + std::chrono::seconds(8)).at("status") == "waiting");
    cache.failed("first publish failed");
    check(cache.object(start).at("status") == "error" && cache.object(start).at("message").is_null());
    cache.published(json::object{{"seq", 0}}, start);
    check(cache.object(start).at("frequency_hz").is_null());
    for (int i = 1; i <= 60; ++i) cache.published(json::object{{"seq", i}}, start + std::chrono::milliseconds(i * 100));
    auto state = cache.object(start + std::chrono::milliseconds(6050));
    check(state.at("published_count").to_number<int>() == 61 && state.at("frequency_hz").to_number<double>() == 10);
    check(state.at("topic") == "/remapped/topic" && state.at("message").as_object().at("seq") == 60);
    cache.failed("simulated failure");
    state = cache.object(start + std::chrono::milliseconds(6100));
    check(state.at("status") == "error" && state.at("published_count").to_number<int>() == 61 && state.at("message").as_object().at("seq") == 60);
    cache.published(json::object{{"seq", 61}}, start + std::chrono::milliseconds(6200));
    state = cache.object(start + std::chrono::milliseconds(6250));
    check(state.at("status") == "publishing" && state.at("error") == "");
    state = cache.object(start + std::chrono::seconds(8));
    check(state.at("status") == "stalled" && state.at("frequency_hz").to_number<double>() == 0 && !state.at("message").is_null());
    cache.published(json::object{{"seq", 62}}, start + std::chrono::seconds(20));
    check(cache.object(start + std::chrono::seconds(20)).at("frequency_hz").is_null());
    cache.configure(true, "/rolling");
    for (int offset : {0, 1, 5, 6}) cache.published(json::object{{"offset", offset}}, start + std::chrono::seconds(offset));
    check(std::abs(cache.object(start + std::chrono::seconds(6)).at("frequency_hz").to_number<double>() - .4) < 1e-10);
    check(cache.object(start + std::chrono::milliseconds(6100)).at("frequency_hz").to_number<double>() == 1);
    check(cache.object(start + std::chrono::seconds(7)).at("status") == "publishing");
    check(cache.object(start + std::chrono::milliseconds(7001)).at("status") == "stalled");
    cache.configure(true, "/concurrent");
    std::atomic<bool> done{false}, consistent{true};
    std::thread writer([&] {for (int i = 1; i <= 500; ++i) cache.published(json::object{{"seq", i}}); done = true;});
    std::thread reader([&] {while (!done) {
      const auto view = cache.object();
      if (!view.at("message").is_null() && view.at("published_count").to_number<int>() != view.at("message").as_object().at("seq").to_number<int>()) consistent = false;
    }});
    writer.join(); reader.join();
    check(consistent && cache.object().at("published_count").to_number<int>() == 500);
    std::cout << "Publication telemetry tests passed\n";
  } catch (const std::exception & error) {std::cerr << error.what() << '\n'; return 1;}
}
