#include "scan_core.hpp"
#include <algorithm>

namespace scan_web {
void PublicationTelemetry::configure(bool enabled, std::string topic) {
  std::lock_guard<std::mutex> guard(mutex_);
  supported_ = true; enabled_ = enabled; topic_ = std::move(topic);
  error_.clear(); count_ = 0; last_.reset(); events_.clear(); message_ = nullptr;
}
void PublicationTelemetry::published(json::object message, Clock::time_point now) {
  std::lock_guard<std::mutex> guard(mutex_);
  if (!enabled_) return;
  message_ = std::move(message); last_ = now; ++count_; error_.clear();
  events_.push_back(now);
  while (!events_.empty() && now - events_.front() > std::chrono::seconds(5)) events_.pop_front();
}
void PublicationTelemetry::failed(std::string error) {
  std::lock_guard<std::mutex> guard(mutex_);
  if (enabled_) error_ = std::move(error);
}
json::object PublicationTelemetry::object(Clock::time_point now) const {
  std::lock_guard<std::mutex> guard(mutex_);
  const double age = last_ ? std::max(0.0, std::chrono::duration<double>(now - *last_).count()) : -1;
  std::string status = !supported_ ? "unsupported" : !enabled_ ? "disabled" :
    !error_.empty() ? "error" : !last_ ? "waiting" : age > 1 ? "stalled" : "publishing";
  json::value frequency;
  if (enabled_ && age > 1) frequency = 0.0;
  else if (enabled_) {
    auto first = events_.begin();
    while (first != events_.end() && now - *first > std::chrono::seconds(5)) ++first;
    const auto count = std::distance(first, events_.end());
    if (count >= 2) {
      const auto duration = std::chrono::duration<double>(events_.back() - *first).count();
      if (duration > 0) frequency = (count - 1) / duration;
    }
  }
  return {{"supported", supported_}, {"enabled", enabled_},
    {"topic", supported_ ? json::value(topic_) : json::value(nullptr)},
    {"message_type", "mid360_suspended_object_detection/msg/LargestObstacle"},
    {"status", status}, {"frequency_hz", frequency}, {"published_count", count_},
    {"last_publish_age_s", last_ ? json::value(age) : json::value(nullptr)},
    {"error", error_}, {"message", message_}};
}
}
