#ifndef CAR_DEBUG_HEALTH_CORE_HPP_
#define CAR_DEBUG_HEALTH_CORE_HPP_

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>

namespace car_debug {
using SteadyClock = std::chrono::steady_clock;

struct StreamHealth {
  bool seen = false;
  std::uint64_t count = 0;
  SteadyClock::time_point last_rx{};

  void on_receive(SteadyClock::time_point now) {
    seen = true;
    ++count;
    last_rx = now;
  }
  std::chrono::milliseconds age(SteadyClock::time_point now) const {
    if (!seen) return std::chrono::milliseconds{0};
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - last_rx);
  }
  bool fresh(SteadyClock::time_point now, std::chrono::milliseconds max_age) const {
    return seen && now >= last_rx && (now - last_rx) <= max_age;
  }
};

inline double max_wheel_tracking_error(
    const std::array<double, 4>& target,
    const std::array<double, 4>& actual) {
  double peak = 0.0;
  for (std::size_t i = 0; i < 4; ++i) {
    if (!std::isfinite(target[i]) || !std::isfinite(actual[i])) {
      return INFINITY;
    }
    peak = std::max(peak, std::abs(target[i] - actual[i]));
  }
  return peak;
}
}  // namespace car_debug
#endif  // CAR_DEBUG_HEALTH_CORE_HPP_
