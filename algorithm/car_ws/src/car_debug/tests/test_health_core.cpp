#include "car_debug/health_core.hpp"
#include <cassert>
#include <cmath>
#include <array>
#include <chrono>
int main() {
  using namespace std::chrono_literals;
  const auto t = car_debug::SteadyClock::now();
  car_debug::StreamHealth x;
  assert(!x.fresh(t, 1s));
  x.on_receive(t);
  assert(x.fresh(t + 500ms, 1s));
  assert(!x.fresh(t + 1001ms, 1s));
  assert(x.count == 1);
  assert(x.age(t + 510ms).count() == 510);
  std::array<double, 4> target{{100, -100, 250, 0}};
  std::array<double, 4> actual{{95, -115, 350, -10}};
  assert(std::abs(car_debug::max_wheel_tracking_error(target, actual)-100.0) < 1e-9);
}
