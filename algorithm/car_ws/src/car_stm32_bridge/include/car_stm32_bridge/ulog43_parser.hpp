#pragma once
#include <array>
#include <cstdint>
#include <string>

namespace car_stm32_bridge {
struct Ulog43Frame {
  std::array<double,43> ch{};
  std::uint32_t source_time_ms = 0;
};

// Strict numeric parser for original 43-column CSV (not the OpenMV UART format).
// Reject malformed/NaN/Infinity/partial lines; no zero-filling missing columns.
bool parse_ulog43(const std::string &line, Ulog43Frame &out, std::string *error=nullptr);

struct BodyVelocity {
  double vx_mps=0, vy_mps=0, wz_radps=0;
};
BodyVelocity wheels_to_body(const Ulog43Frame &f, double pulses_per_meter,
                            double wheelbase_half_m, double track_half_m);
} // namespace car_stm32_bridge
