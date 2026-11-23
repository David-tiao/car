#pragma once

#include "pi_qr/task_code.hpp"

#include <array>
#include <optional>
#include <string>

namespace pi_material {

// QR group 0/2 gives the ordered colors for two batches; group 1/3
// gives the corresponding processing rings. Stacking uses batch 1's
// color-to-storage-ring mapping, not batch 2's processing-ring permutation.
struct Target {
    int step = 0;      // 0..5
    int batch = 0;     // 1 or 2
    int color = 0;     // 1 red, 2 yellow, 3 blue, 4 green, 5 black, 6 cyan
    int process_ring = 0;
    int storage_ring = 0;
};

std::array<Target, 6> make_targets(const pi_qr::TaskCode& task);
const char* color_name(int id);

struct Spot {
    double u = 0;           // image pixel x, right positive
    double v = 0;           // image pixel y, downward positive
    double diameter_px = 0;
    double area_px = 0;
};

// Separate frames only; a missing/ambiguous detection resets the candidate.
// Stable observations never imply an actual gripper action.
class StableSpot {
public:
    explicit StableSpot(int required = 3, double jitter_px = 10.0);
    std::optional<Spot> update(const std::optional<Spot>& spot);
    int count() const { return count_; }
    void reset();

private:
    int required_;
    double jitter_px_;
    int count_ = 0;
    std::optional<Spot> previous_;
};

struct Jacobian {
    // Measured camera image displacement [du,dv] (px) in response
    // to positive robot planar motion [dx,dy] (mm): e = J * motion.
    double j00 = 0, j01 = 0, j10 = 0, j11 = 0;
};

struct ServoSettings {
    double ref_u_px = 0;
    double ref_v_px = 0;
    double tolerance_px = 8;
    double gain_per_s = 0.7;
    double max_step_mm = 5;
    double max_speed_mm_s = 25;
    bool reference_taught = false;
    std::optional<Jacobian> jacobian;
};

struct ServoEstimate {
    double error_u_px = 0;
    double error_v_px = 0;
    bool pixel_centered = false;
    bool calibrated = false;
    // This is only an ADVISORY based on a measured planar Jacobian.
    // Full desired robot motion before clipping, and bounded small-step advice.
    double full_x_mm = 0, full_y_mm = 0;
    double step_x_mm = 0, step_y_mm = 0;
    double vx_mm_s = 0, vy_mm_s = 0;
};

ServoEstimate calculate_servo(const Spot& spot, const ServoSettings& cfg);
bool valid_jacobian(const Jacobian& j);

}  // namespace pi_material
