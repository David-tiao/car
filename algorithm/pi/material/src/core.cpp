#include "pi_material/core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pi_material {
namespace {
bool finite(double x) { return std::isfinite(x); }
void limit_vector(double& x, double& y, double max_length) {
    const double length = std::hypot(x, y);
    if (length > max_length && length > 0) {
        const double factor = max_length / length;
        x *= factor;
        y *= factor;
    }
}
}  // namespace

const char* color_name(int id) {
    switch (id) {
        case 1: return "RED";
        case 2: return "YELLOW";
        case 3: return "BLUE";
        case 4: return "GREEN";
        case 5: return "BLACK";
        case 6: return "CYAN";
        default: return "UNKNOWN";
    }
}

std::array<Target, 6> make_targets(const pi_qr::TaskCode& task) {
    const auto parsed = pi_qr::parse_task_code(task.text);
    if (!parsed) throw std::invalid_argument("invalid QR task code");
    std::array<int, 7> storage_ring{};
    for (int i = 0; i < 3; ++i) {
        const int color = parsed->groups[0][i] - '0';
        storage_ring[color] = parsed->groups[1][i] - '0';
    }
    std::array<Target, 6> targets{};
    for (int index = 0; index < 6; ++index) {
        const int batch = index / 3;
        const int local = index % 3;
        const int color = parsed->groups[batch * 2][local] - '0';
        targets[index] = {index, batch + 1, color,
                          parsed->groups[batch * 2 + 1][local] - '0',
                          storage_ring[color]};
    }
    return targets;
}

StableSpot::StableSpot(int required, double jitter_px)
    : required_(required), jitter_px_(jitter_px) {
    if (required < 1 || required > 100 || !finite(jitter_px) || jitter_px < 0)
        throw std::invalid_argument("bad stability configuration");
}

void StableSpot::reset() {
    count_ = 0;
    previous_.reset();
}

std::optional<Spot> StableSpot::update(const std::optional<Spot>& spot) {
    if (!spot || !finite(spot->u) || !finite(spot->v) ||
        !finite(spot->diameter_px) || !finite(spot->area_px) ||
        spot->diameter_px <= 0 || spot->area_px <= 0) {
        reset();
        return std::nullopt;
    }
    if (previous_ && std::hypot(spot->u - previous_->u,
                                spot->v - previous_->v) <= jitter_px_) {
        count_ = std::min(count_ + 1, required_);
    } else {
        count_ = 1;
    }
    previous_ = spot;
    return count_ >= required_ ? previous_ : std::nullopt;
}

bool valid_jacobian(const Jacobian& j) {
    if (!finite(j.j00) || !finite(j.j01) ||
        !finite(j.j10) || !finite(j.j11)) return false;
    const double det = j.j00 * j.j11 - j.j01 * j.j10;
    const double norm = std::max({std::abs(j.j00), std::abs(j.j01),
                                  std::abs(j.j10), std::abs(j.j11)});
    // Reject singular/ill-conditioned local mappings. Assumes J is locally
    // calibrated at the same camera mount, distance and robot configuration.
    return norm >= 1e-6 && std::abs(det) / (norm * norm) >= 0.02;
}

ServoEstimate calculate_servo(const Spot& spot, const ServoSettings& cfg) {
    if (!finite(cfg.ref_u_px) || !finite(cfg.ref_v_px) ||
        !finite(cfg.tolerance_px) || cfg.tolerance_px <= 0 ||
        !finite(cfg.gain_per_s) || cfg.gain_per_s <= 0 ||
        !finite(cfg.max_step_mm) || cfg.max_step_mm <= 0 ||
        !finite(cfg.max_speed_mm_s) || cfg.max_speed_mm_s <= 0)
        throw std::invalid_argument("invalid servo settings");
    if (!finite(spot.u) || !finite(spot.v))
        throw std::invalid_argument("invalid image coordinate");

    ServoEstimate result;
    result.error_u_px = spot.u - cfg.ref_u_px;
    result.error_v_px = spot.v - cfg.ref_v_px;
    // A default diagnostic crosshair is not a taught pickup reference.
    result.pixel_centered = cfg.reference_taught &&
        std::hypot(result.error_u_px,
                   result.error_v_px) <= cfg.tolerance_px;

    if (!cfg.reference_taught || !cfg.jacobian) return result;
    if (!valid_jacobian(*cfg.jacobian))
        throw std::invalid_argument("uncalibrated or singular image Jacobian");

    const auto& j = *cfg.jacobian;
    const double det = j.j00 * j.j11 - j.j01 * j.j10;
    // e = p_measured - p_goal. Desired correction J*dq = -e.
    result.full_x_mm = -(j.j11 * result.error_u_px -
                         j.j01 * result.error_v_px) / det;
    result.full_y_mm = -(-j.j10 * result.error_u_px +
                          j.j00 * result.error_v_px) / det;
    result.step_x_mm = result.full_x_mm;
    result.step_y_mm = result.full_y_mm;
    limit_vector(result.step_x_mm, result.step_y_mm, cfg.max_step_mm);
    result.vx_mm_s = cfg.gain_per_s * result.full_x_mm;
    result.vy_mm_s = cfg.gain_per_s * result.full_y_mm;
    limit_vector(result.vx_mm_s, result.vy_mm_s, cfg.max_speed_mm_s);
    result.calibrated = true;
    // In the pixel deadband, recommend no oscillatory moves.
    if (result.pixel_centered) {
        result.step_x_mm = result.step_y_mm = 0;
        result.vx_mm_s = result.vy_mm_s = 0;
    }
    return result;
}

}  // namespace pi_material
