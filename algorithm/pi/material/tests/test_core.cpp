#include "pi_material/core.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
bool near(double a, double b) { return std::abs(a - b) < 1e-8; }
}

int main() {
    const auto code = pi_qr::parse_task_code("156+123+516+231");
    assert(code);
    auto steps = pi_material::make_targets(*code);
    // Batch 1: colors 1,5,6; processing rings 1,2,3.
    // Batch 2: colors 5,1,6; processing rings 2,3,1.
    assert(steps[0].batch == 1 && steps[0].color == 1 &&
           steps[0].process_ring == 1 && steps[0].storage_ring == 1);
    assert(steps[1].color == 5 && steps[1].storage_ring == 2);
    assert(steps[2].color == 6 && steps[2].storage_ring == 3);
    assert(steps[3].batch == 2 && steps[3].color == 5 &&
           steps[3].process_ring == 2 && steps[3].storage_ring == 2);
    assert(steps[4].color == 1 && steps[4].process_ring == 3 &&
           steps[4].storage_ring == 1);
    assert(steps[5].color == 6 && steps[5].storage_ring == 3);

    pi_material::StableSpot stable(3, 4.0);
    assert(!stable.update(pi_material::Spot{100, 120, 25, 400}));
    assert(!stable.update(pi_material::Spot{101, 120, 25, 400}));
    assert(stable.update(pi_material::Spot{102, 121, 25, 400}));
    assert(!stable.update(pi_material::Spot{180, 90, 25, 400}));
    assert(stable.count() == 1);
    assert(!stable.update(std::nullopt));
    assert(stable.count() == 0);

    pi_material::ServoSettings settings;
    settings.ref_u_px = 160;
    settings.ref_v_px = 190;
    auto p = pi_material::Spot{180, 200, 30, 600};
    auto uncal = pi_material::calculate_servo(p, settings);
    assert(near(uncal.error_u_px, 20));
    assert(near(uncal.error_v_px, 10));
    assert(!uncal.calibrated);
    assert(!uncal.pixel_centered);

    // Physical calibration example only, do not reuse these numbers on robot.
    settings.reference_taught = true;
    settings.jacobian = pi_material::Jacobian{2, 0, 0, 2};
    auto v = pi_material::calculate_servo(p, settings);
    assert(v.calibrated);
    assert(near(v.full_x_mm, -10) && near(v.full_y_mm, -5));
    assert(near(std::hypot(v.step_x_mm, v.step_y_mm), 5));
    assert(v.vx_mm_s < 0 && v.vy_mm_s < 0);
    auto aligned = pi_material::calculate_servo({160, 190, 30, 500}, settings);
    assert(aligned.pixel_centered && aligned.calibrated);
    assert(near(aligned.vx_mm_s, 0) && near(aligned.vy_mm_s, 0));

    settings.jacobian = pi_material::Jacobian{1, 2, 2, 4};
    assert(!pi_material::valid_jacobian(*settings.jacobian));
    bool rejected = false;
    try { pi_material::calculate_servo(p, settings); }
    catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
    std::cout << "material sequence + stable detection + IBVS Jacobian tests OK\n";
}
