#pragma once
// TEST ONLY: approximate motor dynamics with STM32F4 kinematics.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace car_serial_link {

struct MockPose {
  double x, y, yaw;
};

struct MockMecanumState {
  double x=0.40, y=0.40, yaw=0;
  double vx=0, vy=0, wz=0;
  double cmd_vx=0, cmd_vy=0, cmd_wz=0;
  double target_x=0, target_y=0, target_yaw=0;
  std::array<double,4> wheel_target{};
  std::array<double,4> wheel_actual{};
  std::size_t waypoint=0;
  bool reached=false;
};

class MockMecanumModel {
public:
  static constexpr double PI=3.14159265358979323846;
  static constexpr double L=0.093+0.110;
  static constexpr double PPM=440.0/(2.0*PI*0.04);
  static constexpr double RAMP=4000.0;

  const MockMecanumState& state() const {
    return state_;
  }

  void step(double dt) {
    if (!(dt>0 && dt<=0.1)) return;

    const auto &goal=route_[state_.waypoint];
    state_.target_x=goal.x;
    state_.target_y=goal.y;
    state_.target_yaw=goal.yaw;

    const double ex=goal.x-state_.x;
    const double ey=goal.y-state_.y;
    const double dist=std::hypot(ex,ey);
    const double yaw_error=wrap(goal.yaw-state_.yaw);

    // Position-loop arrival detection.
    state_.reached =
      dist<0.025 &&
      std::abs(yaw_error)<0.06 &&
      std::hypot(state_.vx,state_.vy)<0.04 &&
      std::abs(state_.wz)<0.08;

    if (state_.reached) {
      if (++hold_ticks_>=8) {
        state_.waypoint=(state_.waypoint+1)%route_.size();
        hold_ticks_=0;
      }
    } else {
      hold_ticks_=0;
    }

    // Simplified STM32 position loop, using its proportional gains.
    double vx_world=std::clamp(1.6*ex,-0.28,0.28);
    double vy_world=std::clamp(1.6*ey,-0.28,0.28);

    double limit=std::min(0.28,std::sqrt(2.0*0.50*dist));
    const double ae=std::abs(yaw_error);

    // Yaw-first translation attenuation from STM32 chassis code.
    const double yaw_scale =
      ae<=0.035 ? 1.0 :
      ae>=0.18 ? 0.20 :
      1.0-(ae-0.035)*(0.80/0.145);

    limit*=yaw_scale;

    const double v=std::hypot(vx_world,vy_world);
    if (v>limit && v>0) {
      vx_world*=limit/v;
      vy_world*=limit/v;
    }

    double wz=std::clamp(3.2*yaw_error,-0.8,0.8);
    const double wlimit=std::sqrt(2.0*1.80*ae);
    wz=std::clamp(wz,-wlimit,wlimit);

    if (ae<0.05) wz=0;
    if (dist<0.025) vx_world=vy_world=0;

    // World frame -> body frame.
    const double c=std::cos(state_.yaw);
    const double s=std::sin(state_.yaw);

    state_.cmd_vx=c*vx_world+s*vy_world;
    state_.cmd_vy=-s*vx_world+c*vy_world;
    state_.cmd_wz=wz;

    // STM32 X-layout mecanum inverse kinematics.
    const std::array<double,4> demand={{
      (state_.cmd_vx-state_.cmd_vy-L*wz)*PPM,
      (state_.cmd_vx+state_.cmd_vy+L*wz)*PPM,
      (state_.cmd_vx+state_.cmd_vy-L*wz)*PPM,
      (state_.cmd_vx-state_.cmd_vy+L*wz)*PPM
    }};

    // STM32-compatible target ramp.
    // Actual wheel response is an approximate first-order model.
    const double max_change=RAMP*dt;
    const double alpha=1.0-std::exp(-dt/0.15);

    for (std::size_t i=0;i<4;++i) {
      const double delta=std::clamp(
        demand[i]-state_.wheel_target[i],
        -max_change,max_change
      );

      state_.wheel_target[i]+=delta;
      state_.wheel_actual[i]+=alpha*(
        state_.wheel_target[i]-state_.wheel_actual[i]
      );
    }

    // STM32 mecanum forward kinematics.
    const auto &p=state_.wheel_actual;

    const double fl=p[0]/PPM;
    const double fr=p[1]/PPM;
    const double rl=p[2]/PPM;
    const double rr=p[3]/PPM;

    state_.vx=(fl+fr+rl+rr)/4;
    state_.vy=(-fl+fr+rl-rr)/4;
    state_.wz=(-fl+fr-rl+rr)/(4*L);

    // Body velocity -> world odometry.
    state_.x+=(state_.vx*c-state_.vy*s)*dt;
    state_.y+=(state_.vx*s+state_.vy*c)*dt;
    state_.yaw=wrap(state_.yaw+state_.wz*dt);
  }

private:
  static double wrap(double a) {
    return std::atan2(std::sin(a),std::cos(a));
  }

  // Temporary TEST waypoints, not the browser's planned route.
  inline static constexpr std::array<MockPose,4> route_={{
    {1.90,0.40,0},
    {1.90,1.90,PI/2},
    {0.40,1.90,PI},
    {0.40,0.40,-PI/2}
  }};

  MockMecanumState state_{};
  int hold_ticks_=0;
};

} // namespace car_serial_link
