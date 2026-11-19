#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace car_serial_link {

struct MockPose {
    double x,y,yaw;
};

struct MockMecanumState {
    double x=0.4,y=0.4,yaw=0,vx=0,vy=0,wz=0;
    double cmd_vx=0,cmd_vy=0,cmd_wz=0;
    double target_x=0,target_y=0,target_yaw=0;
    std::array<double,4> wheel_target{},wheel_actual{};
    std::size_t waypoint=0;
    bool reached=false;
};

class MockMecanumModel {
    static constexpr double PI=3.14159265358979323846;
    static constexpr double L=0.093+0.110;
    static constexpr double PPM=440.0/(2*PI*0.04);

    MockMecanumState st_{};
    std::vector<MockPose> path_;
    bool active_=false,complete_=false;

    static double wrap(double x){
        return std::atan2(std::sin(x),std::cos(x));
    }

public:
    const MockMecanumState &state() const {
        return st_;
    }

    void stop(){
        active_=false;
        complete_=false;
        path_.clear();
        st_.reached=false;
    }

    void setRoute(const std::vector<MockPose> &points){
        if(points.size()<2||points.size()>250){
            stop();return;
        }

        bool changed=path_.size()!=points.size();

        if(!changed){
            for(std::size_t i=0;i<points.size();++i){
                if(std::hypot(
                    path_[i].x-points[i].x,
                    path_[i].y-points[i].y
                )>.002){
                    changed=true;
                    break;
                }
            }
        }

        if(!changed)return;

        path_=points;
        active_=true;
        complete_=false;
        st_=MockMecanumState{};

        // TEST 初始化到网页规划路径的起点。
        st_.x=path_.front().x;
        st_.y=path_.front().y;
        st_.yaw=std::atan2(
            path_[1].y-st_.y,
            path_[1].x-st_.x
        );
        st_.waypoint=1;
    }

    void step(double dt){
        if(dt<=0||dt>0.1)return;

        double wx=0,wy=0,wz=0;

        if(active_ && st_.waypoint<path_.size()){
            auto target=path_[st_.waypoint];
            double dx=target.x-st_.x;
            double dy=target.y-st_.y;
            double d=std::hypot(dx,dy);

            if(d<0.045){
                ++st_.waypoint;
                if(st_.waypoint>=path_.size()){
                    active_=false;
                    complete_=true;
                }else{
                    target=path_[st_.waypoint];
                    dx=target.x-st_.x;
                    dy=target.y-st_.y;
                    d=std::hypot(dx,dy);
                }
            }

            if(active_){
                const double desired_yaw=std::atan2(dy,dx);
                const double err=wrap(desired_yaw-st_.yaw);
                const double ae=std::abs(err);

                const double yaw_scale=
                    ae<=0.035?1.0:
                    (ae>=0.18?0.2:
                    1.0-(ae-0.035)*0.8/0.145);

                const double vmax=
                    std::min(0.23,std::sqrt(2*0.5*d))*yaw_scale;

                wx=std::clamp(1.6*dx,-vmax,vmax);
                wy=std::clamp(1.6*dy,-vmax,vmax);

                const double norm=std::hypot(wx,wy);
                if(norm>vmax&&norm>0){
                    wx*=vmax/norm;
                    wy*=vmax/norm;
                }

                wz=std::clamp(3.2*err,-0.5,0.5);

                st_.target_x=target.x;
                st_.target_y=target.y;
                st_.target_yaw=desired_yaw;
            }
        }

        // 世界坐标速度 -> 车体坐标速度。
        const double c=std::cos(st_.yaw);
        const double s=std::sin(st_.yaw);

        st_.cmd_vx=wx*c+wy*s;
        st_.cmd_vy=-wx*s+wy*c;
        st_.cmd_wz=wz;

        // 对齐 STM32F4 X 型麦克纳姆轮解算。
        // 轮序：FL FR RL RR。
        const std::array<double,4> cmd={{
            (st_.cmd_vx-st_.cmd_vy-L*wz)*PPM,
            (st_.cmd_vx+st_.cmd_vy+L*wz)*PPM,
            (st_.cmd_vx+st_.cmd_vy-L*wz)*PPM,
            (st_.cmd_vx-st_.cmd_vy+L*wz)*PPM
        }};

        const double alpha=1-std::exp(-dt/0.15);

        for(std::size_t i=0;i<4;++i){
            st_.wheel_target[i]+=
                std::clamp(
                    cmd[i]-st_.wheel_target[i],
                    -4000*dt,4000*dt
                );

            st_.wheel_actual[i]+=alpha*(
                st_.wheel_target[i]-st_.wheel_actual[i]
            );
        }

        // 四轮实际速度 -> 底盘实际速度。
        const auto &p=st_.wheel_actual;
        const double fl=p[0]/PPM;
        const double fr=p[1]/PPM;
        const double rl=p[2]/PPM;
        const double rr=p[3]/PPM;

        st_.vx=(fl+fr+rl+rr)/4;
        st_.vy=(-fl+fr+rl-rr)/4;
        st_.wz=(-fl+fr-rl+rr)/(4*L);

        // 车体速度 -> 世界坐标里程计积分。
        st_.x+=(st_.vx*c-st_.vy*s)*dt;
        st_.y+=(st_.vx*s+st_.vy*c)*dt;
        st_.yaw=wrap(st_.yaw+st_.wz*dt);

        st_.reached=complete_
            &&std::hypot(st_.vx,st_.vy)<0.04
            &&std::abs(st_.wz)<0.08;
    }
};
}
