// No mechanism commands: only measures validity, source staleness and reported faults.
#include <car_interfaces/msg/mechanism_state.hpp>
#include <car_interfaces/msg/mechanism_health.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <memory>
#include <exception>
#include <chrono>
#include <cstdint>
using namespace std::chrono_literals;
namespace cmsg=car_interfaces::msg;
class MechanismMonitor final:public rclcpp::Node{
 public:
  MechanismMonitor():Node("car_mechanism_monitor"){
    stale_ms_=declare_parameter<int>("stale_ms",1500);
    if(stale_ms_<100)throw std::invalid_argument("stale_ms must be >= 100");
    auto qos=rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
    pub_=create_publisher<cmsg::MechanismHealth>("/car/mechanism/health",qos);
    sub_=create_subscription<cmsg::MechanismState>("/car/mechanism/state",qos,
      [this](cmsg::MechanismState::ConstSharedPtr m){sample_=*m;last_=Clock::now();++count_;});
    timer_=create_wall_timer(200ms,[this]{publish();});
    RCLCPP_INFO(get_logger(),"Mechanism status monitor started (read-only)");
  }
 private:
  using Clock=std::chrono::steady_clock;
  void publish(){
    const bool fresh=last_.time_since_epoch().count()!=0 &&
      Clock::now()-last_<std::chrono::milliseconds(stale_ms_);
    cmsg::MechanismHealth h;h.header.stamp=now();h.header.frame_id="base_link";
    h.source_fresh=fresh;h.samples_received=count_;
    if(fresh){
      h.gripper_valid=sample_.gripper_valid;
      h.turntable_valid=sample_.turntable_valid;
      h.lift_valid=sample_.lift_valid;
      h.extension_valid=sample_.extension_valid;
      h.any_position_valid=h.gripper_valid||h.turntable_valid||h.lift_valid||h.extension_valid;
      h.operation_active=sample_.operation_active;
      h.fault_code=sample_.fault_code;h.fault_active=h.fault_code!=0;
    }
    pub_->publish(h);
  }
  int stale_ms_=1500;Clock::time_point last_{};std::uint64_t count_=0;
  cmsg::MechanismState sample_{};
  rclcpp::Publisher<cmsg::MechanismHealth>::SharedPtr pub_;
  rclcpp::Subscription<cmsg::MechanismState>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);
 try{rclcpp::spin(std::make_shared<MechanismMonitor>());}
 catch(const std::exception&e){RCLCPP_FATAL(rclcpp::get_logger("car_mechanism_monitor"),"%s",e.what());rclcpp::shutdown();return 1;}
 rclcpp::shutdown();return 0;}
