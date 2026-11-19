// Derived vision state only: no camera hardware, no UART, no motor commands.
#include <car_interfaces/msg/vision_event.hpp>
#include <car_interfaces/msg/vision_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <memory>
#include <exception>
#include <chrono>
#include <cstdint>
#include <string>
using namespace std::chrono_literals;
namespace cmsg = car_interfaces::msg;
class VisionMonitor final : public rclcpp::Node {
 public:
  VisionMonitor() : Node("car_vision_monitor") {
    stale_ms_ = declare_parameter<int>("event_stale_ms", 2000);
    observation_ms_ = declare_parameter<int>("observation_stale_ms", 1200);
    if (stale_ms_ < 100 || observation_ms_ < 100) throw std::invalid_argument("stale times must be >= 100ms");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
    pub_ = create_publisher<cmsg::VisionStatus>("/car/vision/status", qos);
    sub_ = create_subscription<cmsg::VisionEvent>("/car/vision/event", qos,
      [this](cmsg::VisionEvent::ConstSharedPtr m) { on_event(*m); });
    timer_ = create_wall_timer(200ms, [this]() { publish(); });
    RCLCPP_INFO(get_logger(), "Vision event monitor started (read-only)");
  }
 private:
  using Clock = std::chrono::steady_clock;
  void on_event(const cmsg::VisionEvent& m) {
    last_event_ = Clock::now(); ++received_;
    last_type_=m.event_type; step_=m.step; token_=m.token;
    if (!m.operation.empty()) operation_=m.operation;
    if (!m.station.empty()) station_=m.station;
    if (m.event_type=="SEQ") {
      bool valid=m.task_code.size()==12;
      for(char c:m.task_code) if (c<'0'||c>'9') valid=false;
      if(valid) {task_=m.task_code;qr_valid_=true;}
    }
    if (m.event_type=="OBS" || m.event_type=="MEASUREMENT") {
      if(m.pixel_position_valid) {
        last_observation_=Clock::now(); target_visible_=true;
        u_=m.pixel_u;v_=m.pixel_v;
      } else target_visible_=false;
    }
    if(m.event_type=="LOST" || m.event_type=="ERR")target_visible_=false;
  }
  bool recent(Clock::time_point t,int ms) const {
    return t.time_since_epoch().count()!=0 &&
      (Clock::now()-t)<std::chrono::milliseconds(ms);
  }
  void publish(){
    cmsg::VisionStatus m; m.header.stamp=now();m.header.frame_id="camera_frame";
    m.event_stream_fresh=recent(last_event_,stale_ms_);
    m.observation_fresh=recent(last_observation_,observation_ms_);
    m.target_visible=m.event_stream_fresh && m.observation_fresh && target_visible_;
    m.qr_valid=qr_valid_; m.task_code=task_;m.last_event_type=last_type_;
    m.operation=operation_;m.station=station_;m.step=step_;m.token=token_;
    m.pixel_u=u_;m.pixel_v=v_;m.events_received=received_;
    pub_->publish(m);
  }
  int stale_ms_=2000,observation_ms_=1200;
  Clock::time_point last_event_{},last_observation_{};
  bool target_visible_=false,qr_valid_=false;
  std::string task_,last_type_,operation_,station_;
  std::uint8_t step_=0;std::uint32_t token_=0;std::uint64_t received_=0;
  double u_=0.0,v_=0.0;
  rclcpp::Publisher<cmsg::VisionStatus>::SharedPtr pub_;
  rclcpp::Subscription<cmsg::VisionEvent>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);
 try{rclcpp::spin(std::make_shared<VisionMonitor>());}
 catch(const std::exception&e){RCLCPP_FATAL(rclcpp::get_logger("car_vision_monitor"),"%s",e.what());rclcpp::shutdown();return 1;}
 rclcpp::shutdown();return 0;}
