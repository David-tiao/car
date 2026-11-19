// STM32 race status is authoritative. This node aggregates signals and never publishes commands.
#include "car_mission/core.hpp"
#include <car_interfaces/msg/command_feedback.hpp>
#include <car_interfaces/msg/link_status.hpp>
#include <car_interfaces/msg/mechanism_health.hpp>
#include <car_interfaces/msg/mission_state.hpp>
#include <car_interfaces/msg/mission_summary.hpp>
#include <car_interfaces/msg/vision_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <memory>
#include <exception>
#include <chrono>
#include <cstdint>
#include <string>
using namespace std::chrono_literals;
namespace cmsg=car_interfaces::msg;
class MissionMonitor final: public rclcpp::Node {
 public:
  MissionMonitor():Node("car_mission_monitor"){
    mission_stale_ms_=declare_parameter<int>("mission_stale_ms",2000);
    odom_stale_ms_=declare_parameter<int>("odom_stale_ms",1000);
    if(mission_stale_ms_<100||odom_stale_ms_<100)throw std::invalid_argument("invalid stale period");
    auto qos=rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
    pub_=create_publisher<cmsg::MissionSummary>("/car/mission/summary",qos);
    alert_pub_=create_publisher<std_msgs::msg::String>("/car/mission/alerts",qos);
    link_sub_=create_subscription<cmsg::LinkStatus>("/car/stm32/link_status",qos,
      [this](cmsg::LinkStatus::ConstSharedPtr m){link_=*m;ever_link_=true;last_link_=Clock::now();});
    mission_sub_=create_subscription<cmsg::MissionState>("/car/mission/state",qos,
      [this](cmsg::MissionState::ConstSharedPtr m){mission_=*m;last_mission_=Clock::now();});
    vision_sub_=create_subscription<cmsg::VisionStatus>("/car/vision/status",qos,
      [this](cmsg::VisionStatus::ConstSharedPtr m){vision_=*m;last_vision_=Clock::now();});
    mechanism_sub_=create_subscription<cmsg::MechanismHealth>("/car/mechanism/health",qos,
      [this](cmsg::MechanismHealth::ConstSharedPtr m){mechanism_=*m;last_mechanism_=Clock::now();});
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/odom",qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr){last_odom_=Clock::now();});
    cmd_sub_=create_subscription<cmsg::CommandFeedback>("/car/stm32/command_feedback",qos,
      [this](cmsg::CommandFeedback::ConstSharedPtr m){
        if(m->event_type==0x96)return; // CONTROL_STATUS has no cmd_id
        last_cmd_id_=m->cmd_id;
        switch(m->event_type){
          case 0x90:last_command_event_="ACK";break;
          case 0x91:last_command_event_="REJECTED";break;
          case 0x92:last_command_event_="STARTED";break;
          case 0x93:last_command_event_="PROGRESS";break;
          case 0x94:last_command_event_="COMPLETED";break;
          case 0x95:last_command_event_="FAILED";break;
          default:break;
        }
      });
    timer_=create_wall_timer(200ms,[this]{publish();});
    RCLCPP_INFO(get_logger(),"Read-only mission aggregation started: no motor commands");
  }
 private:
  using Clock=std::chrono::steady_clock;
  bool recent(Clock::time_point t,int ms)const{
    return t.time_since_epoch().count()!=0 && Clock::now()-t<std::chrono::milliseconds(ms);
  }
  void publish(){
    cmsg::MissionSummary m;m.header.stamp=now();m.header.frame_id="base_link";
    const bool link_recent=recent(last_link_,1200);
    m.link_connected=link_recent&&link_.connected;
    m.heartbeat_fresh=link_recent&&link_.heartbeat_fresh;
    m.mission_fresh=recent(last_mission_,mission_stale_ms_);
    m.vision_fresh=recent(last_vision_,1200)&&vision_.event_stream_fresh;
    m.mechanism_fresh=recent(last_mechanism_,1200)&&mechanism_.source_fresh;
    m.odom_fresh=recent(last_odom_,odom_stale_ms_);
    m.race_state=m.mission_fresh?mission_.status_code:0;
    m.action_index=m.mission_fresh?mission_.action_index:0;
    m.plan_index=m.mission_fresh?mission_.plan_index:0;
    m.plan_done=m.mission_fresh&&mission_.plan_done;
    m.control_owner=link_recent?link_.control_owner:0;
    const bool race_fault=m.mission_fresh && mission_.error_code!=0;
    const bool mech_fault=m.mechanism_fresh && mechanism_.fault_active;
    m.fault_active=race_fault||mech_fault;
    m.fault_code=race_fault?mission_.error_code:(mech_fault?mechanism_.fault_code:0);
    car_mission::Inputs i;
    i.ever_link=ever_link_;i.link_alive=m.link_connected&&m.heartbeat_fresh;
    i.mission_fresh=m.mission_fresh;
    i.mission_semantics_known=m.mission_fresh&&mission_.race_state_semantics_known;
    i.race_state=m.race_state;i.plan_done=m.plan_done;
    i.mission_fault=race_fault;i.mechanism_fault=mech_fault;
    m.state=car_mission::classify(i);
    if(m.state=="RUNNING"&&!m.odom_fresh)m.detail="STM32 reports running, but odometry is stale";
    else if(m.state=="COMPLETE")m.detail="Race done and STM32 plan_done confirmed";
    else if(m.state=="ROUTE_DONE")m.detail="Race route ended; full competition plan not confirmed";
    else if(m.state=="FAULT")m.detail="STM32 or mechanism reported a fault";
    else if(m.state=="UNKNOWN_MODE")m.detail="STM32 race state semantics not known";
    else m.detail=m.state;
    m.last_cmd_id=last_cmd_id_;m.last_command_event=last_command_event_;
    pub_->publish(m);
    if(last_state_!=m.state){
      std_msgs::msg::String alert;alert.data="Mission monitor state: "+m.state+"; "+m.detail;
      alert_pub_->publish(alert);last_state_=m.state;
      RCLCPP_INFO(get_logger(),"Mission monitor: %s",m.state.c_str());
    }
  }
  int mission_stale_ms_=2000,odom_stale_ms_=1000;
  bool ever_link_=false;
  Clock::time_point last_link_{},last_mission_{},last_vision_{},last_mechanism_{},last_odom_{};
  cmsg::LinkStatus link_{};cmsg::MissionState mission_{};
  cmsg::VisionStatus vision_{};cmsg::MechanismHealth mechanism_{};
  std::uint32_t last_cmd_id_=0;std::string last_command_event_,last_state_;
  rclcpp::Publisher<cmsg::MissionSummary>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alert_pub_;
  rclcpp::Subscription<cmsg::LinkStatus>::SharedPtr link_sub_;
  rclcpp::Subscription<cmsg::MissionState>::SharedPtr mission_sub_;
  rclcpp::Subscription<cmsg::VisionStatus>::SharedPtr vision_sub_;
  rclcpp::Subscription<cmsg::MechanismHealth>::SharedPtr mechanism_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<cmsg::CommandFeedback>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);
 try{rclcpp::spin(std::make_shared<MissionMonitor>());}
 catch(const std::exception&e){RCLCPP_FATAL(rclcpp::get_logger("car_mission_monitor"),"%s",e.what());rclcpp::shutdown();return 1;}
 rclcpp::shutdown();return 0;}
