// CAR-LINK V1 binary UART -> ROS2. Deliberately receive-only: NO motor commands.
#include "car_serial_link/protocol.hpp"
#include <car_interfaces/msg/chassis_telemetry.hpp>
#include <car_interfaces/msg/wheel_telemetry.hpp>
#include <car_interfaces/msg/mission_state.hpp>
#include <car_interfaces/msg/vision_event.hpp>
#include <car_interfaces/msg/mechanism_state.hpp>
#include <car_interfaces/msg/power_telemetry.hpp>
#include <car_interfaces/msg/command_feedback.hpp>
#include <car_interfaces/msg/link_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <utility>
using namespace std::chrono_literals;
namespace cmsg = car_interfaces::msg;
namespace proto = car_serial_link;

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
using Clock=std::chrono::steady_clock;
std::string station_name(std::uint8_t v) {
  switch(v){case 1:return "RAW";case 2:return "ROUGH";case 3:return "STORE";case 4:return "HOME";default:return "UNKNOWN";}
}
std::string operation_name(std::uint8_t v) {
  switch(v){case 1:return "PICK";case 2:return "PLACE";case 3:return "STACK";case 4:return "HOME";default:return "UNKNOWN";}
}
std::string event_name(std::uint8_t v) {
  switch(v){case 1:return "SEQ";case 2:return "OBS";case 3:return "LOST";case 4:return "STATE";case 5:return "RESULT_OK";case 6:return "ERR";default:return "UNKNOWN";}
}
std::array<double,4> quaternion(double roll,double pitch,double yaw) {
  const double cy=std::cos(yaw/2),sy=std::sin(yaw/2);
  const double cp=std::cos(pitch/2),sp=std::sin(pitch/2);
  const double cr=std::cos(roll/2),sr=std::sin(roll/2);
  return {{sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy}};
}
}

class SerialBridge final : public rclcpp::Node {
public:
 SerialBridge():Node("car_serial_bridge"){
  port_=declare_parameter<std::string>("port","/tmp/car_link_demo_uart");
  baud_=declare_parameter<int>("baud",115200);
  stale_ms_=declare_parameter<int>("stale_ms",500);
  wheel_ppm_=declare_parameter<double>("pulses_per_meter",1750.0);
  wheel_ppr_=declare_parameter<double>("wheel_ppr",440.0);
  wheelbase_half_=declare_parameter<double>("wheelbase_half_m",0.093);
  track_half_=declare_parameter<double>("track_half_m",0.110);
  publish_joint_=declare_parameter<bool>("publish_joint_states",true);
  if(port_.empty()||baud_!=115200||stale_ms_<100||wheel_ppm_<=0||wheel_ppr_<=0||wheelbase_half_<=0||track_half_<=0){
    throw std::invalid_argument("invalid CAR-LINK parameters (baud fixed at 115200)");
  }
  const auto qos=rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
  chassis_pub_=create_publisher<cmsg::ChassisTelemetry>("/car/chassis/telemetry",qos);
  wheel_pub_=create_publisher<cmsg::WheelTelemetry>("/car/chassis/wheels",qos);
  odom_pub_=create_publisher<nav_msgs::msg::Odometry>("/odom",qos);
  if(publish_joint_)joint_pub_=create_publisher<sensor_msgs::msg::JointState>("/joint_states",qos);
  imu_pub_=create_publisher<sensor_msgs::msg::Imu>("/imu/data",qos);
  mechanism_pub_=create_publisher<cmsg::MechanismState>("/car/mechanism/state",qos);
  power_pub_=create_publisher<cmsg::PowerTelemetry>("/car/power",qos);
  target_twist_pub_=create_publisher<geometry_msgs::msg::TwistStamped>("/car/chassis/target_twist",qos);
  target_pose_pub_=create_publisher<geometry_msgs::msg::PoseStamped>("/car/chassis/target_pose",qos);
  vision_pub_=create_publisher<cmsg::VisionEvent>("/car/vision/event",qos);
  mission_pub_=create_publisher<cmsg::MissionState>("/car/mission/state",qos);
  command_pub_=create_publisher<cmsg::CommandFeedback>("/car/stm32/command_feedback",qos);
  link_pub_=create_publisher<cmsg::LinkStatus>("/car/stm32/link_status",qos);
  fault_pub_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/car/stm32/fault",qos);
  rx_timer_=create_wall_timer(10ms,[this]{poll_serial();});
  status_timer_=create_wall_timer(500ms,[this]{publish_link_status();});
  RCLCPP_WARN(get_logger(),"CAR-LINK V1 RECEIVE ONLY: no command transmitter or motor control exists");
 }
 ~SerialBridge() override{close_port();}
private:
  void close_port(){if(fd_>=0){::close(fd_);fd_=-1;}parser_.reset();}
  bool open_port(){
    fd_=::open(port_.c_str(),O_RDONLY|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);
    if(fd_<0)return false;
    termios cfg{};
    if(::tcgetattr(fd_,&cfg)<0){close_port();return false;}
    ::cfmakeraw(&cfg);
    ::cfsetispeed(&cfg,B115200);::cfsetospeed(&cfg,B115200);
    cfg.c_cflag=(cfg.c_cflag&~CSIZE)|CS8|CLOCAL|CREAD;
    cfg.c_cflag&=~(PARENB|CSTOPB|CRTSCTS);
    cfg.c_cc[VMIN]=0;cfg.c_cc[VTIME]=0;
    if(::tcsetattr(fd_,TCSANOW,&cfg)<0){close_port();return false;}
    RCLCPP_INFO(get_logger(),"Connected to CAR-LINK read-only port: %s",port_.c_str());
    return true;
  }
  void poll_serial(){
    if(fd_<0){if(Clock::now()<next_connect_)return;
      next_connect_=Clock::now()+2s;
      if(!open_port())RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Port not ready: %s",port_.c_str());
      return;
    }
    std::uint8_t buf[1024];
    for(int i=0;i<8;++i){
      const auto n=::read(fd_,buf,sizeof(buf));
      if(n>0){std::vector<proto::Frame> frames;parser_.feed(buf,static_cast<std::size_t>(n),frames);
        for(const auto &f:frames)handle(f);
        continue;
      }
      if(n==0 || (n<0&&(errno==EAGAIN||errno==EWOULDBLOCK||errno==EINTR)))break;
      RCLCPP_WARN(get_logger(),"Serial read failed: %s",std::strerror(errno));
      close_port();next_connect_=Clock::now()+2s;break;
    }
  }
  std_msgs::msg::Header header(const std::string &frame) {
    std_msgs::msg::Header h;h.stamp=now();h.frame_id=frame;return h;
  }
  bool is_recent(const Clock::time_point &t,int timeout_ms)const{
    return t!=Clock::time_point{} && Clock::now()-t<std::chrono::milliseconds(timeout_ms);
  }
  void handle(const proto::Frame &f){
    const auto type=static_cast<proto::Type>(f.type);
    proto::Reader r(f.payload);
    bool ok=false;
    switch(type){
      case proto::Type::CHASSIS: ok=on_chassis(f,r);break;
      case proto::Type::WHEELS: ok=on_wheels(f,r);break;
      case proto::Type::IMU: ok=on_imu(f,r);break;
      case proto::Type::MECHANISM: ok=on_mechanism(f,r);break;
      case proto::Type::POWER: ok=on_power(f,r);break;
      case proto::Type::VISION_MEASUREMENT: ok=on_vision_measure(f,r);break;
      case proto::Type::CHASSIS_TARGET: ok=on_target(f,r);break;
      case proto::Type::HELLO: ok=on_hello(r);break;
      case proto::Type::HEARTBEAT: ok=on_heartbeat(r);break;
      case proto::Type::LINK_STATUS: ok=on_peer_link(r);break;
      case proto::Type::MISSION_EVENT: ok=on_mission(f,r);break;
      case proto::Type::VISION_EVENT: ok=on_vision_event(f,r);break;
      case proto::Type::FAULT_EVENT: ok=on_fault(f,r);break;
      case proto::Type::COMMAND_ACK:case proto::Type::COMMAND_REJECTED:
      case proto::Type::COMMAND_STARTED:case proto::Type::COMMAND_PROGRESS:
      case proto::Type::COMMAND_COMPLETED:case proto::Type::COMMAND_FAILED:
      case proto::Type::CONTROL_STATUS:ok=on_command_feedback(f,r);break;
      default: RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,"Unsupported type 0x%02X (commands disabled)",f.type);return;
    }
    if(!ok || !r.done()){
      ++payload_errors_;
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,"Invalid payload type=0x%02X len=%zu",f.type,f.payload.size());
    }
  }
  bool on_chassis(const proto::Frame &f,proto::Reader &r){
    std::int32_t x,y,yaw,vx,vy,wz;std::uint8_t validity;
    if(!(r.i32(x)&&r.i32(y)&&r.i32(yaw)&&r.i32(vx)&&r.i32(vy)&&r.i32(wz)&&r.u8(validity)))return false;
    if(!r.done() || !(validity&0x01))return r.done(); // invalid sample is intentionally not published
    const auto h=header("odom");const double heading=yaw*0.001;
    auto od=nav_msgs::msg::Odometry();od.header=h;od.child_frame_id="base_link";
    od.pose.pose.position.x=x*0.001;od.pose.pose.position.y=y*0.001;
    od.pose.pose.orientation.z=std::sin(heading/2);od.pose.pose.orientation.w=std::cos(heading/2);
    od.twist.twist.linear.x=vx*0.001;od.twist.twist.linear.y=vy*0.001;od.twist.twist.angular.z=wz*0.001;
    // Uncalibrated odometry: large nominal variances until real calibration.
    for(int i=0;i<6;++i){od.pose.covariance[i*6+i]=100.;od.twist.covariance[i*6+i]=100.;}
    odom_pub_->publish(od);
    cmsg::ChassisTelemetry ch;ch.header=h;ch.source_time_ms=f.timestamp_ms;
    ch.x_m=od.pose.pose.position.x;ch.y_m=od.pose.pose.position.y;ch.yaw_rad=heading;
    ch.vx_mps=od.twist.twist.linear.x;ch.vy_mps=od.twist.twist.linear.y;ch.wz_radps=od.twist.twist.angular.z;
    ch.target_reached=(validity&0x02)!=0;
    const bool have_target=target_seen_ && is_recent(last_target_,1000);
    ch.wheel_estimated_wz_radps=is_recent(last_wheels_,1000)?wheel_wz_:kNaN;
    ch.chassis_mode=have_target?target_mode_:0;
    ch.target_x_m=have_target?target_x_:kNaN;
    ch.target_y_m=have_target?target_y_:kNaN;
    ch.target_yaw_rad=have_target?target_yaw_:kNaN;
    ch.target_wz_radps=have_target?target_wz_:kNaN;
    ch.target_speed_mps=have_target?std::hypot(target_vx_,target_vy_):kNaN;
    ch.target_linear_components_valid=have_target; // the wire target has both components
    ch.x_target_error_m=have_target?target_x_-ch.x_m:kNaN;
    ch.y_target_error_m=have_target?target_y_-ch.y_m:kNaN;
    ch.yaw_target_error_rad=have_target?std::atan2(std::sin(target_yaw_-heading),std::cos(target_yaw_-heading)):kNaN;
    ch.lateral_target_error_m=have_target?(-std::sin(heading)*(target_x_-ch.x_m)+std::cos(heading)*(target_y_-ch.y_m)):kNaN;
    ch.driver_voltage_v=is_recent(last_power_,1500)&&driver_valid_?driver_v_:kNaN;
    ch.board_voltage_v=is_recent(last_power_,1500)&&board_valid_?board_v_:kNaN;
    ch.travelled_distance_m=kNaN; // not present in CAR-LINK CHASSIS V1
    ch.imu_valid=is_recent(last_imu_,1000)&&imu_valid_;
    ch.imu_yaw_rad=ch.imu_valid?imu_yaw_:kNaN;
    chassis_pub_->publish(ch);
    return true;
  }
  bool on_wheels(const proto::Frame &f,proto::Reader &r){
    std::array<std::int32_t,4> target{},actual{};std::array<std::int16_t,4> pid{};std::uint8_t validity=0;
    for(auto &x:target)if(!r.i32(x))return false;
    for(auto &x:actual)if(!r.i32(x))return false;
    for(auto &x:pid)if(!r.i16(x))return false;
    if(!r.u8(validity)||!r.done())return false;
    auto msg=cmsg::WheelTelemetry();msg.header=header("base_link");msg.source_time_ms=f.timestamp_ms;
    for(int i=0;i<4;++i){
      msg.target_pulse_per_s[i]=target[i];msg.actual_pulse_per_s[i]=actual[i];
      msg.target_mps[i]=target[i]/wheel_ppm_;msg.actual_mps[i]=actual[i]/wheel_ppm_;
    }
    msg.per_wheel_pid_output_valid=(validity&0x01)!=0;
    msg.encoder_counts_valid=false; // no cumulative counts on wire in V1
    if(msg.per_wheel_pid_output_valid){
      double mx=0;for(const auto v:pid)mx=std::max(mx,std::abs(static_cast<double>(v)));
      msg.max_abs_pid_output=mx;
    }else msg.max_abs_pid_output=kNaN;
    wheel_wz_=(-msg.actual_mps[0]+msg.actual_mps[1]-msg.actual_mps[2]+msg.actual_mps[3])/(4*(wheelbase_half_+track_half_));
    last_wheels_=Clock::now();wheel_pub_->publish(msg);
    if(joint_pub_){
      sensor_msgs::msg::JointState joints;joints.header=header("base_link");
      joints.name={"wheel_fl_joint","wheel_fr_joint","wheel_rl_joint","wheel_rr_joint"};
      for(int i=0;i<4;++i)joints.velocity.push_back(actual[i]*(2*kPi/wheel_ppr_));
      joint_pub_->publish(joints);
    }
    return true;
  }
  bool on_imu(const proto::Frame &f,proto::Reader &r){
    std::int32_t roll,pitch,yaw;std::uint8_t valid;
    if(!(r.i32(roll)&&r.i32(pitch)&&r.i32(yaw)&&r.u8(valid)&&r.done()))return false;
    imu_valid_=(valid&0x01)!=0;imu_yaw_=yaw*0.001;last_imu_=Clock::now();
    sensor_msgs::msg::Imu msg;msg.header=header("imu_link");
    if(imu_valid_){auto q=quaternion(roll*.001,pitch*.001,yaw*.001);
      msg.orientation.x=q[0];msg.orientation.y=q[1];msg.orientation.z=q[2];msg.orientation.w=q[3];
    }else msg.orientation.w=1.0;
    // All-zero orientation covariance = unknown covariance when a valid attitude is present.
    // -1 only when orientation is unavailable. Gyro/acceleration are always unavailable.
    if(!imu_valid_)msg.orientation_covariance[0]=-1.0;
    msg.angular_velocity_covariance[0]=-1.0;
    msg.linear_acceleration_covariance[0]=-1.0;
    imu_pub_->publish(msg);(void)f;return true;
  }
  bool on_mechanism(const proto::Frame &f,proto::Reader &r){
    std::int32_t grip,turn,lift,ext;std::uint8_t valid,active;std::uint16_t fault;
    if(!(r.i32(grip)&&r.i32(turn)&&r.i32(lift)&&r.i32(ext)&&r.u8(valid)&&r.u8(active)&&r.u16(fault)&&r.done()))return false;
    cmsg::MechanismState m;m.header=header("base_link");m.source_time_ms=f.timestamp_ms;
    m.gripper_valid=valid&1;m.turntable_valid=valid&2;m.lift_valid=valid&4;m.extension_valid=valid&8;
    m.gripper_angle_deg=m.gripper_valid?grip*.001*180./kPi:kNaN;
    m.turntable_angle_deg=m.turntable_valid?turn*.001*180./kPi:kNaN;
    m.lift_height_mm=m.lift_valid?lift:kNaN;
    m.extension_mm=m.extension_valid?ext:kNaN;
    m.operation_active=active!=0;m.fault_code=fault;mechanism_pub_->publish(m);return true;
  }
  bool on_power(const proto::Frame &f,proto::Reader &r){
    std::uint16_t driver,board;std::uint8_t valid;
    if(!(r.u16(driver)&&r.u16(board)&&r.u8(valid)&&r.done()))return false;
    driver_valid_=valid&1;board_valid_=valid&2;
    driver_v_=driver*.001;board_v_=board*.001;last_power_=Clock::now();
    cmsg::PowerTelemetry p;p.header=header("base_link");p.source_time_ms=f.timestamp_ms;
    p.driver_voltage_valid=driver_valid_;p.board_voltage_valid=board_valid_;
    p.driver_voltage_v=driver_valid_?driver_v_:kNaN;
    p.board_voltage_v=board_valid_?board_v_:kNaN;
    power_pub_->publish(p);return true;
  }
  bool on_vision_measure(const proto::Frame &f,proto::Reader &r){
    std::uint8_t type,valid;std::int16_t u,v;
    if(!(r.u8(type)&&r.i16(u)&&r.i16(v)&&r.u8(valid)&&r.done()))return false;
    cmsg::VisionEvent m;m.header=header("camera_frame");m.event_type="MEASUREMENT";
    m.state_name="target_type="+std::to_string(type);m.pixel_u=u;m.pixel_v=v;
    m.pixel_position_valid=(valid&1)!=0;m.metric_position_valid=false;
    vision_pub_->publish(m);(void)f;return true;
  }
  bool on_target(const proto::Frame &f,proto::Reader &r){
    std::int32_t x,y,yaw,vx,vy,wz;std::uint8_t mode;
    if(!(r.i32(x)&&r.i32(y)&&r.i32(yaw)&&r.i32(vx)&&r.i32(vy)&&r.i32(wz)&&r.u8(mode)&&r.done()))return false;
    target_seen_=true;last_target_=Clock::now();target_x_=x*.001;target_y_=y*.001;target_yaw_=yaw*.001;
    target_vx_=vx*.001;target_vy_=vy*.001;target_wz_=wz*.001;target_mode_=mode;
    geometry_msgs::msg::TwistStamped vel;vel.header=header("base_link");
    vel.twist.linear.x=target_vx_;vel.twist.linear.y=target_vy_;vel.twist.angular.z=target_wz_;
    target_twist_pub_->publish(vel);
    geometry_msgs::msg::PoseStamped pose;pose.header=header("odom");pose.pose.position.x=target_x_;
    pose.pose.position.y=target_y_;pose.pose.orientation.z=std::sin(target_yaw_/2);
    pose.pose.orientation.w=std::cos(target_yaw_/2);target_pose_pub_->publish(pose);(void)f;return true;
  }
  bool on_hello(proto::Reader &r){
    std::uint8_t device,minor;std::uint32_t session,capabilities;
    if(!(r.u8(device)&&r.u8(minor)&&r.u32(session)&&r.u32(capabilities)&&r.done()))return false;
    if(device!=1){RCLCPP_WARN(get_logger(),"Unexpected peer device %u",device);return true;}
    if(session!=session_id_){session_id_=session;last_heartbeat_=Clock::time_point{};}
    peer_minor_=minor;
    RCLCPP_INFO(get_logger(),"STM32 HELLO session=%u minor=%u capabilities=0x%08X",session,minor,capabilities);
    return true;
  }
  bool on_heartbeat(proto::Reader &r){
    std::uint8_t state,owner;std::uint32_t uptime;
    if(!(r.u8(state)&&r.u8(owner)&&r.u32(uptime)&&r.done()))return false;
    control_owner_=owner;last_heartbeat_=Clock::now();++heartbeats_;
    (void)state;(void)uptime;return true;
  }
  bool on_peer_link(proto::Reader &r){
    return r.u32(peer_dropped_)&&r.u32(peer_crc_)&&r.u32(peer_tx_errors_)&&r.done();
  }
  bool on_mission(const proto::Frame &f,proto::Reader &r){
    std::uint8_t phase,state,action,plan,done;std::uint16_t err;
    if(!(r.u8(phase)&&r.u8(state)&&r.u8(action)&&r.u8(plan)&&r.u8(done)&&r.u16(err)&&r.done()))return false;
    cmsg::MissionState m;m.header=header("base_link");m.source_time_ms=f.timestamp_ms;
    m.phase_code=phase;m.status_code=state;m.action_index=action;m.plan_index=plan;
    m.plan_done=done!=0;m.error_code=static_cast<std::uint8_t>(std::min<std::uint16_t>(err,255));
    m.race_state_semantics_known=true;mission_pub_->publish(m);return true;
  }
  bool on_vision_event(const proto::Frame &f,proto::Reader &r){
    std::uint8_t event,step,op,station,color,ring,valid,task_len;
    std::uint32_t token;std::int16_t u,v;
    if(!(r.u8(event)&&r.u8(step)&&r.u32(token)&&r.u8(op)&&r.u8(station)&&r.u8(color)&&r.u8(ring)&&r.i16(u)&&r.i16(v)&&r.u8(valid)&&r.u8(task_len)))return false;
    if(task_len>16)return false;
    std::string task;if(!r.text(task_len,task)||!r.done())return false;
    if(event==1 && (task.size()!=12 || !std::all_of(task.begin(),task.end(),[](char c){return c>='0'&&c<='9';})))return false;
    cmsg::VisionEvent m;m.header=header("camera_frame");m.event_type=event_name(event);
    m.task_code=task;m.step=step;m.token=token;m.operation=operation_name(op);m.station=station_name(station);
    m.color_id=color;m.ring_id=ring;m.pixel_u=u;m.pixel_v=v;
    m.pixel_position_valid=(valid&1)!=0;m.metric_position_valid=false;
    vision_pub_->publish(m);(void)f;return true;
  }
  bool on_fault(const proto::Frame &f,proto::Reader &r){
    std::uint8_t kind,source;std::uint16_t code;
    if(!(r.u8(kind)&&r.u16(code)&&r.u8(source)&&r.done()))return false;
    diagnostic_msgs::msg::DiagnosticArray a;a.header=header("");
    diagnostic_msgs::msg::DiagnosticStatus s;s.name="car/stm32/fault";
    s.level=diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    s.message="STM32 fault event";s.hardware_id="STM32F407";
    for(const auto &p:std::vector<std::pair<std::string,std::string>>{{"fault_kind",std::to_string(kind)},{"error_code",std::to_string(code)},{"source",std::to_string(source)}}){
      diagnostic_msgs::msg::KeyValue k;k.key=p.first;k.value=p.second;s.values.push_back(k);
    }
    a.status.push_back(s);fault_pub_->publish(a);(void)f;return true;
  }
  bool on_command_feedback(const proto::Frame &f,proto::Reader &r){
    cmsg::CommandFeedback m;m.header=header("base_link");m.source_time_ms=f.timestamp_ms;m.event_type=f.type;
    switch(static_cast<proto::Type>(f.type)){
      case proto::Type::COMMAND_ACK:if(!(r.u32(m.cmd_id)&&r.u8(m.original_command_type)&&r.u8(m.status_code)))return false;break;
      case proto::Type::COMMAND_REJECTED:case proto::Type::COMMAND_FAILED:
        if(!(r.u32(m.cmd_id)&&r.u16(m.error_code)))return false;
        break;
      case proto::Type::COMMAND_STARTED:if(!r.u32(m.cmd_id))return false;break;
      case proto::Type::COMMAND_PROGRESS:if(!(r.u32(m.cmd_id)&&r.u8(m.progress_percent)&&r.u8(m.phase_code)))return false;break;
      case proto::Type::COMMAND_COMPLETED:if(!(r.u32(m.cmd_id)&&r.u8(m.status_code)))return false;break;
      case proto::Type::CONTROL_STATUS:
        if(!(r.u8(m.control_owner)&&r.u8(m.chassis_mode)&&r.u8(m.run_state)))return false;
        control_owner_=m.control_owner;break;
      default:return false;
    }
    if(!r.done())return false;
    command_pub_->publish(m);return true;
  }
  void publish_link_status(){
    cmsg::LinkStatus m;m.header=header("base_link");m.connected=fd_>=0;
    m.heartbeat_fresh=is_recent(last_heartbeat_,stale_ms_);
    m.peer_session_id=session_id_;m.peer_protocol_minor=peer_minor_;m.control_owner=control_owner_;
    m.received_frames=parser_.frames_ok();m.crc_errors=parser_.crc_errors();
    m.length_errors=parser_.length_errors();m.version_errors=parser_.version_errors();
    m.discarded_bytes=parser_.discarded_bytes();m.payload_errors=payload_errors_;
    m.received_heartbeats=heartbeats_;m.peer_reported_crc_errors=peer_crc_;
    m.peer_reported_dropped_frames=peer_dropped_;m.peer_reported_tx_errors=peer_tx_errors_;
    link_pub_->publish(m);
  }
  std::string port_;int baud_=115200,stale_ms_=500,fd_=-1;
  double wheel_ppm_=1750,wheel_ppr_=440,wheelbase_half_=0.093,track_half_=0.110;
  bool publish_joint_=true;
  proto::Parser parser_;std::uint64_t payload_errors_=0,heartbeats_=0;
  Clock::time_point next_connect_{},last_heartbeat_{},last_power_{},last_imu_{},last_wheels_{},last_target_{};
  std::uint32_t session_id_=0,peer_dropped_=0,peer_crc_=0,peer_tx_errors_=0;
  std::uint8_t peer_minor_=0,control_owner_=0,target_mode_=0;
  bool target_seen_=false,imu_valid_=false,driver_valid_=false,board_valid_=false;
  double target_x_=0,target_y_=0,target_yaw_=0,target_vx_=0,target_vy_=0,target_wz_=0;
  double imu_yaw_=0,driver_v_=0,board_v_=0,wheel_wz_=0;
  rclcpp::TimerBase::SharedPtr rx_timer_,status_timer_;
  rclcpp::Publisher<cmsg::ChassisTelemetry>::SharedPtr chassis_pub_;
  rclcpp::Publisher<cmsg::WheelTelemetry>::SharedPtr wheel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<cmsg::MechanismState>::SharedPtr mechanism_pub_;
  rclcpp::Publisher<cmsg::PowerTelemetry>::SharedPtr power_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr target_twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<cmsg::VisionEvent>::SharedPtr vision_pub_;
  rclcpp::Publisher<cmsg::MissionState>::SharedPtr mission_pub_;
  rclcpp::Publisher<cmsg::CommandFeedback>::SharedPtr command_pub_;
  rclcpp::Publisher<cmsg::LinkStatus>::SharedPtr link_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr fault_pub_;
};
int main(int argc,char **argv){
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<SerialBridge>());}
  catch(const std::exception &e){RCLCPP_FATAL(rclcpp::get_logger("car_serial_bridge"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
