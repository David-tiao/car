#include "car_stm32_bridge/ulog43_parser.hpp"

#include <car_interfaces/msg/chassis_telemetry.hpp>
#include <car_interfaces/msg/mission_state.hpp>
#include <car_interfaces/msg/ulog43.hpp>
#include <car_interfaces/msg/wheel_telemetry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <stdexcept>
#include <utility>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;
namespace cmsg = car_interfaces::msg;
namespace {
constexpr double kPi=3.14159265358979323846264338327950288;
constexpr double kDegToRad=kPi/180.0;
constexpr double kULogPV=0.073632;

std::uint8_t u8(double v) {
  if (!std::isfinite(v)) return 0;
  return static_cast<std::uint8_t>(std::clamp(std::lround(v),0l,255l));
}

// Firmware explicitly initializes these channels; all other columns can be
// zero placeholders. Compare STM32F4/Hardware/bsp_ulog.c assignment lines.
std::array<bool,43> semantics_mask() {
  std::array<bool,43> mask{};
  constexpr int known[]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,
     16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,
     34,35,36,37,38,39,40,41,42};
  for(int i:known) mask[static_cast<std::size_t>(i)]=true;
  return mask;
}
}

class Ulog43Bridge final : public rclcpp::Node {
public:
  Ulog43Bridge(): Node("car_ulog43_bridge") {
    port_=declare_parameter<std::string>("port","");
    replay_file_=declare_parameter<std::string>("replay_file","");
    baud_=declare_parameter<int>("baud",115200);
    interval_ms_=declare_parameter<int>("replay_interval_ms",50);
    pulses_per_meter_=declare_parameter<double>("pulses_per_meter",1750.0);
    wheel_ppr_=declare_parameter<double>("wheel_ppr",440.0);
    wheelbase_half_=declare_parameter<double>("wheelbase_half_m",0.093);
    track_half_=declare_parameter<double>("track_half_m",0.11);
    publish_tf_=declare_parameter<bool>("publish_tf",false);
    publish_joint_=declare_parameter<bool>("publish_joint_states",true);
    if (!(pulses_per_meter_>0 && wheel_ppr_>0 && wheelbase_half_>0 && track_half_>0)) {
      throw std::invalid_argument("pulses_per_meter / wheel geometry must be positive");
    }
    if (baud_ != 115200 && baud_ != 230400) {
      throw std::invalid_argument("baud must be 115200 or 230400");
    }
    if (!port_.empty() && !replay_file_.empty()) {
      throw std::invalid_argument("port and replay_file must not both be set");
    }
    auto qos=rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
    raw_pub_=create_publisher<cmsg::Ulog43>("/car/debug/ulog43",qos);
    chassis_pub_=create_publisher<cmsg::ChassisTelemetry>("/car/chassis/telemetry",qos);
    wheel_pub_=create_publisher<cmsg::WheelTelemetry>("/car/chassis/wheels",qos);
    mission_pub_=create_publisher<cmsg::MissionState>("/car/mission/state",qos);
    odom_pub_=create_publisher<nav_msgs::msg::Odometry>("/odom",qos);
    if (publish_joint_) joint_pub_=create_publisher<sensor_msgs::msg::JointState>("/joint_states",qos);
    if (publish_tf_) tf_pub_=std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    if (!replay_file_.empty()) {
      replay_.open(replay_file_);
      if (!replay_) throw std::runtime_error("Cannot read replay_file: "+replay_file_);
      timer_=create_wall_timer(std::chrono::milliseconds(std::max(1,interval_ms_)),[this](){tick_replay();});
      RCLCPP_INFO(get_logger(),"Read-only ULog43 replay: %s",replay_file_.c_str());
    } else {
      timer_=create_wall_timer(10ms,[this](){tick_serial();});
      if (port_.empty()) RCLCPP_WARN(get_logger(),"STANDBY: no port/replay file; motor commands disabled");
      else RCLCPP_INFO(get_logger(),"Read-only ULog43 serial port: %s",port_.c_str());
    }
    RCLCPP_WARN(get_logger(),"This is a MONITOR ONLY. No STM32 command protocol is implemented.");
  }
  ~Ulog43Bridge() override {close_serial();}

private:
  void close_serial() {
    if (fd_>=0) {::close(fd_);fd_=-1;}
  }
  bool open_serial() {
    close_serial();
    fd_=::open(port_.c_str(),O_RDONLY|O_NOCTTY|O_NONBLOCK);
    if(fd_<0) return false;
    termios t{};
    if(::tcgetattr(fd_,&t)!=0){close_serial();return false;}
    ::cfmakeraw(&t);
    const speed_t speed = baud_ == 230400 ? B230400 : B115200;
    if (::cfsetispeed(&t, speed) != 0 || ::cfsetospeed(&t, speed) != 0) {
      close_serial();
      return false;
    }
    t.c_cflag=(t.c_cflag & ~CSIZE)|CS8|CLOCAL|CREAD;
    t.c_cflag &= ~(PARENB|CSTOPB|CRTSCTS);
    t.c_cc[VMIN]=0;
    t.c_cc[VTIME]=0;
    if(::tcsetattr(fd_,TCSANOW,&t)!=0){close_serial();return false;}
    have_last_=false;rx_buffer_.clear();
    return true;
  }
  void tick_serial() {
    if(port_.empty()) return;
    if(fd_<0) {
      const auto now=std::chrono::steady_clock::now();
      if(now<next_reconnect_) return;
      next_reconnect_=now+2s;
      if(open_serial()) RCLCPP_INFO(get_logger(),"Serial reader opened: %s",port_.c_str());
      else RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Cannot open %s: %s",port_.c_str(),std::strerror(errno));
      return;
    }
    char buf[1024];
    for(int attempt=0; attempt<8; ++attempt) {
      const ssize_t n=::read(fd_,buf,sizeof(buf));
      if(n>0) {on_bytes(buf,static_cast<std::size_t>(n));continue;}
      if(n==0 || (n<0 && (errno==EAGAIN || errno==EWOULDBLOCK))) break;
      RCLCPP_WARN(get_logger(),"Serial reader disconnected: %s",std::strerror(errno));
      close_serial();
      next_reconnect_=std::chrono::steady_clock::now()+2s;
      break;
    }
  }
  void tick_replay() {
    if(replay_done_) return;
    std::string line;
    if(std::getline(replay_,line)) on_line(std::move(line));
    else {replay_done_=true;RCLCPP_INFO(get_logger(),"Replay complete; no loop or motor command");}
  }
  void on_bytes(const char *buf,std::size_t n) {
    for(std::size_t i=0;i<n;++i) {
      const char ch=buf[i];
      if(ch=='\n') {
        if(!rx_buffer_.empty()) on_line(std::move(rx_buffer_));
        rx_buffer_.clear();
      } else if(rx_buffer_.size()<2048) rx_buffer_.push_back(ch);
      else rx_buffer_.clear(); // discard unbounded/noisy frames
    }
  }
  void on_line(std::string line) {
    if(line.empty() || line.front()=='#') return;
    car_stm32_bridge::Ulog43Frame frame;
    std::string why;
    if(!car_stm32_bridge::parse_ulog43(line,frame,&why)) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,"Invalid 43-column row (%s)",why.c_str());
      return;
    }
    // Native 32-bit uptime wraps at ~49.7 days. Distinguish wrap from backward time.
    if(have_last_) {
      if(frame.source_time_ms==last_time_ms_) return;
      if(frame.source_time_ms<last_time_ms_ &&
         last_time_ms_-frame.source_time_ms<0x80000000u) {
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,"Backwards MCU timestamp, discarding row");
        return;
      }
    }
    have_last_=true;
    last_time_ms_=frame.source_time_ms;
    publish(frame,line);
  }
  void publish(const car_stm32_bridge::Ulog43Frame &f,const std::string &line) {
    auto h=std_msgs::msg::Header();h.stamp=now();h.frame_id="odom";
    const auto v=car_stm32_bridge::wheels_to_body(f,pulses_per_meter_,wheelbase_half_,track_half_);
    cmsg::Ulog43 raw;raw.header=h;raw.source_time_ms=f.source_time_ms;
    raw.channels=f.ch;raw.valid_semantics=semantics_mask();raw.original_line=line;
    raw_pub_->publish(raw);

    cmsg::ChassisTelemetry chassis;
    chassis.header=h;chassis.source_time_ms=f.source_time_ms;
    chassis.chassis_mode=u8(f.ch[0]);chassis.target_reached=f.ch[4]>0.5;
    chassis.x_m=f.ch[32];chassis.y_m=f.ch[33];chassis.yaw_rad=f.ch[16]*kDegToRad;
    chassis.vx_mps=v.vx_mps;chassis.vy_mps=v.vy_mps;
    chassis.wz_radps=f.ch[15];chassis.wheel_estimated_wz_radps=v.wz_radps;
    chassis.target_x_m=f.ch[21];chassis.target_y_m=f.ch[22];
    chassis.target_yaw_rad=f.ch[23]*kDegToRad;chassis.target_wz_radps=f.ch[14];
    chassis.target_speed_mps=f.ch[19]*kULogPV;
    chassis.target_linear_components_valid=false; // firmware serializes ch12/13 as %.0f
    chassis.x_target_error_m=f.ch[24];chassis.y_target_error_m=f.ch[25];
    chassis.yaw_target_error_rad=f.ch[42]*kDegToRad;
    chassis.lateral_target_error_m=f.ch[39];chassis.travelled_distance_m=f.ch[34];
    chassis.driver_voltage_v=f.ch[40];chassis.board_voltage_v=f.ch[41];
    chassis.imu_yaw_rad=f.ch[17]*kDegToRad;chassis.imu_valid=false; // ULog43 has no IMU-valid bit
    chassis_pub_->publish(chassis);

    cmsg::WheelTelemetry wheel;wheel.header=h;wheel.header.frame_id="base_link";
    wheel.source_time_ms=f.source_time_ms;
    const std::array<int,4> ti{{27,35,36,37}},ai{{28,29,30,31}};
    for(std::size_t i=0;i<4;++i){
      wheel.target_pulse_per_s[i]=f.ch[ti[i]];
      wheel.actual_pulse_per_s[i]=f.ch[ai[i]];
      wheel.target_mps[i]=f.ch[ti[i]]/pulses_per_meter_;
      wheel.actual_mps[i]=f.ch[ai[i]]/pulses_per_meter_;
    }
    wheel.max_abs_pid_output=f.ch[38];
    wheel.per_wheel_pid_output_valid=false;
    wheel.encoder_counts_valid=false;
    wheel_pub_->publish(wheel);

    cmsg::MissionState mission;mission.header=h;mission.header.frame_id="base_link";
    mission.source_time_ms=f.source_time_ms;
    mission.phase_code=u8(f.ch[1]);mission.status_code=u8(f.ch[2]);
    mission.action_index=u8(f.ch[3]);mission.error_code=u8(f.ch[6]);
    mission.plan_index=u8(f.ch[7]);mission.plan_done=f.ch[9]>0.5;
    mission.race_state_semantics_known=false; // depends on APP_RUN_MODE build flag
    mission_pub_->publish(mission);

    nav_msgs::msg::Odometry odom;odom.header=h;odom.child_frame_id="base_link";
    odom.pose.pose.position.x=chassis.x_m;odom.pose.pose.position.y=chassis.y_m;
    const double half=chassis.yaw_rad*0.5;
    odom.pose.pose.orientation.z=std::sin(half);
    odom.pose.pose.orientation.w=std::cos(half);
    odom.twist.twist.linear.x=chassis.vx_mps;
    odom.twist.twist.linear.y=chassis.vy_mps;
    odom.twist.twist.angular.z=chassis.wz_radps;
    // Not a calibrated covariance: deliberately very uncertain. Do NOT fuse
    // as a calibrated navigation source without real measurement/model tests.
    for(int i=0;i<6;++i) {
      odom.pose.covariance[static_cast<std::size_t>(i*6+i)]=1e3;
      odom.twist.covariance[static_cast<std::size_t>(i*6+i)]=1e3;
    }
    odom_pub_->publish(odom);

    if(joint_pub_) {
      sensor_msgs::msg::JointState joint;
      joint.header=h;joint.header.frame_id="base_link";
      joint.name={"wheel_fl_joint","wheel_fr_joint","wheel_rl_joint","wheel_rr_joint"};
      const double ratio=2.0*kPi/wheel_ppr_;
      joint.velocity={f.ch[28]*ratio,f.ch[29]*ratio,f.ch[30]*ratio,f.ch[31]*ratio};
      // No wheel position; ULog43 does not record accumulated encoder counts.
      joint_pub_->publish(joint);
    }
    if(tf_pub_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header=h;tf.child_frame_id="base_link";
      tf.transform.translation.x=chassis.x_m;tf.transform.translation.y=chassis.y_m;
      tf.transform.rotation.z=std::sin(half);tf.transform.rotation.w=std::cos(half);
      tf_pub_->sendTransform(tf);
    }
  }

  std::string port_,replay_file_,rx_buffer_;
  int baud_=115200,interval_ms_=50,fd_=-1;
  double pulses_per_meter_=1750.0,wheel_ppr_=440.0,wheelbase_half_=0.093,track_half_=0.11;
  bool publish_tf_=false,publish_joint_=true,replay_done_=false,have_last_=false;
  std::uint32_t last_time_ms_=0;
  std::chrono::steady_clock::time_point next_reconnect_{};
  std::ifstream replay_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<cmsg::Ulog43>::SharedPtr raw_pub_;
  rclcpp::Publisher<cmsg::ChassisTelemetry>::SharedPtr chassis_pub_;
  rclcpp::Publisher<cmsg::WheelTelemetry>::SharedPtr wheel_pub_;
  rclcpp::Publisher<cmsg::MissionState>::SharedPtr mission_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
};

int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<Ulog43Bridge>());}
  catch(const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("car_ulog43_bridge"),"Startup failed: %s",e.what());
    rclcpp::shutdown();return 1;
  }
  rclcpp::shutdown();return 0;
}
