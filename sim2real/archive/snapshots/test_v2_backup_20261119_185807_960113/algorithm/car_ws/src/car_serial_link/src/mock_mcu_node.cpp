// No motor commands or hardware. Sends CAR-LINK V1 bytes over a Linux PTY.
#include "car_serial_link/protocol.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <pty.h>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
using namespace std::chrono_literals;
namespace proto=car_serial_link;

class MockMcu final:public rclcpp::Node{
public:
 MockMcu():Node("car_serial_mock_mcu"){
  link_=declare_parameter<std::string>("port_link","/tmp/car_link_demo_uart");
  emit_fault_=declare_parameter<bool>("emit_fault_demo",false);
  if(link_.rfind("/tmp/car_link_",0)!=0)throw std::invalid_argument("mock symlink must be under /tmp/car_link_*");
  char name[128]{};
  if(::openpty(&master_,&slave_,name,nullptr,nullptr)!=0)throw std::runtime_error("openpty failed");
  termios t{};if(::tcgetattr(slave_,&t)!=0)throw std::runtime_error("PTY tcgetattr failed");
  ::cfmakeraw(&t);if(::tcsetattr(slave_,TCSANOW,&t)!=0)throw std::runtime_error("PTY raw mode failed");
  struct stat st{};
  if(::lstat(link_.c_str(),&st)==0){
    if(!S_ISLNK(st.st_mode))throw std::runtime_error("refuse to replace non-symlink file: "+link_);
    if(::unlink(link_.c_str())!=0)throw std::runtime_error("remove old symlink failed");
  }
  if(::symlink(name,link_.c_str())!=0)throw std::runtime_error("symlink failed: "+std::string(std::strerror(errno)));
  ::fcntl(master_,F_SETFL,::fcntl(master_,F_GETFL)|O_NONBLOCK);
  RCLCPP_INFO(get_logger(),"MOCK ONLY; CAR-LINK PTY %s -> %s",link_.c_str(),name);
  RCLCPP_WARN(get_logger(),"Synthetic MCU data; command events are simulated, no real motor control");
  start_=std::chrono::steady_clock::now();timer_=create_wall_timer(50ms,[this]{tick();});
 }
 ~MockMcu() override{
  if(master_>=0)::close(master_);
  if(slave_>=0)::close(slave_);
  struct stat st{};if(::lstat(link_.c_str(),&st)==0&&S_ISLNK(st.st_mode))::unlink(link_.c_str());
 }
private:
 void send(proto::Type type,const proto::Writer &payload){
  proto::Frame frame;frame.type=static_cast<std::uint8_t>(type);frame.seq=seq_++;
  frame.timestamp_ms=static_cast<std::uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-start_).count());
  frame.payload=payload.bytes();auto b=proto::encode(frame);
  const auto n=::write(master_,b.data(),b.size());
  if(n<0&&errno!=EAGAIN&&errno!=EWOULDBLOCK)
    RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"PTY write error: %s",std::strerror(errno));
  else if(n>=0&&static_cast<std::size_t>(n)!=b.size())
    RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"PTY write incomplete; dropped demonstration frame");
 }
 void tick(){
  ++ticks_;
  if(ticks_==1||ticks_%100==0){
    proto::Writer w;w.u8(1);w.u8(0);w.u32(0x20261119);w.u32(0x00000007);
    send(proto::Type::HELLO,w);
  }
  if(ticks_%2==0){proto::Writer w;w.u8(1);w.u8(1);w.u32(ticks_*50);send(proto::Type::HEARTBEAT,w);}
  // A class: pose, speeds and yaw are physical quantities (all MOCK values).
  const std::int32_t x=static_cast<std::int32_t>(ticks_)*6;
  const std::int32_t y=static_cast<std::int32_t>(80*std::sin(ticks_*.03));
  const std::int32_t yaw=static_cast<std::int32_t>(ticks_)*2;
  {proto::Writer w;w.i32(x);w.i32(y);w.i32(yaw);w.i32(120);
   w.i32(static_cast<std::int32_t>(48*std::cos(ticks_*.03)));w.i32(40);w.u8(0x01);
   send(proto::Type::CHASSIS,w);}
  {proto::Writer w;
   for(const std::int32_t v:{230,260,230,260})w.i32(v);
   for(const std::int32_t v:{196,224,196,224})w.i32(v);
   for(const std::int16_t v:{100,110,105,115})w.i16(v);
   w.u8(0x01);send(proto::Type::WHEELS,w);}
  {proto::Writer w;w.i32(0);w.i32(0);w.i32(yaw);w.u8(1);send(proto::Type::IMU,w);}
  {proto::Writer w;w.i32(x+1000);w.i32(0);w.i32(yaw+100);w.i32(150);w.i32(0);w.i32(0);w.u8(1);
   send(proto::Type::CHASSIS_TARGET,w);}
  if(ticks_%2==0){
    proto::Writer w;w.i32(950);w.i32(260);w.i32(110);w.i32(40);w.u8(0x03);w.u8(0);w.u16(0);
    send(proto::Type::MECHANISM,w);
  }
  if(ticks_%10==0){
    proto::Writer w;w.u16(12000);w.u16(11800);w.u8(0x03);send(proto::Type::POWER,w);
    proto::Writer event;event.u8(1);event.u8(1);event.u8(static_cast<std::uint8_t>(ticks_/10));
    event.u8(0);event.u8(0);event.u16(0);send(proto::Type::MISSION_EVENT,event);
    proto::Writer vision;vision.u8(2);vision.i16(158);vision.i16(122);vision.u8(1);
    send(proto::Type::VISION_MEASUREMENT,vision);
  }
  if(ticks_%20==0){
    proto::Writer w;w.u32(0);w.u32(0);w.u32(0);send(proto::Type::LINK_STATUS,w);
    proto::Writer ctrl;ctrl.u8(1);ctrl.u8(1);ctrl.u8(1);send(proto::Type::CONTROL_STATUS,ctrl);
    proto::Writer event;event.u8(2);event.u8(4);event.u32(ticks_/20);event.u8(1);
    event.u8(1);event.u8(1);event.u8(2);event.i16(158);event.i16(122);event.u8(1);event.u8(0);
    send(proto::Type::VISION_EVENT,event);
  }
  if(ticks_%100==0){
    proto::Writer event;event.u8(1);event.u8(0);event.u32(ticks_/100);event.u8(0);
    event.u8(1);event.u8(0);event.u8(0);event.i16(0);event.i16(0);event.u8(0);
    event.u8(12);event.text("156123516231");send(proto::Type::VISION_EVENT,event);
  }
  // B2 lifecycle example is SYNTHETIC, not a response to any real command.
  const auto mock_cmd_id=42u+ticks_/100;
  if(ticks_%100==20){proto::Writer w;w.u32(mock_cmd_id);w.u8(0x83);w.u8(0);send(proto::Type::COMMAND_ACK,w);}
  if(ticks_%100==22){proto::Writer w;w.u32(mock_cmd_id);send(proto::Type::COMMAND_STARTED,w);}
  if(ticks_%100==30){proto::Writer w;w.u32(mock_cmd_id);w.u8(60);w.u8(2);send(proto::Type::COMMAND_PROGRESS,w);}
  if(ticks_%100==42){proto::Writer w;w.u32(mock_cmd_id);w.u8(0);send(proto::Type::COMMAND_COMPLETED,w);}
  if(emit_fault_ && ticks_==200){proto::Writer w;w.u8(1);w.u16(4);w.u8(1);send(proto::Type::FAULT_EVENT,w);}
 }
 int master_=-1,slave_=-1;std::uint16_t seq_=0;std::uint32_t ticks_=0;
 std::string link_;bool emit_fault_=false;
 std::chrono::steady_clock::time_point start_{};rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char **argv){rclcpp::init(argc,argv);
 try{rclcpp::spin(std::make_shared<MockMcu>());}
 catch(const std::exception &e){RCLCPP_FATAL(rclcpp::get_logger("car_serial_mock_mcu"),"%s",e.what());rclcpp::shutdown();return 1;}
 rclcpp::shutdown();return 0;
}
