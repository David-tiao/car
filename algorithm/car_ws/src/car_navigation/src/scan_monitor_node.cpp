// Read-only diagnostics: scan age, valid ranges, map status, odometry, static/dynamic TF.
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <cmath>
#include <memory>
#include <string>
using namespace std::chrono_literals;
namespace {
void kv(diagnostic_msgs::msg::DiagnosticStatus& s,const std::string& k,const std::string& v){
 diagnostic_msgs::msg::KeyValue m;m.key=k;m.value=v;s.values.push_back(m);
}
}
class ScanMonitor final:public rclcpp::Node {
public:
 ScanMonitor():Node("car_navigation_monitor") {
   stale_ms_=declare_parameter<int>("stale_ms",1000);
   if(stale_ms_<100)throw std::invalid_argument("stale_ms must be >=100");
   tf_buffer_=std::make_shared<tf2_ros::Buffer>(get_clock());
   tf_listener_=std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
   diag_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/car/navigation/diagnostics",10);
   scan_sub_=create_subscription<sensor_msgs::msg::LaserScan>("/scan",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m){
       last_scan_=std::chrono::steady_clock::now();++scan_count_;
       scan_frame_=m->header.frame_id;scan_samples_=m->ranges.size();valid_ranges_=0;
       for(auto r:m->ranges)
         if(std::isfinite(r)&&r>=m->range_min&&r<=m->range_max)++valid_ranges_;
      });
   odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/odom",10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m){
       last_odom_=std::chrono::steady_clock::now();odom_frame_=m->header.frame_id;
      });
   auto map_q=rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
   map_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/map",map_q,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){map_ok_=(m->header.frame_id=="map"&&!m->data.empty());});
   mask_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/car/navigation/keepout_mask",map_q,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){mask_ok_=(m->header.frame_id=="map"&&!m->data.empty());});
   timer_=create_wall_timer(1s,[this](){publish();});
   RCLCPP_INFO(get_logger(),"Read-only scan/TF checker started; no navigation controller or velocity commands");
 }
private:
 using Clock=std::chrono::steady_clock;
 static int age_ms(Clock::time_point t) {
   if(t==Clock::time_point{})return -1;
   return static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now()-t).count());
 }
 void publish(){
   diagnostic_msgs::msg::DiagnosticArray arr;arr.header.stamp=now();
   diagnostic_msgs::msg::DiagnosticStatus scan;
   scan.name="car_navigation/laser_scan";scan.hardware_id="YDLIDAR_or_mock";
   const int age=age_ms(last_scan_);
   scan.level=(age<0||age>stale_ms_||scan_frame_!="laser_frame"||scan_samples_==0||valid_ranges_==0)?
     diagnostic_msgs::msg::DiagnosticStatus::WARN:diagnostic_msgs::msg::DiagnosticStatus::OK;
   scan.message=(age<0?"NO_SCAN":age>stale_ms_?"STALE_SCAN":scan_frame_!="laser_frame"?"WRONG_FRAME":
                 scan_samples_==0?"EMPTY_SCAN":valid_ranges_==0?"NO_FINITE_RANGES":"SCAN_RECEIVED");
   kv(scan,"age_ms",std::to_string(age));kv(scan,"ranges",std::to_string(scan_samples_));
   kv(scan,"finite_in_range",std::to_string(valid_ranges_));kv(scan,"frames",std::to_string(scan_count_));
   arr.status.push_back(scan);
   diagnostic_msgs::msg::DiagnosticStatus mapping;
   mapping.name="car_navigation/map_keepout";mapping.hardware_id="static_field";
   mapping.level=(map_ok_&&mask_ok_)?diagnostic_msgs::msg::DiagnosticStatus::OK:
       diagnostic_msgs::msg::DiagnosticStatus::WARN;
   mapping.message=(map_ok_&&mask_ok_)?"MAP_AND_MASK_RECEIVED":"MISSING_MAP_OR_MASK";
   arr.status.push_back(mapping);
   diagnostic_msgs::msg::DiagnosticStatus transforms;
   transforms.name="car_navigation/tf_localization";transforms.hardware_id="TF";
   const bool static_ok=tf_buffer_->canTransform("base_link","laser_frame",tf2::TimePointZero);
   const bool map_ok=tf_buffer_->canTransform("map","base_link",tf2::TimePointZero);
   const int odom_age=age_ms(last_odom_);
   // nav planning preview is allowed without localization, autonomous motion is not.
   transforms.level=(static_ok&&map_ok&&odom_age>=0&&odom_age<=stale_ms_)?
       diagnostic_msgs::msg::DiagnosticStatus::OK:diagnostic_msgs::msg::DiagnosticStatus::WARN;
   transforms.message=transforms.level==diagnostic_msgs::msg::DiagnosticStatus::OK?
       "TF_AND_ODOM_PRESENT":"TF_OR_ODOM_NOT_READY_FOR_REAL_NAV";
   kv(transforms,"base_to_laser",static_ok?"true":"false");
   kv(transforms,"map_to_base",map_ok?"true":"false");
   kv(transforms,"odom_age_ms",std::to_string(odom_age));
   kv(transforms,"odom_frame",odom_frame_);
   arr.status.push_back(transforms);
   diag_->publish(arr);
 }
 int stale_ms_=1000;bool map_ok_=false,mask_ok_=false;
 std::size_t scan_samples_=0,valid_ranges_=0;std::uint64_t scan_count_=0;
 std::string scan_frame_,odom_frame_;
 Clock::time_point last_scan_{},last_odom_{};
 rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_;
 rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
 rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
 rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_,mask_sub_;
 rclcpp::TimerBase::SharedPtr timer_;
 std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
 std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);
 try {rclcpp::spin(std::make_shared<ScanMonitor>());}
 catch(const std::exception&ex){RCLCPP_FATAL(rclcpp::get_logger("car_navigation_monitor"),"%s",ex.what());rclcpp::shutdown();return 1;}
 rclcpp::shutdown();return 0;}
