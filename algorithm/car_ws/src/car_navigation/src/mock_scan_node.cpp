// SYNTHETIC scan for offline topic validation only. Not real obstacle information.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <chrono>
#include <cmath>
#include <memory>
using namespace std::chrono_literals;
class MockScan : public rclcpp::Node {
public:
 MockScan():Node("car_mock_scan"){
   pub_=create_publisher<sensor_msgs::msg::LaserScan>("/scan",rclcpp::SensorDataQoS());
   timer_=create_wall_timer(200ms,[this](){
      sensor_msgs::msg::LaserScan m;
      m.header.frame_id="laser_frame";m.header.stamp=now();
      m.angle_min=-3.14159265f;m.angle_max=3.14159265f;
      m.angle_increment=2*3.14159265f/359.0f;
      m.time_increment=0.0f;m.scan_time=0.2f;
      m.range_min=0.1f;m.range_max=8.0f;
      m.ranges.resize(360,2.2f);
      for(std::size_t i=171;i<189;++i)m.ranges[i]=0.8f;
      pub_->publish(m);
   });
   RCLCPP_WARN(get_logger(),"SYNTHETIC /scan! Not connected to a lidar. Read-only test only.");
 }
private:
 rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
 rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<MockScan>());rclcpp::shutdown();return 0;}
