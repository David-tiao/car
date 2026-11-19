// Standalone read-only static map publisher; do not run with another /map publisher.
#include "car_navigation/navigation_core.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <stdexcept>
#include <string>
using namespace std::chrono_literals;
class MapPublisher : public rclcpp::Node {
public:
  MapPublisher() : Node("car_map_publisher") {
    auto path = declare_parameter<std::string>("map_yaml","");
    auto mask_path = declare_parameter<std::string>("mask_yaml","");
    if (path.empty() || mask_path.empty()) throw std::invalid_argument("map_yaml and mask_yaml required");
    const auto map=car_navigation::load_trinary_pgm_yaml(path);
    const auto mask=car_navigation::load_trinary_pgm_yaml(mask_path);
    if (!car_navigation::matching_geometry(map,mask))
      throw std::runtime_error("physical map and keepout mask geometry mismatch");
    physical_ = make_grid(map);
    keepout_ = make_grid(mask);
    auto q=rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map",q);
    mask_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/car/navigation/keepout_mask",q);
    tick();
    timer_=create_wall_timer(1s,[this](){tick();});
    RCLCPP_INFO(get_logger(),"Read-only map + keepout published: %dx%d @ %.3fm; no motor commands",
                map.width,map.height,map.resolution);
  }
private:
  nav_msgs::msg::OccupancyGrid make_grid(const car_navigation::Grid &g) {
    nav_msgs::msg::OccupancyGrid m;
    m.header.frame_id="map";
    m.info.resolution=static_cast<float>(g.resolution);
    m.info.width=static_cast<std::uint32_t>(g.width);
    m.info.height=static_cast<std::uint32_t>(g.height);
    m.info.origin.position.x=g.origin_x; m.info.origin.position.y=g.origin_y;
    m.info.origin.orientation.w=1.0;
    m.data=g.data;
    return m;
  }
  void tick() {
    const auto stamp=now();
    physical_.header.stamp=stamp;physical_.info.map_load_time=stamp;
    keepout_.header.stamp=stamp;keepout_.info.map_load_time=stamp;
    map_pub_->publish(physical_);
    mask_pub_->publish(keepout_);
  }
  nav_msgs::msg::OccupancyGrid physical_,keepout_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_,mask_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<MapPublisher>());}
  catch(const std::exception&ex){RCLCPP_FATAL(rclcpp::get_logger("car_map_publisher"),"%s",ex.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
