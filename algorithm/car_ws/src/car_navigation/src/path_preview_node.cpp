// A* path PREVIEW only; manual start/goal in `map` frame, NO /cmd_vel/actuator/TF output.
#include "car_navigation/navigation_core.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
using namespace std::chrono_literals;
namespace {
car_navigation::Grid get_grid(const nav_msgs::msg::OccupancyGrid &m) {
  car_navigation::Grid g;
  g.width=static_cast<int>(m.info.width);g.height=static_cast<int>(m.info.height);
  g.resolution=m.info.resolution;
  g.origin_x=m.info.origin.position.x;g.origin_y=m.info.origin.position.y;
  g.origin_yaw=2*std::atan2(m.info.origin.orientation.z,m.info.origin.orientation.w);
  g.data=m.data;
  return g;
}
}
class PathPreview : public rclcpp::Node {
public:
  PathPreview():Node("car_path_preview") {
    clearance_=declare_parameter<double>("clearance_m",0.17);
    if(!std::isfinite(clearance_)||clearance_<0||clearance_>0.5)
      throw std::invalid_argument("clearance_m must be within 0..0.5");
    auto trans=rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    auto qos=rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    path_pub_=create_publisher<nav_msgs::msg::Path>("/car/navigation/preview_path",trans);
    status_pub_=create_publisher<std_msgs::msg::String>("/car/navigation/plan_status",trans);
    map_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/map",trans,
       [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){map_=get_grid(*m);have_map_=true;plan();});
    mask_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/car/navigation/keepout_mask",trans,
       [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){mask_=get_grid(*m);have_mask_=true;plan();});
    start_sub_=create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose",qos,
       [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m){
          if(m->header.frame_id!="map") {publish_status("INVALID_START_FRAME: expected map");return;}
          start_x_=m->pose.pose.position.x; start_y_=m->pose.pose.position.y;
          have_start_=true;plan();
       });
    goal_sub_=create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose",qos,
       [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m){
          if(m->header.frame_id!="map") {publish_status("INVALID_GOAL_FRAME: expected map");return;}
          goal_x_=m->pose.position.x; goal_y_=m->pose.position.y;
          have_goal_=true;plan();
       });
    timer_=create_wall_timer(1s,[this](){
      if(have_result_) {cached_.header.stamp=now();path_pub_->publish(cached_);}
      if(!last_status_.empty()) {std_msgs::msg::String s;s.data=last_status_;status_pub_->publish(s);}
    });
    publish_status("WAITING_FOR_MAP_MASK_START_GOAL");
    RCLCPP_WARN(get_logger(),"PATH PREVIEW ONLY; requires /initialpose and /goal_pose in map frame; never publishes /cmd_vel");
  }
private:
  void publish_status(const std::string& s){
    last_status_=s;std_msgs::msg::String m;m.data=s;status_pub_->publish(m);
    RCLCPP_INFO(get_logger(),"%s",s.c_str());
  }
  void plan(){
    if(!have_map_||!have_mask_||!have_start_||!have_goal_)return;
    nav_msgs::msg::Path out;out.header.frame_id="map";out.header.stamp=now();
    auto res=car_navigation::plan_a_star(map_,mask_,start_x_,start_y_,goal_x_,goal_y_,clearance_);
    if(!res.success) {
      cached_=out;have_result_=true;path_pub_->publish(out);
      publish_status("REJECTED: "+res.reason);return;
    }
    out.poses.reserve(res.cells.size());
    for(std::size_t i=0;i<res.cells.size();++i) {
      geometry_msgs::msg::PoseStamped p;p.header=out.header;
      auto xy=car_navigation::cell_center(map_,res.cells[i]);
      p.pose.position.x=xy.first;p.pose.position.y=xy.second;
      double yaw=0;
      if(i+1<res.cells.size()) {
        auto next=car_navigation::cell_center(map_,res.cells[i+1]);
        yaw=std::atan2(next.second-xy.second,next.first-xy.first);
      } else if(i>0) {
        auto prev=car_navigation::cell_center(map_,res.cells[i-1]);
        yaw=std::atan2(xy.second-prev.second,xy.first-prev.first);
      }
      p.pose.orientation.z=std::sin(yaw/2);p.pose.orientation.w=std::cos(yaw/2);
      out.poses.push_back(p);
    }
    cached_=std::move(out);have_result_=true;path_pub_->publish(cached_);
    publish_status("OK: "+std::to_string(res.cells.size())+" cells, "+
                   std::to_string(res.length_m)+" m (PREVIEW ONLY)");
  }
  double clearance_=0.17,start_x_=0,start_y_=0,goal_x_=0,goal_y_=0;
  bool have_map_=false,have_mask_=false,have_start_=false,have_goal_=false,have_result_=false;
  car_navigation::Grid map_,mask_;
  nav_msgs::msg::Path cached_;
  std::string last_status_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_,mask_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);
 try{rclcpp::spin(std::make_shared<PathPreview>());}
 catch(const std::exception&ex){RCLCPP_FATAL(rclcpp::get_logger("car_path_preview"),"%s",ex.what());rclcpp::shutdown();return 1;}
 rclcpp::shutdown();return 0;}
