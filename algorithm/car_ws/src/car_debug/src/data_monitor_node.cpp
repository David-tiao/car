// ROS 2 Humble C++ read-only diagnostics for STM32 ULog43 replay/monitoring.
// The monitor never publishes /cmd_vel and never touches serial ports.
#include "car_debug/health_core.hpp"

#include <car_interfaces/msg/chassis_telemetry.hpp>
#include <car_interfaces/msg/mission_state.hpp>
#include <car_interfaces/msg/ulog43.hpp>
#include <car_interfaces/msg/wheel_telemetry.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <utility>
#include <string>
#include <vector>

using namespace std::chrono_literals;
namespace cmsg = car_interfaces::msg;

class DataMonitor final : public rclcpp::Node {
 public:
  DataMonitor() : Node("car_data_monitor") {
    stale_after_ms_ = declare_parameter<int>("stale_after_ms", 1000);
    max_path_points_ = declare_parameter<int>("max_path_points", 1000);
    path_publish_stride_ = declare_parameter<int>("path_publish_stride", 4);
    wheel_error_warn_pps_ = declare_parameter<double>("wheel_error_warn_pps", 350.0);
    if (stale_after_ms_ < 100 || max_path_points_ < 10 ||
        path_publish_stride_ < 1 || wheel_error_warn_pps_ <= 0.0) {
      throw std::invalid_argument("invalid car_debug parameters");
    }
    auto qos = rclcpp::QoS(rclcpp::KeepLast(30)).reliable();
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/car/debug/trajectory", qos);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/car/debug/diagnostics", qos);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", qos, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          received_[ODOM].on_receive(car_debug::SteadyClock::now());
          const auto& p = msg->pose.pose.position;
          const auto& q = msg->pose.pose.orientation;
          if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
              !std::isfinite(q.x) || !std::isfinite(q.y) ||
              !std::isfinite(q.z) || !std::isfinite(q.w)) {
            ++invalid_odom_count_;
            return;
          }
          if (msg->header.frame_id.empty()) {
            ++invalid_odom_count_;
            return;
          }
          if (!path_.poses.empty() && msg->header.frame_id != path_.header.frame_id) {
            path_.poses.clear();
          }
          path_.header.frame_id = msg->header.frame_id;
          path_.header.stamp = msg->header.stamp;
          geometry_msgs::msg::PoseStamped pose;
          pose.header = msg->header;
          pose.pose = msg->pose.pose;
          path_.poses.push_back(pose);
          if (path_.poses.size() > static_cast<std::size_t>(max_path_points_)) {
            path_.poses.erase(path_.poses.begin());
          }
          if (++path_counter_ % static_cast<std::uint64_t>(path_publish_stride_) == 0) {
            path_pub_->publish(path_);
          }
        });
    chassis_sub_ = create_subscription<cmsg::ChassisTelemetry>(
        "/car/chassis/telemetry", qos,
        [this](cmsg::ChassisTelemetry::ConstSharedPtr msg) {
          received_[CHASSIS].on_receive(car_debug::SteadyClock::now());
          last_voltage_v_ = msg->board_voltage_v;
          last_chassis_mode_ = msg->chassis_mode;
        });
    wheels_sub_ = create_subscription<cmsg::WheelTelemetry>(
        "/car/chassis/wheels", qos,
        [this](cmsg::WheelTelemetry::ConstSharedPtr msg) {
          received_[WHEELS].on_receive(car_debug::SteadyClock::now());
          std::array<double, 4> target{}, actual{};
          for (std::size_t i = 0; i < 4; ++i) {
            target[i] = msg->target_pulse_per_s[i];
            actual[i] = msg->actual_pulse_per_s[i];
          }
          last_wheel_error_pps_ = car_debug::max_wheel_tracking_error(target, actual);
        });
    mission_sub_ = create_subscription<cmsg::MissionState>(
        "/car/mission/state", qos,
        [this](cmsg::MissionState::ConstSharedPtr msg) {
          received_[MISSION].on_receive(car_debug::SteadyClock::now());
          last_mission_error_ = msg->error_code;
          last_mission_phase_ = msg->phase_code;
          last_plan_index_ = msg->plan_index;
        });
    raw_sub_ = create_subscription<cmsg::Ulog43>(
        "/car/debug/ulog43", qos,
        [this](cmsg::Ulog43::ConstSharedPtr msg) {
          received_[RAW].on_receive(car_debug::SteadyClock::now());
          last_mcu_ms_ = msg->source_time_ms;
        });
    timer_ = create_wall_timer(500ms, [this]() { publish_diagnostics(); });
    RCLCPP_INFO(get_logger(), "C++ monitor started: offline/replay safe; NO motor commands");
  }

 private:
  enum Channel : std::size_t {ODOM = 0, CHASSIS, WHEELS, MISSION, RAW, COUNT};
  static diagnostic_msgs::msg::KeyValue value(std::string key, std::string val) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = std::move(key);
    kv.value = std::move(val);
    return kv;
  }
  void publish_diagnostics() {
    const auto now_steady = car_debug::SteadyClock::now();
    const auto max_age = std::chrono::milliseconds(stale_after_ms_);
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = now();
    const std::array<const char*, COUNT> names = {{"odom", "chassis", "wheels", "mission", "ulog43"}};
    bool all_fresh = true;
    for (std::size_t i = 0; i < COUNT; ++i) {
      const auto& st = received_[i];
      const bool fresh = st.fresh(now_steady, max_age);
      all_fresh &= fresh;
      diagnostic_msgs::msg::DiagnosticStatus diag;
      diag.name = std::string("car/replay/") + names[i];
      diag.hardware_id = "software_pipeline";
      diag.level = fresh ? diagnostic_msgs::msg::DiagnosticStatus::OK
                         : diagnostic_msgs::msg::DiagnosticStatus::STALE;
      diag.message = fresh ? "Receiving messages" :
                     (st.seen ? "No recent messages (replay may have finished)" :
                                "No messages received yet");
      diag.values.push_back(value("received_frames", std::to_string(st.count)));
      diag.values.push_back(value("age_ms", st.seen ? std::to_string(st.age(now_steady).count()) : "n/a"));
      arr.status.push_back(std::move(diag));
    }
    diagnostic_msgs::msg::DiagnosticStatus summary;
    summary.name = "car/replay/summary";
    summary.hardware_id = "software_pipeline";
    summary.level = all_fresh ? diagnostic_msgs::msg::DiagnosticStatus::OK
                              : diagnostic_msgs::msg::DiagnosticStatus::STALE;
    summary.message = all_fresh ? "All 5 ROS2 streams are fresh" :
                                  "Offline stream incomplete/stale; not a hardware fault diagnosis";
    summary.values.push_back(value("source", "offline replay or read-only bridge"));
    summary.values.push_back(value("motor_control", "disabled"));
    summary.values.push_back(value("trajectory_points", std::to_string(path_.poses.size())));
    summary.values.push_back(value("invalid_odom_count", std::to_string(invalid_odom_count_)));
    summary.values.push_back(value("last_mcu_uptime_ms", std::to_string(last_mcu_ms_)));
    summary.values.push_back(value("max_abs_wheel_speed_error_pulse_per_s", std::to_string(last_wheel_error_pps_)));
    summary.values.push_back(value("latest_mission_error_code", std::to_string(last_mission_error_)));
    summary.values.push_back(value("latest_mission_phase_code", std::to_string(last_mission_phase_)));
    summary.values.push_back(value("latest_plan_index", std::to_string(last_plan_index_)));
    summary.values.push_back(value("latest_chassis_mode", std::to_string(last_chassis_mode_)));
    summary.values.push_back(value("board_voltage_v", std::to_string(last_voltage_v_)));
    if (all_fresh && (!std::isfinite(last_wheel_error_pps_) ||
                      last_wheel_error_pps_ > wheel_error_warn_pps_)) {
      summary.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      summary.message = "Wheel tracking discrepancy (threshold is for debugging only)";
    }
    arr.status.push_back(std::move(summary));
    diagnostics_pub_->publish(arr);
  }

  int stale_after_ms_{1000};
  int max_path_points_{1000};
  int path_publish_stride_{4};
  double wheel_error_warn_pps_{350.0};
  std::array<car_debug::StreamHealth, COUNT> received_{};
  std::uint64_t path_counter_{};
  std::uint64_t invalid_odom_count_{};
  double last_wheel_error_pps_{};
  double last_voltage_v_{};
  std::uint32_t last_mcu_ms_{};
  std::uint8_t last_chassis_mode_{};
  std::uint8_t last_mission_error_{};
  std::uint8_t last_mission_phase_{};
  std::uint8_t last_plan_index_{};
  nav_msgs::msg::Path path_{};

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<cmsg::ChassisTelemetry>::SharedPtr chassis_sub_;
  rclcpp::Subscription<cmsg::WheelTelemetry>::SharedPtr wheels_sub_;
  rclcpp::Subscription<cmsg::MissionState>::SharedPtr mission_sub_;
  rclcpp::Subscription<cmsg::Ulog43>::SharedPtr raw_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<DataMonitor>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("car_data_monitor"), "Initialization failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
