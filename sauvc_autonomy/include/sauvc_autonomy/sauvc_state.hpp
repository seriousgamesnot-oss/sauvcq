#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <utility>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sauvc_interfaces/msg/gate_detection.hpp"
#include "sauvc_interfaces/msg/floor_clearance.hpp"
#include "sauvc_interfaces/msg/control_setpoint.hpp"

namespace sauvc_bt {

using SClock = std::chrono::steady_clock;

inline double secSince(SClock::time_point t) {
  return std::chrono::duration<double>(SClock::now() - t).count();
}
inline double wrapPi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}
inline double yawFromQuat(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

struct Params {
  double operational_depth{1.0}, min_safe_depth{0.35}, max_safe_depth{2.40};
  double arrival_radius_m{1.0};
  std::string gate_camera_id{"front"};
  bool gate_position_in_body_frame{true};
  bool depth_is_minus_z{true};
  bool use_bar30{false}, use_floor_camera{false}, use_battery{false};
  double atm_pressure_pa{101325.0}, water_density{997.0};
  double ekf_timeout_s{0.5}, imu_timeout_s{0.5}, bar30_timeout_s{0.5};
  double pool_depth_m{-1.0}, min_floor_clearance_m{-1.0};
  double floor_safe_depth_m{-1.0}, floor_data_timeout_s{1.0};
  double battery_warn_pct{25.0}, battery_critical_pct{15.0};
  std::string battery_critical_action{"hold"};   // warn | hold | abort
};

struct FloorInfo {
  bool ever{false}, fresh{false}, valid{false};
  double clearance{0.0};
};

// Thread-safe latest-value store. Subscribers write, BT nodes read.
// BT nodes never publish directly: they call command*(), and the executor
// publishes ONE merged ControlSetpoint per tick.
class SharedState {
public:
  using ControlSetpoint = sauvc_interfaces::msg::ControlSetpoint;
  using GateDetection = sauvc_interfaces::msg::GateDetection;

  explicit SharedState(const Params& params) : p(params) {}
  const Params p;

  // ------------------------------------------------ subscriber side
  void onOdom(const nav_msgs::msg::Odometry& m) {
    std::lock_guard<std::mutex> l(mtx_);
    odom_ = m; odom_t_ = SClock::now(); have_odom_ = true;
    depth_hist_.emplace_back(odom_t_, depthOfZ(m.pose.pose.position.z));
    while (!depth_hist_.empty() && secSince(depth_hist_.front().first) > 10.0)
      depth_hist_.pop_front();
  }
  void onImu(const sensor_msgs::msg::Imu& m) {
    std::lock_guard<std::mutex> l(mtx_);
    imu_ = m; imu_t_ = SClock::now(); have_imu_ = true;
  }
  void onBar30(const sensor_msgs::msg::FluidPressure& m) {
    std::lock_guard<std::mutex> l(mtx_);
    bar_pa_ = m.fluid_pressure; bar_t_ = SClock::now(); have_bar_ = true;
  }
  void onGate(const GateDetection& m) {
    std::lock_guard<std::mutex> l(mtx_);
    gates_[m.camera_id] = {m, SClock::now()};
  }
  void onFloor(const sauvc_interfaces::msg::FloorClearance& m) {
    std::lock_guard<std::mutex> l(mtx_);
    floor_ = m; floor_t_ = SClock::now(); have_floor_ = true;
  }
  void onBattery(const sensor_msgs::msg::BatteryState& m) {
    std::lock_guard<std::mutex> l(mtx_);
    double pct = m.percentage;
    if (std::isnan(pct)) return;
    if (pct <= 1.0) pct *= 100.0;      // accept 0..1 or 0..100
    batt_pct_ = pct; have_batt_ = true;
  }

  // ------------------------------------------------ readers
  bool odomEver() const { std::lock_guard<std::mutex> l(mtx_); return have_odom_; }
  double odomAge() const { std::lock_guard<std::mutex> l(mtx_); return have_odom_ ? secSince(odom_t_) : 1e9; }
  bool odomFresh() const { return odomAge() < p.ekf_timeout_s; }
  bool imuEver() const { std::lock_guard<std::mutex> l(mtx_); return have_imu_; }
  double imuAge() const { std::lock_guard<std::mutex> l(mtx_); return have_imu_ ? secSince(imu_t_) : 1e9; }
  bool imuFresh() const { return imuAge() < p.imu_timeout_s; }
  bool bar30Ever() const { std::lock_guard<std::mutex> l(mtx_); return have_bar_; }
  double bar30Age() const { std::lock_guard<std::mutex> l(mtx_); return have_bar_ ? secSince(bar_t_) : 1e9; }
  bool bar30Fresh() const { return bar30Age() < p.bar30_timeout_s; }

  geometry_msgs::msg::Pose pose() const { std::lock_guard<std::mutex> l(mtx_); return odom_.pose.pose; }
  geometry_msgs::msg::Vector3 linearVelocity() const { std::lock_guard<std::mutex> l(mtx_); return odom_.twist.twist.linear; }
  std::array<double, 36> covariance() const { std::lock_guard<std::mutex> l(mtx_); return odom_.pose.covariance; }
  sensor_msgs::msg::Imu imu() const { std::lock_guard<std::mutex> l(mtx_); return imu_; }
  double bar30Pressure() const { std::lock_guard<std::mutex> l(mtx_); return bar_pa_; }
  double yaw() const { return yawFromQuat(pose().orientation); }
  double depth() const { std::lock_guard<std::mutex> l(mtx_); return depthOfZ(odom_.pose.pose.position.z); }

  bool bar30Depth(double& d) const {
    std::lock_guard<std::mutex> l(mtx_);
    if (!have_bar_ || secSince(bar_t_) >= p.bar30_timeout_s) return false;
    d = (bar_pa_ - p.atm_pressure_pa) / (p.water_density * 9.80665);
    return true;
  }
  // Bar30 when configured and fresh, otherwise EKF depth.
  double bestDepth() const { double d; if (p.use_bar30 && bar30Depth(d)) return d; return depth(); }

  bool depthStable(double window_s, double threshold) const {
    std::lock_guard<std::mutex> l(mtx_);
    double lo = 1e9, hi = -1e9, oldest = 0.0; int n = 0;
    for (const auto& h : depth_hist_) {
      const double age = secSince(h.first);
      if (age > window_s) continue;
      lo = std::min(lo, h.second); hi = std::max(hi, h.second);
      oldest = std::max(oldest, age); ++n;
    }
    return n >= 3 && oldest >= 0.8 * window_s && (hi - lo) <= threshold;
  }

  std::optional<GateDetection> freshGate(double max_age_s) const {
    std::lock_guard<std::mutex> l(mtx_);
    auto it = gates_.find(p.gate_camera_id);
    if (it == gates_.end() || secSince(it->second.second) > max_age_s) return std::nullopt;
    return it->second.first;
  }
  FloorInfo floorInfo(double timeout_s) const {
    std::lock_guard<std::mutex> l(mtx_);
    FloorInfo f; f.ever = have_floor_;
    if (have_floor_) {
      f.fresh = secSince(floor_t_) <= timeout_s;
      f.valid = floor_.valid;
      f.clearance = floor_.clearance_m;
    }
    return f;
  }
  bool batteryInfo(double& pct) const {
    std::lock_guard<std::mutex> l(mtx_);
    if (!have_batt_) return false;
    pct = batt_pct_; return true;
  }

  // ------------------------------------------------ mission start pose
  void captureStart() {
    std::lock_guard<std::mutex> l(mtx_);
    start_ = odom_.pose.pose; have_start_ = true;
  }
  bool hasStart() const { std::lock_guard<std::mutex> l(mtx_); return have_start_; }
  geometry_msgs::msg::Pose startPose() const { std::lock_guard<std::mutex> l(mtx_); return start_; }

  // ------------------------------------------------ setpoint commands (BT side)
  void commandHorizontal(uint8_t mode, double x, double y, double yaw) {
    std::lock_guard<std::mutex> l(mtx_);
    cmd_mode_ = mode; cmd_x_ = x; cmd_y_ = y; cmd_yaw_ = yaw; have_xy_ = true;
  }
  void commandDepth(double depth) {
    std::lock_guard<std::mutex> l(mtx_);
    cmd_depth_ = depth; have_cmd_depth_ = true;
  }

  // ------------------------------------------------ safety overrides (monitors)
  // Any active hold reason forces HOLD_POSITION at the pose where the first reason engaged.
  void setHold(const std::string& reason, bool on) {
    std::lock_guard<std::mutex> l(mtx_);
    if (on) {
      const bool was_empty = hold_reasons_.empty();
      hold_reasons_.insert(reason);
      if (was_empty && have_odom_) {
        hold_x_ = odom_.pose.pose.position.x;
        hold_y_ = odom_.pose.pose.position.y;
        hold_yaw_ = yawFromQuat(odom_.pose.pose.orientation);
      }
    } else {
      hold_reasons_.erase(reason);
    }
  }
  // Shallowest active override wins.
  void setDepthOverride(const std::string& reason, std::optional<double> depth) {
    std::lock_guard<std::mutex> l(mtx_);
    if (depth) depth_over_[reason] = *depth; else depth_over_.erase(reason);
  }
  void setSensorHealth(const std::string& s) { std::lock_guard<std::mutex> l(mtx_); health_ = s; }
  std::string sensorHealth() const { std::lock_guard<std::mutex> l(mtx_); return health_; }

  // ------------------------------------------------ merged output
  ControlSetpoint buildSetpoint(const builtin_interfaces::msg::Time& stamp) const {
    std::lock_guard<std::mutex> l(mtx_);
    ControlSetpoint m;
    m.header.stamp = stamp;
    m.header.frame_id = odom_.header.frame_id;

    double depth = have_odom_ ? depthOfZ(odom_.pose.pose.position.z) : 0.0;
    if (have_cmd_depth_) depth = cmd_depth_;
    if (!depth_over_.empty()) {
      depth = std::numeric_limits<double>::max();
      for (const auto& kv : depth_over_) depth = std::min(depth, kv.second);
    }

    uint8_t mode = ControlSetpoint::MODE_IDLE;
    double x = 0.0, y = 0.0, yaw = 0.0;
    if (!hold_reasons_.empty() && have_odom_) {
      mode = ControlSetpoint::MODE_HOLD_POSITION; x = hold_x_; y = hold_y_; yaw = hold_yaw_;
    } else if (have_xy_) {
      mode = cmd_mode_; x = cmd_x_; y = cmd_y_; yaw = cmd_yaw_;
    }
    m.control_mode = mode;
    m.desired_position.x = x;
    m.desired_position.y = y;
    m.desired_position.z = p.depth_is_minus_z ? -depth : depth;
    m.desired_depth = depth;
    m.desired_yaw = yaw;
    return m;
  }

private:
  double depthOfZ(double z) const { return p.depth_is_minus_z ? -z : z; }

  mutable std::mutex mtx_;
  nav_msgs::msg::Odometry odom_;      SClock::time_point odom_t_;  bool have_odom_{false};
  sensor_msgs::msg::Imu imu_;         SClock::time_point imu_t_;   bool have_imu_{false};
  double bar_pa_{0.0};                SClock::time_point bar_t_;   bool have_bar_{false};
  sauvc_interfaces::msg::FloorClearance floor_; SClock::time_point floor_t_; bool have_floor_{false};
  double batt_pct_{100.0};            bool have_batt_{false};
  std::map<std::string, std::pair<GateDetection, SClock::time_point>> gates_;
  std::deque<std::pair<SClock::time_point, double>> depth_hist_;

  geometry_msgs::msg::Pose start_;    bool have_start_{false};

  uint8_t cmd_mode_{ControlSetpoint::MODE_IDLE};
  double cmd_x_{0}, cmd_y_{0}, cmd_yaw_{0}, cmd_depth_{0};
  bool have_xy_{false}, have_cmd_depth_{false};

  std::set<std::string> hold_reasons_;
  double hold_x_{0}, hold_y_{0}, hold_yaw_{0};
  std::map<std::string, double> depth_over_;
  std::string health_{"NO_DATA"};
};

}  // namespace sauvc_bt
