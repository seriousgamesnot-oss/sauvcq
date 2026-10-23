#pragma once

#include <cmath>
#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "sauvc_autonomy/sauvc_state.hpp"

namespace sauvc_bt {

using ControlSetpoint = sauvc_interfaces::msg::ControlSetpoint;

struct BtRosContext {
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<SharedState> state;
};

template <typename T>
T getIn(BT::TreeNode& n, const std::string& key, T def) {
  T v;
  if (n.getInput<T>(key, v)) return v;
  return def;
}

constexpr double kDeg2Rad = M_PI / 180.0;

// Common base: gives every node the ROS node and the shared state.
template <typename Base>
class RosNode : public Base {
public:
  RosNode(const std::string& name, const BT::NodeConfig& cfg, BtRosContext ctx)
      : Base(name, cfg), node_(ctx.node), s_(ctx.state) {}
protected:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<SharedState> s_;
};

// Gate position (from perception) -> world/EKF-frame pose.
// ASSUMPTION: if gate_position_in_body_frame, detection.position is in the vehicle
// body frame (x forward, y left). No TF is used yet.
inline bool gatePoseFromDetection(const SharedState& s,
                                  const sauvc_interfaces::msg::GateDetection& d,
                                  geometry_msgs::msg::PoseStamped& out) {
  if (!d.has_position) return false;
  const auto v = s.pose();
  const double yaw = yawFromQuat(v.orientation);
  double wx = d.position.x, wy = d.position.y;
  if (s.p.gate_position_in_body_frame) {
    wx = v.position.x + std::cos(yaw) * d.position.x - std::sin(yaw) * d.position.y;
    wy = v.position.y + std::sin(yaw) * d.position.x + std::cos(yaw) * d.position.y;
  }
  out.header.frame_id = "ekf";
  out.pose.position.x = wx;
  out.pose.position.y = wy;
  out.pose.position.z = v.position.z;
  out.pose.orientation.w = 1.0;
  return true;
}

inline void toBody(const geometry_msgs::msg::Pose& v, double wx, double wy, double& bx, double& by) {
  const double yaw = yawFromQuat(v.orientation);
  const double dx = wx - v.position.x, dy = wy - v.position.y;
  bx = std::cos(yaw) * dx + std::sin(yaw) * dy;
  by = -std::sin(yaw) * dx + std::cos(yaw) * dy;
}

// ============================================================================
// PERCEPTION
// ============================================================================
class IsGateVisible : public RosNode<BT::ConditionNode> {
public:
  using RosNode<BT::ConditionNode>::RosNode;
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("min_confidence", 0.65, "Minimum detection confidence"),
      BT::InputPort<double>("max_age_s", 0.5, "Maximum detection age"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_gate_pose"),
    };
  }
  BT::NodeStatus tick() override {
    auto det = s_->freshGate(getIn<double>(*this, "max_age_s", 0.5));
    if (!det) return BT::NodeStatus::FAILURE;
    if (det->confidence < getIn<double>(*this, "min_confidence", 0.65))
      return BT::NodeStatus::FAILURE;
    geometry_msgs::msg::PoseStamped g;
    if (!gatePoseFromDetection(*s_, *det, g)) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "Gate detection has no 3D position (has_position=false); bbox-only is not supported yet");
      return BT::NodeStatus::FAILURE;
    }
    setOutput("output_gate_pose", g);
    return BT::NodeStatus::SUCCESS;
  }
};

// ============================================================================
// NAVIGATION
// ============================================================================
class IsAlignedWithGate : public RosNode<BT::ConditionNode> {
public:
  using RosNode<BT::ConditionNode>::RosNode;
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("gate_pose"),
      BT::InputPort<double>("lateral_tolerance_m", 0.30, ""),
      BT::InputPort<double>("yaw_tolerance_deg", 8.0, ""),
    };
  }
  // "Aligned" = gate centre is ahead within the lateral and bearing tolerances.
  // (Perception gives no gate orientation, so this is bearing-based.)
  BT::NodeStatus tick() override {
    auto g = getInput<geometry_msgs::msg::PoseStamped>("gate_pose");
    if (!g || !s_->odomFresh()) return BT::NodeStatus::FAILURE;
    double bx, by;
    toBody(s_->pose(), g.value().pose.position.x, g.value().pose.position.y, bx, by);
    const double bearing = std::fabs(std::atan2(by, bx));
    const bool ok = std::fabs(by) <= getIn<double>(*this, "lateral_tolerance_m", 0.30) &&
                    bearing <= getIn<double>(*this, "yaw_tolerance_deg", 8.0) * kDeg2Rad;
    return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class CalculateUTurnTrajectory : public RosNode<BT::SyncActionNode> {
public:
  using RosNode<BT::SyncActionNode>::RosNode;
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("target_yaw_delta", 180.0, "degrees, signed"),
      BT::OutputPort<double>("output_target_yaw"),
    };
  }
  BT::NodeStatus tick() override {
    if (!s_->odomFresh()) return BT::NodeStatus::FAILURE;
    const double delta = getIn<double>(*this, "target_yaw_delta", 180.0) * kDeg2Rad;
    setOutput("output_target_yaw", wrapPi(s_->yaw() + delta));
    return BT::NodeStatus::SUCCESS;
  }
};

class MoveToRelativePose : public RosNode<BT::StatefulActionNode> {
public:
  using RosNode<BT::StatefulActionNode>::RosNode;
  static BT::PortsList providedPorts() {
    return {
      BT::BidirectionalPort<geometry_msgs::msg::PoseStamped>("gate_pose"),
      BT::InputPort<double>("approach_standoff", 1.5, "m before gate; negative = beyond gate"),
      BT::InputPort<std::string>("mode", "COARSE", "COARSE | FINE | PUSH"),
      BT::InputPort<double>("position_tolerance_m", -1.0, "<0 uses mode default"),
      BT::InputPort<double>("min_confidence", 0.65, ""),
      BT::InputPort<double>("timeout_s", 60.0, ""),
    };
  }
  BT::NodeStatus onStart() override {
    mode_ = getIn<std::string>(*this, "mode", "COARSE");
    t0_ = SClock::now();
    auto g = getInput<geometry_msgs::msg::PoseStamped>("gate_pose");
    if (!g || !s_->odomFresh()) return BT::NodeStatus::FAILURE;
    gx_ = g.value().pose.position.x; gy_ = g.value().pose.position.y;
    const auto v = s_->pose();
    updateTarget(v);
    return onRunning();
  }
  BT::NodeStatus onRunning() override {
    if (secSince(t0_) > getIn<double>(*this, "timeout_s", 60.0)) {
      RCLCPP_ERROR(node_->get_logger(), "MoveToRelativePose(%s) timed out", mode_.c_str());
      return BT::NodeStatus::FAILURE;
    }
    if (!s_->odomFresh()) return BT::NodeStatus::RUNNING;
    const auto v = s_->pose();
    if (mode_ != "PUSH") {
      auto det = s_->freshGate(0.5);
      geometry_msgs::msg::PoseStamped g;
      if (det && det->confidence >= getIn<double>(*this, "min_confidence", 0.65) &&
          gatePoseFromDetection(*s_, *det, g)) {
        gx_ = g.pose.position.x; gy_ = g.pose.position.y;
        setOutput("gate_pose", g);
      }
      updateTarget(v);
    }
    s_->commandHorizontal(ControlSetpoint::MODE_GOTO_POSITION, tx_, ty_, tyaw_);
    const double def_tol = (mode_ == "FINE") ? 0.15 : 0.30;
    double tol = getIn<double>(*this, "position_tolerance_m", -1.0);
    if (tol < 0.0) tol = def_tol;
    const double err = std::hypot(tx_ - v.position.x, ty_ - v.position.y);
    return err <= tol ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }
  void onHalted() override {}
private:
  void updateTarget(const geometry_msgs::msg::Pose& v) {
    const double standoff = getIn<double>(*this, "approach_standoff", 1.5);
    double dx = gx_ - v.position.x, dy = gy_ - v.position.y;
    double d = std::hypot(dx, dy);
    if (d < 1e-3) d = 1e-3;
    const double ux = dx / d, uy = dy / d;
    tx_ = gx_ - standoff * ux;
    ty_ = gy_ - standoff * uy;
    tyaw_ = std::atan2(dy, dx);
  }
  std::string mode_;
  double gx_{0}, gy_{0}, tx_{0}, ty_{0}, tyaw_{0};
  SClock::time_point t0_;
};

class YawSweepSearch : public RosNode<BT::StatefulActionNode> {
public:
  using RosNode<BT::StatefulActionNode>::RosNode;
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("sweep_angle_deg", 60.0, "+/- sweep amplitude"),
      BT::InputPort<double>("sweep_rate_deg_s", 8.0, ""),
      BT::InputPort<double>("timeout_s", 120.0, ""),
    };
  }
  BT::NodeStatus onStart() override {
    if (!s_->odomFresh()) return BT::NodeStatus::FAILURE;
    const auto v = s_->pose();
    x_ = v.position.x; y_ = v.position.y; yaw0_ = yawFromQuat(v.orientation);
    t0_ = SClock::now();
    return onRunning();
  }
  BT::NodeStatus onRunning() override {
    const double t = secSince(t0_);
    if (t > getIn<double>(*this, "timeout_s", 120.0)) {
      RCLCPP_ERROR(node_->get_logger(), "YawSweepSearch: gate not found before timeout");
      return BT::NodeStatus::FAILURE;
    }
    const double A = std::max(1e-3, getIn<double>(*this, "sweep_angle_deg", 60.0) * kDeg2Rad);
    const double rate = std::max(1e-3, getIn<double>(*this, "sweep_rate_deg_s", 8.0) * kDeg2Rad);
    const double ph = std::fmod(rate * t, 4.0 * A);       // triangle wave: 0 -> +A -> -A -> 0
    double off = ph < A ? ph : (ph < 3.0 * A ? 2.0 * A - ph : ph - 4.0 * A);
    s_->commandHorizontal(ControlSetpoint::MODE_GOTO_POSITION, x_, y_, wrapPi(yaw0_ + off));
    return BT::NodeStatus::RUNNING;
  }
  void onHalted() override {}
private:
  double x_{0}, y_{0}, yaw0_{0};
  SClock::time_point t0_;
};

class ExecuteYawRotation : public RosNode<BT::StatefulActionNode> {
public:
  using RosNode<BT::StatefulActionNode>::RosNode;
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("target_yaw", 0.0, "rad"),
      BT::InputPort<double>("yaw_rate_deg_s", 20.0, ""),
      BT::InputPort<double>("tolerance_deg", 3.0, ""),
      BT::InputPort<std::string>("direction", "CCW", "CCW | CW"),
    };
  }
  BT::NodeStatus onStart() override {
    if (!s_->odomFresh()) return BT::NodeStatus::FAILURE;
    const auto v = s_->pose();
    hx_ = v.position.x; hy_ = v.position.y; start_yaw_ = yawFromQuat(v.orientation);
    const double target = getIn<double>(*this, "target_yaw", start_yaw_);
    double d = std::fmod(target - start_yaw_, 2.0 * M_PI);
    if (getIn<std::string>(*this, "direction", "CCW") == "CCW") { if (d < 0.0) d += 2.0 * M_PI; }
    else { if (d > 0.0) d -= 2.0 * M_PI; }
    total_ = d;
    t0_ = SClock::now();
    return onRunning();
  }
  BT::NodeStatus onRunning() override {
    const double rate = std::max(1e-3, getIn<double>(*this, "yaw_rate_deg_s", 20.0) * kDeg2Rad);
    const double tol = getIn<double>(*this, "tolerance_deg", 3.0) * kDeg2Rad;
    const double t = secSince(t0_);
    const double mag = std::fabs(total_);
    const double prog = std::min(rate * t, mag);
    const double cmd = wrapPi(start_yaw_ + (total_ >= 0.0 ? 1.0 : -1.0) * prog);
    s_->commandHorizontal(ControlSetpoint::MODE_HOLD_POSITION, hx_, hy_, cmd);
    const double final_yaw = wrapPi(start_yaw_ + total_);
    if (prog >= mag && std::fabs(wrapPi(final_yaw - s_->yaw())) <= tol) return BT::NodeStatus::SUCCESS;
    if (t > mag / rate + 20.0) {
      RCLCPP_ERROR(node_->get_logger(), "ExecuteYawRotation timed out");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }
  void onHalted() override {}
private:
  double hx_{0}, hy_{0}, start_yaw_{0}, total_{0};
  SClock::time_point t0_;
};

class ReturnToStart : public RosNode<BT::StatefulActionNode> {
public:
  using RosNode<BT::StatefulActionNode>::RosNode;
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("arrival_radius_m", 1.0, "horizontal radius"),
      BT::InputPort<double>("timeout_s", 180.0, ""),
    };
  }
  BT::NodeStatus onStart() override {
    if (!s_->hasStart()) return BT::NodeStatus::FAILURE;
    t0_ = SClock::now();
    return onRunning();
  }
  BT::NodeStatus onRunning() override {
    if (secSince(t0_) > getIn<double>(*this, "timeout_s", 180.0)) {
      RCLCPP_ERROR(node_->get_logger(), "ReturnToStart timed out");
      return BT::NodeStatus::FAILURE;
    }
    const auto st = s_->startPose();
    s_->commandHorizontal(ControlSetpoint::MODE_GOTO_POSITION,
                          st.position.x, st.position.y, yawFromQuat(st.orientation));
    if (!s_->odomFresh()) return BT::NodeStatus::RUNNING;
    const auto v = s_->pose();
    // Horizontal distance only: the start pose was captured at arming depth,
    // the vehicle keeps its operational depth on the way back.
    const double dist = std::hypot(v.position.x - st.position.x, v.position.y - st.position.y);
    return dist < getIn<double>(*this, "arrival_radius_m", 1.0) ? BT::NodeStatus::SUCCESS
                                                               : BT::NodeStatus::RUNNING;
  }
  void onHalted() override {}
private:
  SClock::time_point t0_;
};

class HoldPosition : public RosNode<BT::SyncActionNode> {
public:
  using RosNode<BT::SyncActionNode>::RosNode;
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    if (!s_->hasStart()) return BT::NodeStatus::FAILURE;
    const auto st = s_->startPose();
    s_->commandHorizontal(ControlSetpoint::MODE_HOLD_POSITION,
                          st.position.x, st.position.y, yawFromQuat(st.orientation));
    RCLCPP_INFO(node_->get_logger(), "HOLD_POSITION at start pose");
    return BT::NodeStatus::SUCCESS;
  }
};

// ============================================================================
// CONTROLS / STATE
// ============================================================================
class MaintainDepth : public RosNode<BT::StatefulActionNode> {
public:
  using RosNode<BT::StatefulActionNode>::RosNode;
  static BT::PortsList providedPorts() {
    return { BT::InputPort<double>("target_depth", 1.0, "m, positive down") };
  }
  BT::NodeStatus onStart() override { return step(); }
  BT::NodeStatus onRunning() override { return step(); }
  void onHalted() override {}
private:
  BT::NodeStatus step() {
    if (s_->hasStart()) s_->commandDepth(getIn<double>(*this, "target_depth", 1.0));
    return BT::NodeStatus::RUNNING;    // only commands depth after the start pose is captured
  }
};

class WaitForDepth : public RosNode<BT::StatefulActionNode> {
public:
  using RosNode<BT::StatefulActionNode>::RosNode;
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("target_depth", 1.0, ""),
      BT::InputPort<double>("tolerance", 0.10, ""),
      BT::InputPort<double>("window_sec", 1.5, ""),
      BT::InputPort<double>("stable_threshold", 0.05, ""),
      BT::InputPort<double>("timeout_s", 60.0, ""),
    };
  }
  BT::NodeStatus onStart() override { t0_ = SClock::now(); return onRunning(); }
  BT::NodeStatus onRunning() override {
    const double target = getIn<double>(*this, "target_depth", 1.0);
    s_->commandDepth(target);
    if (std::fabs(s_->bestDepth() - target) <= getIn<double>(*this, "tolerance", 0.10) &&
        s_->depthStable(getIn<double>(*this, "window_sec", 1.5),
                        getIn<double>(*this, "stable_threshold", 0.05)))
      return BT::NodeStatus::SUCCESS;
    if (secSince(t0_) > getIn<double>(*this, "timeout_s", 60.0)) {
      RCLCPP_ERROR(node_->get_logger(), "WaitForDepth timed out (depth %.2f, target %.2f)",
                   s_->bestDepth(), target);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }
  void onHalted() override {}
private:
  SClock::time_point t0_;
};

class IsYawComplete : public RosNode<BT::ConditionNode> {
public:
  using RosNode<BT::ConditionNode>::RosNode;
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("target_yaw", "rad"),
      BT::InputPort<double>("tolerance_deg", 3.0, ""),
    };
  }
  BT::NodeStatus tick() override {
    auto t = getInput<double>("target_yaw");
    if (!t || !s_->odomFresh()) return BT::NodeStatus::FAILURE;
    return std::fabs(wrapPi(t.value() - s_->yaw())) <=
           getIn<double>(*this, "tolerance_deg", 3.0) * kDeg2Rad
           ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class IsVehicleClearOfGate : public RosNode<BT::ConditionNode> {
public:
  using RosNode<BT::ConditionNode>::RosNode;
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("gate_pose"),
      BT::InputPort<double>("vehicle_length", 0.60, ""),
      BT::InputPort<double>("clearance_margin", 0.20, ""),
    };
  }
  BT::NodeStatus tick() override {
    auto g = getInput<geometry_msgs::msg::PoseStamped>("gate_pose");
    if (!g || !s_->odomFresh()) return BT::NodeStatus::FAILURE;
    double bx, by;
    toBody(s_->pose(), g.value().pose.position.x, g.value().pose.position.y, bx, by);
    const double need = 0.5 * getIn<double>(*this, "vehicle_length", 0.60) +
                        getIn<double>(*this, "clearance_margin", 0.20);
    return bx < -need ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;   // gate is behind us
  }
};

// ============================================================================
// PRE-FLIGHT
// ============================================================================
class WaitForSensors : public RosNode<BT::StatefulActionNode> {
public:
  using RosNode<BT::StatefulActionNode>::RosNode;
  static BT::PortsList providedPorts() { return { BT::InputPort<double>("timeout_s", 20.0, "") }; }
  BT::NodeStatus onStart() override { t0_ = SClock::now(); return onRunning(); }
  BT::NodeStatus onRunning() override {
    if (s_->odomFresh() && s_->imuFresh() && (!s_->p.use_bar30 || s_->bar30Fresh()))
      return BT::NodeStatus::SUCCESS;
    if (secSince(t0_) > getIn<double>(*this, "timeout_s", 20.0)) {
      RCLCPP_ERROR(node_->get_logger(), "Sensors not ready: ekf=%d imu=%d bar30=%d",
                   s_->odomFresh(), s_->imuFresh(), s_->p.use_bar30 ? s_->bar30Fresh() : true);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }
  void onHalted() override {}
private:
  SClock::time_point t0_;
};

class CaptureStartPose : public RosNode<BT::SyncActionNode> {
public:
  using RosNode<BT::SyncActionNode>::RosNode;
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    if (!s_->odomFresh()) return BT::NodeStatus::FAILURE;
    s_->captureStart();
    const auto st = s_->startPose();
    s_->commandHorizontal(ControlSetpoint::MODE_HOLD_POSITION,
                          st.position.x, st.position.y, yawFromQuat(st.orientation));
    RCLCPP_INFO(node_->get_logger(), "Start pose captured: x=%.2f y=%.2f z=%.2f yaw=%.1f deg",
                st.position.x, st.position.y, st.position.z, yawFromQuat(st.orientation) / kDeg2Rad);
    return BT::NodeStatus::SUCCESS;
  }
};

// ============================================================================
// MONITORS (always RUNNING; FAILURE only for a fatal fault)
// ============================================================================
class DepthMonitor : public RosNode<BT::StatefulActionNode> {
public:
  using RosNode<BT::StatefulActionNode>::RosNode;
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("min_depth", 0.35, ""),
      BT::InputPort<double>("max_depth", 2.40, ""),
      BT::InputPort<double>("operational_depth", 1.0, ""),
    };
  }
  BT::NodeStatus onStart() override { return check(); }
  BT::NodeStatus onRunning() override { return check(); }
  void onHalted() override {}
private:
  BT::NodeStatus check() {
    if (!s_->odomFresh() && !(s_->p.use_bar30 && s_->bar30Fresh())) return BT::NodeStatus::RUNNING;
    const double lo = getIn<double>(*this, "min_depth", 0.35);
    const double hi = getIn<double>(*this, "max_depth", 2.40);
    const double op = getIn<double>(*this, "operational_depth", 1.0);
    const double d = s_->bestDepth();
    const bool bad = d < lo || d > hi;
    if (bad && !active_)
      RCLCPP_WARN(node_->get_logger(), "Depth %.2f outside [%.2f, %.2f]: commanding %.2f", d, lo, hi, op);
    if (!bad && active_) RCLCPP_INFO(node_->get_logger(), "Depth back within bounds");
    active_ = bad;
    s_->setDepthOverride("depth_bounds", bad ? std::optional<double>(std::clamp(op, lo, hi)) : std::nullopt);
    return BT::NodeStatus::RUNNING;
  }
  bool active_{false};
};

class FloorClearanceMonitor : public RosNode<BT::StatefulActionNode> {
public:
  using RosNode<BT::StatefulActionNode>::RosNode;
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override { return check(); }
  BT::NodeStatus onRunning() override { return check(); }
  void onHalted() override {}
private:
  BT::NodeStatus check() {
    const auto& p = s_->p;
    if (!s_->odomFresh()) return BT::NodeStatus::RUNNING;
    const double depth = s_->bestDepth();
    double floor_depth = p.pool_depth_m;
    bool cam_bad = false;
    if (p.use_floor_camera) {
      const auto f = s_->floorInfo(p.floor_data_timeout_s);
      if (f.fresh && f.valid) floor_depth = std::min(floor_depth, depth + f.clearance);
      else if (f.ever) cam_bad = true;                     // stale or invalid -> treat as unsafe
    }
    const double clearance = floor_depth - depth;
    // Latched: once triggered, release only when the OPERATING depth would be safe again.
    const double at_op = floor_depth - p.operational_depth;
    const bool viol = active_ ? (at_op < p.min_floor_clearance_m + 0.10 || cam_bad)
                              : (clearance < p.min_floor_clearance_m || cam_bad);
    if (viol && !active_)
      RCLCPP_WARN(node_->get_logger(), "FLOOR SAFETY: clearance %.2f m (min %.2f)%s -> hold + climb to %.2f m",
                  clearance, p.min_floor_clearance_m, cam_bad ? ", bottom-camera data bad" : "",
                  p.floor_safe_depth_m);
    if (!viol && active_) RCLCPP_INFO(node_->get_logger(), "Floor clearance recovered");
    active_ = viol;
    s_->setHold("floor", viol);
    s_->setDepthOverride("floor", viol ? std::optional<double>(p.floor_safe_depth_m) : std::nullopt);
    return BT::NodeStatus::RUNNING;
  }
  bool active_{false};
};

class BatteryMonitor : public RosNode<BT::StatefulActionNode> {
public:
  using RosNode<BT::StatefulActionNode>::RosNode;
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override { return check(); }
  BT::NodeStatus onRunning() override { return check(); }
  void onHalted() override {}
private:
  BT::NodeStatus check() {
    const auto& p = s_->p;
    double pct;
    if (!p.use_battery || !s_->batteryInfo(pct)) return BT::NodeStatus::RUNNING;
    if (pct <= p.battery_critical_pct) {
      if (!crit_) { crit_ = true; RCLCPP_ERROR(node_->get_logger(), "BATTERY CRITICAL %.0f%% (action: %s)", pct, p.battery_critical_action.c_str()); }
      if (p.battery_critical_action == "abort") return BT::NodeStatus::FAILURE;
      if (p.battery_critical_action == "hold") s_->setHold("battery", true);
    } else if (pct <= p.battery_warn_pct) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000, "Battery low: %.0f%%", pct);
    }
    return BT::NodeStatus::RUNNING;
  }
  bool crit_{false};
};

class SensorHealthMonitor : public RosNode<BT::StatefulActionNode> {
public:
  using RosNode<BT::StatefulActionNode>::RosNode;
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override { return check(); }
  BT::NodeStatus onRunning() override { return check(); }
  void onHalted() override {}
private:
  BT::NodeStatus check() {
    std::string bad;
    if (s_->odomEver() && !s_->odomFresh()) bad += "ekf ";
    if (s_->imuEver() && !s_->imuFresh()) bad += "imu ";
    if (s_->p.use_bar30 && s_->bar30Ever() && !s_->bar30Fresh()) bad += "bar30 ";
    s_->setSensorHealth(bad.empty() ? "OK" : "STALE: " + bad);
    if (s_->odomEver() && s_->odomAge() > 2.0) {
      RCLCPP_ERROR(node_->get_logger(), "EKF lost for %.1f s: aborting mission", s_->odomAge());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }
};

// ============================================================================
// REGISTRATION
// ============================================================================
inline void registerSauvcNodes(BT::BehaviorTreeFactory& f, BtRosContext ctx) {
  f.registerNodeType<IsGateVisible>("IsGateVisible", ctx);
  f.registerNodeType<IsAlignedWithGate>("IsAlignedWithGate", ctx);
  f.registerNodeType<CalculateUTurnTrajectory>("CalculateUTurnTrajectory", ctx);
  f.registerNodeType<MoveToRelativePose>("MoveToRelativePose", ctx);
  f.registerNodeType<YawSweepSearch>("YawSweepSearch", ctx);
  f.registerNodeType<ExecuteYawRotation>("ExecuteYawRotation", ctx);
  f.registerNodeType<ReturnToStart>("ReturnToStart", ctx);
  f.registerNodeType<HoldPosition>("HoldPosition", ctx);
  f.registerNodeType<MaintainDepth>("MaintainDepth", ctx);
  f.registerNodeType<WaitForDepth>("WaitForDepth", ctx);
  f.registerNodeType<IsYawComplete>("IsYawComplete", ctx);
  f.registerNodeType<IsVehicleClearOfGate>("IsVehicleClearOfGate", ctx);
  f.registerNodeType<WaitForSensors>("WaitForSensors", ctx);
  f.registerNodeType<CaptureStartPose>("CaptureStartPose", ctx);
  f.registerNodeType<DepthMonitor>("DepthMonitor", ctx);
  f.registerNodeType<FloorClearanceMonitor>("FloorClearanceMonitor", ctx);
  f.registerNodeType<BatteryMonitor>("BatteryMonitor", ctx);
  f.registerNodeType<SensorHealthMonitor>("SensorHealthMonitor", ctx);
}

}  // namespace sauvc_bt
