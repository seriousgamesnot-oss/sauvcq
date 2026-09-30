#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

#include "sauvc_autonomy/sauvc_bt_nodes.hpp"

class SauvcBtExecutor : public rclcpp::Node {
public:
  SauvcBtExecutor() : Node("sauvc_bt_executor") {
    declare_parameter<std::string>("bt_xml_path", "");
    declare_parameter<double>("tick_rate_hz", 20.0);
    declare_parameter<double>("operational_depth", 1.0);
    declare_parameter<double>("min_safe_depth", 0.35);
    declare_parameter<double>("max_safe_depth", 2.40);
    declare_parameter<double>("vehicle_length_m", 0.65);

    declare_parameter<std::string>("ekf_topic", "/odometry/filtered");
    declare_parameter<std::string>("imu_topic", "/imu/data");
    declare_parameter<std::string>("bar30_topic", "");
    declare_parameter<std::string>("gate_topic", "/varuna/gate_detection");
    declare_parameter<std::string>("floor_topic", "");
    declare_parameter<std::string>("setpoint_topic", "/varuna/control_setpoint");
    declare_parameter<std::string>("battery_topic", "");

    declare_parameter<std::string>("gate_camera_id", "front");
    declare_parameter<bool>("gate_position_in_body_frame", true);
    declare_parameter<bool>("depth_is_minus_z", true);
    declare_parameter<double>("arrival_radius_m", 1.0);

    declare_parameter<double>("atm_pressure_pa", 101325.0);
    declare_parameter<double>("water_density", 997.0);
    declare_parameter<double>("ekf_timeout_s", 0.5);
    declare_parameter<double>("imu_timeout_s", 0.5);
    declare_parameter<double>("bar30_timeout_s", 0.5);

    declare_parameter<double>("pool_depth_m", -1.0);
    declare_parameter<double>("min_floor_clearance_m", -1.0);
    declare_parameter<double>("floor_safe_depth_m", -1.0);
    declare_parameter<double>("floor_data_timeout_s", 1.0);

    declare_parameter<double>("battery_warn_pct", 25.0);
    declare_parameter<double>("battery_critical_pct", 15.0);
    declare_parameter<std::string>("battery_critical_action", "hold");
  }

  bool setup() {
    const std::string xml_path = get_parameter("bt_xml_path").as_string();
    if (xml_path.empty()) {
      RCLCPP_FATAL(get_logger(), "bt_xml_path parameter is required!");
      return false;
    }
    const double tick_rate = get_parameter("tick_rate_hz").as_double();
    vehicle_length_ = get_parameter("vehicle_length_m").as_double();

    sauvc_bt::Params p;
    p.operational_depth = get_parameter("operational_depth").as_double();
    p.min_safe_depth = get_parameter("min_safe_depth").as_double();
    p.max_safe_depth = get_parameter("max_safe_depth").as_double();
    p.gate_camera_id = get_parameter("gate_camera_id").as_string();
    p.gate_position_in_body_frame = get_parameter("gate_position_in_body_frame").as_bool();
    p.depth_is_minus_z = get_parameter("depth_is_minus_z").as_bool();
    p.arrival_radius_m = get_parameter("arrival_radius_m").as_double();
    p.atm_pressure_pa = get_parameter("atm_pressure_pa").as_double();
    p.water_density = get_parameter("water_density").as_double();
    p.ekf_timeout_s = get_parameter("ekf_timeout_s").as_double();
    p.imu_timeout_s = get_parameter("imu_timeout_s").as_double();
    p.bar30_timeout_s = get_parameter("bar30_timeout_s").as_double();
    p.pool_depth_m = get_parameter("pool_depth_m").as_double();
    p.min_floor_clearance_m = get_parameter("min_floor_clearance_m").as_double();
    p.floor_safe_depth_m = get_parameter("floor_safe_depth_m").as_double();
    p.floor_data_timeout_s = get_parameter("floor_data_timeout_s").as_double();
    p.battery_warn_pct = get_parameter("battery_warn_pct").as_double();
    p.battery_critical_pct = get_parameter("battery_critical_pct").as_double();
    p.battery_critical_action = get_parameter("battery_critical_action").as_string();

    const std::string ekf_topic = get_parameter("ekf_topic").as_string();
    const std::string imu_topic = get_parameter("imu_topic").as_string();
    const std::string bar30_topic = get_parameter("bar30_topic").as_string();
    const std::string gate_topic = get_parameter("gate_topic").as_string();
    const std::string floor_topic = get_parameter("floor_topic").as_string();
    const std::string setpoint_topic = get_parameter("setpoint_topic").as_string();
    const std::string battery_topic = get_parameter("battery_topic").as_string();
    p.use_bar30 = !bar30_topic.empty();
    p.use_floor_camera = !floor_topic.empty();
    p.use_battery = !battery_topic.empty();

    // ---- safety parameter validation (no defaults for floor safety) ----
    if (p.pool_depth_m <= 0.0 || p.min_floor_clearance_m <= 0.0 || p.floor_safe_depth_m <= 0.0) {
      RCLCPP_FATAL(get_logger(),
        "Floor safety parameters must be set: pool_depth_m, min_floor_clearance_m, floor_safe_depth_m");
      return false;
    }
    const double max_allowed = p.pool_depth_m - p.min_floor_clearance_m;
    if (p.floor_safe_depth_m > max_allowed || p.operational_depth > max_allowed) {
      RCLCPP_FATAL(get_logger(),
        "operational_depth (%.2f) and floor_safe_depth_m (%.2f) must be <= pool_depth - min_clearance (%.2f)",
        p.operational_depth, p.floor_safe_depth_m, max_allowed);
      return false;
    }
    if (!(p.min_safe_depth < p.operational_depth && p.operational_depth < p.max_safe_depth)) {
      RCLCPP_FATAL(get_logger(), "Need min_safe_depth < operational_depth < max_safe_depth");
      return false;
    }
    if (!p.use_bar30)
      RCLCPP_WARN(get_logger(), "bar30_topic not set: depth watchdog/floor check use EKF depth only");
    if (!p.use_floor_camera)
      RCLCPP_WARN(get_logger(), "floor_topic not set: floor safety uses depth-only check");
    if (!p.use_battery)
      RCLCPP_WARN(get_logger(), "battery_topic not set: battery monitor inactive");

    state_ = std::make_shared<sauvc_bt::SharedState>(p);

    // ---- subscriptions (SensorDataQoS is compatible with reliable and best-effort publishers) ----
    const auto qos = rclcpp::SensorDataQoS();
    subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(ekf_topic, qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { state_->onOdom(*m); }));
    subs_.push_back(create_subscription<sensor_msgs::msg::Imu>(imu_topic, qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m) { state_->onImu(*m); }));
    subs_.push_back(create_subscription<sauvc_interfaces::msg::GateDetection>(gate_topic, qos,
      [this](sauvc_interfaces::msg::GateDetection::ConstSharedPtr m) { state_->onGate(*m); }));
    if (p.use_bar30)
      subs_.push_back(create_subscription<sensor_msgs::msg::FluidPressure>(bar30_topic, qos,
        [this](sensor_msgs::msg::FluidPressure::ConstSharedPtr m) { state_->onBar30(*m); }));
    if (p.use_floor_camera)
      subs_.push_back(create_subscription<sauvc_interfaces::msg::FloorClearance>(floor_topic, qos,
        [this](sauvc_interfaces::msg::FloorClearance::ConstSharedPtr m) { state_->onFloor(*m); }));
    if (p.use_battery)
      subs_.push_back(create_subscription<sensor_msgs::msg::BatteryState>(battery_topic, qos,
        [this](sensor_msgs::msg::BatteryState::ConstSharedPtr m) { state_->onBattery(*m); }));

    setpoint_pub_ = create_publisher<sauvc_interfaces::msg::ControlSetpoint>(setpoint_topic, 10);

    // ---- tree ----
    sauvc_bt::BtRosContext ctx{shared_from_this(), state_};
    sauvc_bt::registerSauvcNodes(factory_, ctx);
    try {
      tree_ = factory_.createTreeFromFile(xml_path);
    } catch (const std::exception& e) {
      RCLCPP_FATAL(get_logger(), "Failed to build tree: %s", e.what());
      return false;
    }

    auto bb = tree_.rootBlackboard();
    bb->set("operational_depth", p.operational_depth);
    bb->set("min_safe_depth", p.min_safe_depth);
    bb->set("max_safe_depth", p.max_safe_depth);
    bb->set("vehicle_length_m", vehicle_length_);
    bb->set("depth_tolerance", 0.15);
    bb->set("arrival_radius", p.arrival_radius_m);
    bb->set("gate_pass_count", 0);
    bb->set("mission_phase", std::string("INIT"));

    file_logger_ = std::make_unique<BT::FileLogger2>(tree_, "/tmp/sauvc_bt_log.btlog");
    groot2_pub_ = std::make_unique<BT::Groot2Publisher>(tree_);   // port 1667

    const auto period = std::chrono::duration<double>(1.0 / tick_rate);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this]() { tick(); });

    RCLCPP_INFO(get_logger(), "SAUVC BT Executor ready. %.1f Hz, operational depth %.2f m, pool %.2f m",
                tick_rate, p.operational_depth, p.pool_depth_m);
    return true;
  }

private:
  void tick() {
    syncBlackboard();

    if (!mission_done_) {
      // tickOnce, NOT tickWhileRunning: the latter blocks this callback and starves subscriptions.
      const BT::NodeStatus status = tree_.tickOnce();
      if (status == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(get_logger(), "Mission COMPLETE. Gate passes: %d",
                    tree_.rootBlackboard()->get<int>("gate_pass_count"));
        mission_done_ = true;
      } else if (status == BT::NodeStatus::FAILURE) {
        RCLCPP_ERROR(get_logger(), "Mission ABORTED: holding position. See /tmp/sauvc_bt_log.btlog");
        state_->setHold("mission_aborted", true);
        mission_done_ = true;
      }
    }

    if (state_->odomEver())
      setpoint_pub_->publish(state_->buildSetpoint(now()));   // setpoints keep flowing after the mission ends
  }

  void syncBlackboard() {
    auto bb = tree_.rootBlackboard();
    if (state_->odomEver()) {
      const auto pose = state_->pose();
      bb->set("current_pose", pose);
      bb->set("current_position", pose.position);
      bb->set("current_orientation", pose.orientation);
      bb->set("current_velocity", state_->linearVelocity());
      bb->set("covariance", state_->covariance());
      bb->set("current_depth", state_->depth());
    }
    if (state_->imuEver()) {
      const auto imu = state_->imu();
      const auto& q = imu.orientation;
      bb->set("filtered_roll", std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y)));
      bb->set("filtered_pitch", std::asin(std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0)));
      bb->set("filtered_yaw", sauvc_bt::yawFromQuat(q));
      bb->set("imu_accel", imu.linear_acceleration);
      bb->set("imu_gyro", imu.angular_velocity);
    }
    if (state_->bar30Ever()) bb->set("bar30_pressure", state_->bar30Pressure());
    bb->set("sensor_health", state_->sensorHealth());

    if (state_->hasStart() && state_->odomEver()) {
      const auto st = state_->startPose();
      const auto v = state_->pose();
      const double dist = std::hypot(v.position.x - st.position.x, v.position.y - st.position.y);
      bb->set("start_pose", st);
      bb->set("distance_to_start", dist);
      bb->set("at_start", dist < state_->p.arrival_radius_m);
    }

    const auto sp = state_->buildSetpoint(now());
    bb->set("desired_position", sp.desired_position);
    bb->set("desired_depth", sp.desired_depth);
    bb->set("target_depth", sp.desired_depth);
    bb->set("desired_yaw", sp.desired_yaw);
    const char* modes[] = {"IDLE", "GOTO_POSITION", "HOLD_POSITION"};
    bb->set("control_mode", std::string(sp.control_mode <= 2 ? modes[sp.control_mode] : "UNKNOWN"));
    bb->set("controller_status", std::string("UNKNOWN"));   // no controller status topic defined yet
  }

  double vehicle_length_{0.65};
  std::shared_ptr<sauvc_bt::SharedState> state_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp::Publisher<sauvc_interfaces::msg::ControlSetpoint>::SharedPtr setpoint_pub_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::unique_ptr<BT::FileLogger2> file_logger_;
  std::unique_ptr<BT::Groot2Publisher> groot2_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool mission_done_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SauvcBtExecutor>();
  if (!node->setup()) {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
