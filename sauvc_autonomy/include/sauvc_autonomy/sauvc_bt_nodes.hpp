#pragma once

#include <string>
#include <chrono>
#include <cmath>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"

namespace sauvc_bt {

struct BtRosContext {
  rclcpp::Node::SharedPtr node;
};

// ============================================================================
// PERCEPTION NODES
// ============================================================================

class IsGateVisible : public BT::ConditionNode {
public:
  IsGateVisible(const std::string& name, const BT::NodeConfig& config, BtRosContext ctx)
      : BT::ConditionNode(name, config), node_(ctx.node) {}
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("camera_topic", "/varuna/camera/front/compressed", ""),
      BT::InputPort<double>("min_confidence", 0.65, ""),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_gate_pose"),
    };
  }
  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
private:
  rclcpp::Node::SharedPtr node_;
};

class GetGatePose : public BT::SyncActionNode {
public:
  GetGatePose(const std::string& name, const BT::NodeConfig& config, BtRosContext ctx)
      : BT::SyncActionNode(name, config), node_(ctx.node) {}
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("camera_topic", "/varuna/camera/front/compressed", ""),
      BT::InputPort<std::string>("imu_topic", "/varuna/imu", ""),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_gate_pose"),
    };
  }
  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
private:
  rclcpp::Node::SharedPtr node_;
};

// ============================================================================
// NAVIGATION NODES
// ============================================================================

class IsAlignedWithGate : public BT::ConditionNode {
public:
  IsAlignedWithGate(const std::string& name, const BT::NodeConfig& config, BtRosContext ctx)
      : BT::ConditionNode(name, config), node_(ctx.node) {}
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("gate_pose"),
      BT::InputPort<double>("lateral_tolerance_m", 0.30, ""),
      BT::InputPort<double>("yaw_tolerance_deg", 8.0, ""),
    };
  }
  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
private:
  rclcpp::Node::SharedPtr node_;
};

class CalculateUTurnTrajectory : public BT::SyncActionNode {
public:
  CalculateUTurnTrajectory(const std::string& name, const BT::NodeConfig& config, BtRosContext ctx)
      : BT::SyncActionNode(name, config), node_(ctx.node) {}
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("imu_topic", "/varuna/imu", ""),
      BT::InputPort<double>("target_yaw_delta", 180.0, ""),
      BT::OutputPort<double>("output_target_yaw"),
    };
  }
  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
private:
  rclcpp::Node::SharedPtr node_;
};

class MoveToRelativePose : public BT::StatefulActionNode {
public:
  MoveToRelativePose(const std::string& name, const BT::NodeConfig& config, BtRosContext ctx)
      : BT::StatefulActionNode(name, config), node_(ctx.node) {}
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("gate_pose"),
      BT::InputPort<double>("approach_standoff", 1.5, ""),
      BT::InputPort<double>("lateral_kp", 0.6, ""),
      BT::InputPort<double>("yaw_kp", 0.4, ""),
      BT::InputPort<std::string>("mode", "COARSE", ""),
    };
  }
  BT::NodeStatus onStart() override { return BT::NodeStatus::SUCCESS; }
  BT::NodeStatus onRunning() override { return BT::NodeStatus::SUCCESS; }
  void onHalted() override {}
private:
  rclcpp::Node::SharedPtr node_;
};

class YawSweepSearch : public BT::StatefulActionNode {
public:
  YawSweepSearch(const std::string& name, const BT::NodeConfig& config, BtRosContext ctx)
      : BT::StatefulActionNode(name, config), node_(ctx.node) {}
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("sweep_angle_deg", 60.0, ""),
      BT::InputPort<double>("sweep_rate_deg_s", 8.0, ""),
      BT::InputPort<std::string>("camera_topic", "/varuna/camera/front/compressed", ""),
    };
  }
  BT::NodeStatus onStart() override { return BT::NodeStatus::SUCCESS; }
  BT::NodeStatus onRunning() override { return BT::NodeStatus::SUCCESS; }
  void onHalted() override {}
private:
  rclcpp::Node::SharedPtr node_;
};

class ExecuteYawRotation : public BT::StatefulActionNode {
public:
  ExecuteYawRotation(const std::string& name, const BT::NodeConfig& config, BtRosContext ctx)
      : BT::StatefulActionNode(name, config), node_(ctx.node) {}
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("target_yaw", 0.0, ""),
      BT::InputPort<double>("yaw_rate_deg_s", 20.0, ""),
      BT::InputPort<std::string>("imu_topic", "/varuna/imu", ""),
    };
  }
  BT::NodeStatus onStart() override { return BT::NodeStatus::SUCCESS; }
  BT::NodeStatus onRunning() override { return BT::NodeStatus::SUCCESS; }
  void onHalted() override {}
private:
  rclcpp::Node::SharedPtr node_;
};

// ============================================================================
// CONTROLS NODES
// ============================================================================

class MaintainDepth : public BT::StatefulActionNode {
public:
  MaintainDepth(const std::string& name, const BT::NodeConfig& config, BtRosContext ctx)
      : BT::StatefulActionNode(name, config), node_(ctx.node) {}
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("target_depth", 1.0, ""),
      BT::InputPort<double>("tolerance", 0.10, ""),
    };
  }
  BT::NodeStatus onStart() override { return BT::NodeStatus::RUNNING; }
  BT::NodeStatus onRunning() override { return BT::NodeStatus::RUNNING; }
  void onHalted() override {}
private:
  rclcpp::Node::SharedPtr node_;
};

class IsDepthStable : public BT::ConditionNode {
public:
  IsDepthStable(const std::string& name, const BT::NodeConfig& config, BtRosContext ctx)
      : BT::ConditionNode(name, config), node_(ctx.node) {}
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("depth_topic", "/varuna/depth", ""),
      BT::InputPort<double>("stable_threshold", 0.05, ""),
      BT::InputPort<double>("window_sec", 1.0, ""),
    };
  }
  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
private:
  rclcpp::Node::SharedPtr node_;
};

class IsWithinDepthBounds : public BT::ConditionNode {
public:
  IsWithinDepthBounds(const std::string& name, const BT::NodeConfig& config, BtRosContext ctx)
      : BT::ConditionNode(name, config), node_(ctx.node) {}
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("depth_topic", "/varuna/depth", ""),
      BT::InputPort<double>("min_depth", 0.30, ""),
      BT::InputPort<double>("max_depth", 2.50, ""),
    };
  }
  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
private:
  rclcpp::Node::SharedPtr node_;
};

class EmergencySurface : public BT::SyncActionNode {
public:
  EmergencySurface(const std::string& name, const BT::NodeConfig& config, BtRosContext ctx)
      : BT::SyncActionNode(name, config), node_(ctx.node) {}
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("reason", "Unknown", ""),
    };
  }
  BT::NodeStatus tick() override { return BT::NodeStatus::FAILURE; }
private:
  rclcpp::Node::SharedPtr node_;
};

// ============================================================================
// STATE NODES
// ============================================================================

class IsVehicleClearOfGate : public BT::ConditionNode {
public:
  IsVehicleClearOfGate(const std::string& name, const BT::NodeConfig& config, BtRosContext ctx)
      : BT::ConditionNode(name, config), node_(ctx.node) {}
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("gate_pose"),
      BT::InputPort<double>("vehicle_length", 0.60, ""),
      BT::InputPort<double>("clearance_margin", 0.20, ""),
    };
  }
  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
private:
  rclcpp::Node::SharedPtr node_;
};

class IsYawComplete : public BT::ConditionNode {
public:
  IsYawComplete(const std::string& name, const BT::NodeConfig& config, BtRosContext ctx)
      : BT::ConditionNode(name, config), node_(ctx.node) {}
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("current_yaw_topic", "/varuna/imu", ""),
      BT::InputPort<double>("target_yaw", 0.0, ""),
      BT::InputPort<double>("tolerance_deg", 3.0, ""),
    };
  }
  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
private:
  rclcpp::Node::SharedPtr node_;
};

// THE NEW NODE
class AlwaysRunning : public BT::StatefulActionNode {
public:
  AlwaysRunning(const std::string& name, const BT::NodeConfig& config, BtRosContext ctx)
      : BT::StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override { return BT::NodeStatus::RUNNING; }
  BT::NodeStatus onRunning() override { return BT::NodeStatus::RUNNING; }
  void onHalted() override {}
};

// ============================================================================
// REGISTRATION
// ============================================================================
inline void registerSauvcNodes(BT::BehaviorTreeFactory& factory, BtRosContext ctx) {
  factory.registerNodeType<IsGateVisible>("IsGateVisible", ctx);
  factory.registerNodeType<GetGatePose>("GetGatePose", ctx);
  factory.registerNodeType<IsAlignedWithGate>("IsAlignedWithGate", ctx);
  factory.registerNodeType<CalculateUTurnTrajectory>("CalculateUTurnTrajectory", ctx);
  factory.registerNodeType<MoveToRelativePose>("MoveToRelativePose", ctx);
  factory.registerNodeType<YawSweepSearch>("YawSweepSearch", ctx);
  factory.registerNodeType<ExecuteYawRotation>("ExecuteYawRotation", ctx);
  factory.registerNodeType<MaintainDepth>("MaintainDepth", ctx);
  factory.registerNodeType<IsDepthStable>("IsDepthStable", ctx);
  factory.registerNodeType<IsWithinDepthBounds>("IsWithinDepthBounds", ctx);
  factory.registerNodeType<EmergencySurface>("EmergencySurface", ctx);
  factory.registerNodeType<IsVehicleClearOfGate>("IsVehicleClearOfGate", ctx);
  factory.registerNodeType<IsYawComplete>("IsYawComplete", ctx);
  
  // Registering the new node
  factory.registerNodeType<AlwaysRunning>("AlwaysRunning", ctx);
}

} // namespace sauvc_bt
