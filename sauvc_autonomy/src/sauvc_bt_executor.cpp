#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

#include "sauvc_autonomy/sauvc_bt_nodes.hpp"

using namespace std::chrono_literals;

class SauvcBtExecutor : public rclcpp::Node {
public:
  SauvcBtExecutor() : Node("sauvc_bt_executor") {
    this->declare_parameter<std::string>("bt_xml_path", "");
    this->declare_parameter<double>("tick_rate_hz",      20.0);
    this->declare_parameter<double>("operational_depth",  1.0);
    this->declare_parameter<double>("min_safe_depth",     0.35);
    this->declare_parameter<double>("max_safe_depth",     2.40);
    this->declare_parameter<double>("vehicle_length_m",   0.65);
  }

  void setup() {
    std::string xml_path       = this->get_parameter("bt_xml_path").as_string();
    double      tick_rate      = this->get_parameter("tick_rate_hz").as_double();
    double      op_depth       = this->get_parameter("operational_depth").as_double();
    double      min_depth      = this->get_parameter("min_safe_depth").as_double();
    double      max_depth      = this->get_parameter("max_safe_depth").as_double();
    double      vehicle_length = this->get_parameter("vehicle_length_m").as_double();

    if (xml_path.empty()) {
      RCLCPP_FATAL(this->get_logger(), "bt_xml_path parameter is required!");
      rclcpp::shutdown();
      return;
    }

    sauvc_bt::BtRosContext ctx{this->shared_from_this()};
    sauvc_bt::registerSauvcNodes(factory_, ctx);

    tree_ = factory_.createTreeFromFile(xml_path);

    auto bb = tree_.rootBlackboard();
    bb->set("operational_depth",  op_depth);
    bb->set("min_safe_depth",     min_depth);
    bb->set("max_safe_depth",     max_depth);
    bb->set("vehicle_length_m",   vehicle_length);
    bb->set("depth_tolerance",    0.15);
    bb->set("gate_pass_count",    0);
    bb->set("mission_phase",      std::string("INIT"));

    // Loggers
    // cout_logger_ = std::make_unique<BT::StdCoutLogger>(tree_); // Muted to stop console spam
    file_logger_  = std::make_unique<BT::FileLogger2>(tree_, "/tmp/sauvc_bt_log.btlog");
    
    // The Groot2 Live Publisher (Broadcasts on port 1667)
    groot2_pub_   = std::make_unique<BT::Groot2Publisher>(tree_);

    auto period = std::chrono::duration<double>(1.0 / tick_rate);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { this->tick(); });

    RCLCPP_INFO(this->get_logger(),
      "SAUVC BT Executor started. Ticking at %.1f Hz. Operational depth: %.2f m",
      tick_rate, op_depth);
  }

private:
  void tick() {
    if (mission_complete_) return;

    BT::NodeStatus status = tree_.tickWhileRunning();

    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Mission COMPLETE. Gate passes: %d",
        tree_.rootBlackboard()->get<int>("gate_pass_count"));
      mission_complete_ = true;
      timer_->cancel();
    } else if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_ERROR(this->get_logger(), "Mission ABORTED. Check /tmp/sauvc_bt_log.btlog");
      mission_complete_ = true;
      timer_->cancel();
    }
  }

  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::unique_ptr<BT::StdCoutLogger> cout_logger_;
  std::unique_ptr<BT::FileLogger2>   file_logger_;
  std::unique_ptr<BT::Groot2Publisher> groot2_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool mission_complete_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto executor_node = std::make_shared<SauvcBtExecutor>();
  executor_node->setup();
  rclcpp::spin(executor_node);
  rclcpp::shutdown();
  return 0;
}
