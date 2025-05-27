#include "RobotNode.h"

RobotNode::RobotNode()
    : Node("robot_node"),
      estop_active_(false),
      dms_active_(false),
      is_human_turn_(true),
      start_(false)//,
      //difficulty_(0)
{
    // Service
    start_service_ = this->create_service<std_srvs::srv::SetBool>(
        "ur3/start_signal",
        std::bind(&RobotNode::handle_start_service, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Subscribers
    estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "ur3/estop", 10,
        std::bind(&RobotNode::estop_callback, this, std::placeholders::_1));

    dms_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "ur3/dms", 10,
        std::bind(&RobotNode::dms_callback, this, std::placeholders::_1));

    turn_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "ur3/turn", 10,
        std::bind(&RobotNode::turn_callback, this, std::placeholders::_1));

    diff_sub_ = this->create_subscription<std_msgs::msg::String>(
        "ur3/diff", 10,
        std::bind(&RobotNode::diff_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "RobotNode initialized.");
}

void RobotNode::handle_start_service(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (request->data) {
        RCLCPP_INFO(this->get_logger(), "Received start signal!");
        response->success = true;
        response->message = "Robot received start signal.";
        start_ = true;
    } else {
        response->success = false;
        response->message = "Start signal not set.";
    }
}

void RobotNode::estop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (true || start_) {
        estop_active_.store(msg->data);
        RCLCPP_INFO(this->get_logger(), "E-Stop: %s", msg->data ? "ON" : "OFF");

    }
}

void RobotNode::dms_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (start_) {
        dms_active_.store(msg->data);
        RCLCPP_INFO(this->get_logger(), "Dead Man's Switch: %s", msg->data ? "ON" : "OFF");
    }
}

void RobotNode::turn_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (start_) {
        is_human_turn_.store(msg->data);
        RCLCPP_INFO(this->get_logger(), "Turn: %s", msg->data ? "HUMAN" : "ROBOT");
    }
}

void RobotNode::diff_callback(const std_msgs::msg::String::SharedPtr msg)
{
    if (start_) {
        difficulty_.store(std::stoi(msg->data));
        RCLCPP_INFO(this->get_logger(), "Difficulty: %s", msg->data.c_str());
    }
}