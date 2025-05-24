#ifndef ROBOT_NODE_HPP
#define ROBOT_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <mutex>
#include <atomic>

class RobotNode : public rclcpp::Node
{
public:
    RobotNode();

private:
    // Service
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_service_;

    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dms_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr turn_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;

    // Internal state
    std::atomic<bool> estop_active_;
    std::atomic<bool> dms_active_;
    std::atomic<bool> is_human_turn_;
    std::atomic<bool> start_;
    int difficulty_;

    std::mutex state_mutex_;

    // Callbacks
    void handle_start_service(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    void estop_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void dms_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void turn_callback(const std_msgs::msg::Bool::SharedPtr msg);
};

#endif // ROBOT_NODE_HPP
