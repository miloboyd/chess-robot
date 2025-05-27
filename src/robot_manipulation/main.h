/**
 * @file main.h
 * @brief Main header file for the Chess Robot system
 * @author Your Name
 * @date 2025
*/

#ifndef MAIN_H
#define MAIN_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <memory>
#include <atomic>
#include <thread>
#include <string>
#include <vector>

#include "robotControl.h"
#include "boardPos.h"
#include "../GUI/GUI.h"
#include "safetyManager.h"


// Forward declarations
class RobotControl;
class BoardPos;

/**
 * @brief Main ROS2 node that coordinates the entire chess robot system
 */
class Main : public rclcpp::Node {
public:
    Main();
    ~Main() = default;
    
    /**
     * @brief Main execution function - initializes robot and starts event loop
     */
    void run();

private:
    // Component classes
    std::unique_ptr<RobotControl> robot_control_;
    std::unique_ptr<BoardPos> board_pos_;
    std::unique_ptr<GUI> GUI_;
    std::unique_ptr<SafetyManager> safety_manager_;
    
    
    // Atomic state variables for thread safety
    std::atomic<bool> game_active_{false};
    std::atomic<bool> robot_busy_{false};
    
    // ROS2 Service server
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_game_service_;
    
    // ROS2 Subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ai_move_subscriber_;
    
    // ROS2 Publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr move_complete_publisher_;
    
    // Service callback
    void startGameCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    // Subscriber callbacks
    void aiMoveCallback(const std_msgs::msg::String::SharedPtr msg);
    
    // Internal methods
    bool executeMove(const std::string& notation);
    void publishMoveComplete(bool success);
    bool initialiseRobot();
};

#endif // MAIN_H