/**
 * @file safetyManager.h
 * @brief Safety management system matching GUI interface expectations
 * @author Your Name
 * @date 2025
 */

 #ifndef SAFETY_MANAGER_H
 #define SAFETY_MANAGER_H
 
 #include <rclcpp/rclcpp.hpp>
 #include <std_msgs/msg/bool.hpp>
 #include <atomic>
 #include <chrono>
 #include <string>
 
 /**
  * @brief Safety manager that interfaces with GUI safety controls
  * 
  * This class subscribes to the exact topics your GUI publishes and
  * manages safety state according to your GUI's behavior.
  */
 class SafetyManager {
 public:
     explicit SafetyManager(rclcpp::Node::SharedPtr node);
     ~SafetyManager() = default;
     
     /**
      * @brief Check if deadman switch (DMS) is currently active
      * @return True if space bar is being held (DMS active)
      */
     bool isDMSActive() const { return dms_active_; }
     
     /**
      * @brief Check if emergency stop has been triggered
      * @return True if e-stop button was pressed in GUI
      */
     bool isEstopTriggered() const { return estop_triggered_; }
     
     /**
      * @brief Check if it's safe to operate the robot
      * @return True if DMS is active AND e-stop is not triggered
      */
     bool isSafeToOperate() const { 
         return dms_active_ && !estop_triggered_; 
     }
     
     /**
      * @brief Check if it's currently robot's turn
      * @return True if GUI has set turn to robot
      */
     bool isRobotTurn() const { return is_robot_turn_; }
     
     /**
      * @brief Check if robot can move (safe + robot's turn)
      * @return True if safe to operate AND it's robot's turn
      */
     bool canRobotMove() const {
         return isSafeToOperate() && isRobotTurn();
     }
     
     /**
      * @brief Get current safety status as string for logging/publishing
      */
     std::string getSafetyStatus() const;
     
     /**
      * @brief Get detailed status for debugging
      */
     std::string getDetailedStatus() const;
 
 private:
     // Core components
     rclcpp::Node::SharedPtr node_;
     
     // Safety state (atomic for thread safety)
     std::atomic<bool> dms_active_{false};        // Dead Man's Switch (space bar)
     std::atomic<bool> estop_triggered_{false};   // Emergency Stop button
     std::atomic<bool> is_robot_turn_{false};     // Turn management
     
     // ROS2 Subscribers (matching your GUI's publishers)
     rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dms_subscriber_;
     rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_subscriber_;
     rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr turn_subscriber_;
     
     // Safety monitoring
     rclcpp::TimerBase::SharedPtr safety_monitor_timer_;
     std::chrono::steady_clock::time_point last_dms_update_;
     std::chrono::steady_clock::time_point last_safety_check_;
     
     // Timeout constants
     static constexpr std::chrono::milliseconds DMS_TIMEOUT{1000};  // Quick timeout for DMS
     static constexpr std::chrono::milliseconds SAFETY_CHECK_INTERVAL{50}; // Check every 50ms
     
     // Subscriber callbacks (matching GUI behavior)
     void dmsCallback(const std_msgs::msg::Bool::SharedPtr msg);
     void estopCallback(const std_msgs::msg::Bool::SharedPtr msg);
     void turnCallback(const std_msgs::msg::Bool::SharedPtr msg);
     
     /**
      * @brief Periodic safety monitoring callback
      */
     void safetyMonitorCallback();
     
     /**
      * @brief Log safety events with timestamps
      * @param event Description of the safety event
      */
     void logSafetyEvent(const std::string& event);
     
     /**
      * @brief Check for DMS timeout (space bar released too long)
      */
     void checkDMSTimeout();
     
     /**
      * @brief Handle emergency stop activation
      */
     void handleEmergencyStop();
 };
 
 #endif // SAFETY_MANAGER_H