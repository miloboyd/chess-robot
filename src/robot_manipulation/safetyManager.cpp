/**
 * @file safetyManager.cpp
 * @brief Implementation of safety management system matching GUI interface
 */

 #include "safetyManager.h"

 SafetyManager::SafetyManager(rclcpp::Node::SharedPtr node) : node_(node) {
     
     // Subscribe to the EXACT topics your GUI publishes to
     dms_subscriber_ = node_->create_subscription<std_msgs::msg::Bool>(
         "ur3/dms", 10,  // Dead Man's Switch (space bar)
         std::bind(&SafetyManager::dmsCallback, this, std::placeholders::_1)
     );
     
     estop_subscriber_ = node_->create_subscription<std_msgs::msg::Bool>(
         "ur3/estop", 10,  // Emergency Stop button
         std::bind(&SafetyManager::estopCallback, this, std::placeholders::_1)
     );
     
     turn_subscriber_ = node_->create_subscription<std_msgs::msg::Bool>(
         "/move_complete", 10,  // Turn management
         std::bind(&SafetyManager::turnCallback, this, std::placeholders::_1)
     );
     
     // Create a timer to monitor safety conditions periodically
     safety_monitor_timer_ = node_->create_wall_timer(
         SAFETY_CHECK_INTERVAL,
         std::bind(&SafetyManager::safetyMonitorCallback, this)
     );
     
     // Initialize timestamps
     last_dms_update_ = std::chrono::steady_clock::now();
     last_safety_check_ = std::chrono::steady_clock::now();
     
     RCLCPP_INFO(node_->get_logger(), 
                 "SafetyManager initialized - subscribing to ur3/dms, ur3/estop, ur3/turn");
     RCLCPP_INFO(node_->get_logger(), 
                 "Safety monitoring interval: %d ms", 
                 static_cast<int>(SAFETY_CHECK_INTERVAL.count()));
 }
 
 // Callback for Dead Man's Switch (space bar from GUI)
 void SafetyManager::dmsCallback(const std_msgs::msg::Bool::SharedPtr msg) {
     bool previous_state = dms_active_.exchange(msg->data);
     last_dms_update_ = std::chrono::steady_clock::now();
     
     // Log state changes (matching GUI behavior)
     if (previous_state != msg->data) {
         if (msg->data) {
             logSafetyEvent("Dead Man's Switch ACTIVATED (space bar pressed)");
             RCLCPP_INFO(node_->get_logger(), "DMS: Robot movement enabled");
         } else {
             logSafetyEvent("Dead Man's Switch DEACTIVATED (space bar released)");
             RCLCPP_WARN(node_->get_logger(), "DMS: Robot movement disabled - hold space bar");
         }
     }
 }
 
 // Callback for Emergency Stop (button from GUI)
 void SafetyManager::estopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
     bool previous_state = estop_triggered_.exchange(msg->data);
     last_safety_check_ = std::chrono::steady_clock::now();
     
     // Emergency stop state change
     if (previous_state != msg->data) {
         if (msg->data) {
             logSafetyEvent("EMERGENCY STOP ACTIVATED");
             RCLCPP_ERROR(node_->get_logger(), "EMERGENCY STOP TRIGGERED - ALL MOVEMENT HALTED");
             handleEmergencyStop();
         } else {
             logSafetyEvent("Emergency stop RESET");
             RCLCPP_WARN(node_->get_logger(), "Emergency stop has been reset - system ready");
         }
     }
 }
 
 // Callback for Turn management (from GUI)
 void SafetyManager::turnCallback(const std_msgs::msg::Bool::SharedPtr msg) {
     bool previous_turn = is_robot_turn_.exchange(!msg->data); // GUI sends false for robot turn
     
     // Log turn changes
     if (previous_turn != (!msg->data)) {
         if (msg->data) {  // GUI sends true when it's robot's turn
             logSafetyEvent("Turn: ROBOT TURN - robot can move");
             RCLCPP_INFO(node_->get_logger(), "Turn: Robot's turn to move");
         } else {
             logSafetyEvent("Turn: HUMAN TURN - robot must wait");
             RCLCPP_INFO(node_->get_logger(), "Turn: Human's turn - robot waiting");
         }
     }
 }
 
 void SafetyManager::handleEmergencyStop() {
     // Your GUI expects the program to handle e-stop gracefully
     // Based on your GUI code, e-stop should:
     // 1. Stop all robot movement immediately
     // 2. Reset turn to human if it was robot's turn
     // 3. Disable robot movement until e-stop is reset
     
     RCLCPP_FATAL(node_->get_logger(), "Emergency stop procedures initiated");
     
     // If you want to exit like your original design:
     // rclcpp::shutdown();
     // std::exit(1);
     
     // Or handle gracefully (recommended):
     // Just keep estop_triggered_ = true and let other systems check isSafeToOperate()
 }
 
 std::string SafetyManager::getSafetyStatus() const {
     std::string status;
     
     if (isEstopTriggered()) {
         status = "EMERGENCY STOP ACTIVE";
     } else if (!isDMSActive() && isRobotTurn()) {
         status = "DMS REQUIRED - Hold space bar to enable movement";
     } else if (!isRobotTurn()) {
         status = "HUMAN TURN - Robot waiting";
     } else if (canRobotMove()) {
         status = "SAFE TO MOVE - DMS active, robot's turn";
     } else {
         status = "WAITING - Check DMS and turn status";
     }
     
     return status;
 }
 
 std::string SafetyManager::getDetailedStatus() const {
     return "Safety: DMS=" + std::string(dms_active_ ? "ON" : "OFF") + 
            ", E-Stop=" + std::string(estop_triggered_ ? "TRIGGERED" : "CLEAR") +
            ", Turn=" + std::string(is_robot_turn_ ? "ROBOT" : "HUMAN") +
            ", CanMove=" + std::string(canRobotMove() ? "YES" : "NO");
 }
 
 void SafetyManager::safetyMonitorCallback() {
     // Check for DMS timeout (space bar held too long without update)
     checkDMSTimeout();
     
     // Update last check time
     last_safety_check_ = std::chrono::steady_clock::now();
     
     // Periodic safety status logging (every 5 seconds when not safe)
     static int counter = 0;
     if (++counter >= 100) { // 100 * 50ms = 5 seconds
         counter = 0;
         if (!isSafeToOperate() || !isRobotTurn()) {
             RCLCPP_DEBUG(node_->get_logger(), "%s", getDetailedStatus().c_str());
         }
     }
     
     // Alert if safety conditions are not met and robot is trying to move
     if (isRobotTurn() && !isSafeToOperate()) {
         static bool warning_logged = false;
         if (!warning_logged) {
             RCLCPP_WARN(node_->get_logger(), 
                        "SAFETY ALERT: Robot turn but unsafe conditions - %s", 
                        getSafetyStatus().c_str());
             warning_logged = true;
         }
     } else {
         static bool warning_logged = false;
         warning_logged = false; // Reset warning flag when safe
     }
 }
 
 void SafetyManager::checkDMSTimeout() {
     auto now = std::chrono::steady_clock::now();
     auto time_since_last_update = std::chrono::duration_cast<std::chrono::milliseconds>(
         now - last_dms_update_);
     
     // If we haven't received a DMS update recently and it was active, assume it's inactive
     // This handles the case where GUI crashes or connection is lost
     if (time_since_last_update > DMS_TIMEOUT && dms_active_) {
         RCLCPP_WARN(node_->get_logger(), 
                    "DMS timeout detected (%ld ms since last update) - assuming DMS inactive", 
                    time_since_last_update.count());
         
         dms_active_ = false;
         logSafetyEvent("DMS timeout - automatically deactivated for safety");
     }
 }
 
 void SafetyManager::logSafetyEvent(const std::string& event) {
     auto now = std::chrono::system_clock::now();
     auto time_t = std::chrono::system_clock::to_time_t(now);
     auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
         now.time_since_epoch()) % 1000;
     
     // Create timestamp string
     std::tm* tm_ptr = std::localtime(&time_t);
     char timestamp[100];
     std::strftime(timestamp, sizeof(timestamp), "%H:%M:%S", tm_ptr);
     
     std::string log_message = "[SAFETY " + std::string(timestamp) + "." + 
                              std::to_string(ms.count()) + "] " + event;
     
     // Log at appropriate level based on event type
     if (event.find("EMERGENCY") != std::string::npos) {
         RCLCPP_FATAL(node_->get_logger(), "%s", log_message.c_str());
     } else if (event.find("timeout") != std::string::npos || 
                event.find("DEACTIVATED") != std::string::npos) {
         RCLCPP_WARN(node_->get_logger(), "%s", log_message.c_str());
     } else {
         RCLCPP_INFO(node_->get_logger(), "%s", log_message.c_str());
     }
 }