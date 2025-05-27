/**
 * @file chess_main.cpp
 * @brief Simple entry point for the simplified chess robot system (no safety)
 */

 #include "main.h"

 int main(int argc, char** argv) {
     rclcpp::init(argc, argv);
     
     try {
         auto main_node = std::make_shared<Main>();
         main_node->run();
     } catch (const std::exception& e) {
         RCLCPP_ERROR(rclcpp::get_logger("chess_main"), "Fatal error: %s", e.what());
         return 1;
     }
     
     rclcpp::shutdown();
     return 0;
 }