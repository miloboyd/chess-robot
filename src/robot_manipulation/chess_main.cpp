/**
 * @file chess_main.cpp
 * @brief Simple entry point for the simplified chess robot system (no safety)
 */

 #include "main_tester.h"
 #include "main.h"

 int main(int argc, char** argv) {
     rclcpp::init(argc, argv);
     QApplication app(argc, argv);

     
     try {
         auto main_node = std::make_shared<Main>(); //Main for normal operations
         main_node->run();
     } catch (const std::exception& e) {
         RCLCPP_ERROR(rclcpp::get_logger("chess_main"), "Fatal error: %s", e.what());
         QApplication::quit();
         rclcpp::shutdown();
         return 1;
     }

     return 0;
 }