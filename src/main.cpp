//#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <string>
#include <thread>
#include <QApplication>
#include <iostream>

#include "GUI/GUI.h"
#include "GUI/RobotNode.h"
#include "robot_manipulation/safetyManager.h"

int main(int argc, char **argv)
{
    //setvbuf(stdout, NULL, _IONBF, 0);  // Optional, for clean logs
    rclcpp::init(argc, argv);
    // Initialize Qt
    QApplication app(argc, argv);
    
    auto robot_node = std::make_shared<RobotNode>();
    GUI *gui = new GUI(robot_node);
    std::unique_ptr<SafetyManager> safety_manager_ = std::make_unique<SafetyManager>(robot_node);
    // Create and show the GUI
   
    gui->show();
    // Create an executor for ROS2
    rclcpp::executors::MultiThreadedExecutor executor;
    //executor.add_node(gui_node); /old method
    executor.add_node((robot_node));
    
    // Spin in a separate thread
    std::thread robotThread([&executor]() {
        executor.spin();
    });
    // Run the Qt event loop
    int result = app.exec();
    // Clean up ROS2
    rclcpp::shutdown();
    // Wait for the spin thread to finish
    robotThread.join();
    return result;
}