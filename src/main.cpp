//#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <string>
#include <thread>
#include <QApplication>
#include <iostream>

#include "GUI/GUI.h"
#include "GUI/RobotNode.h"

int main(int argc, char **argv)
{
    //setvbuf(stdout, NULL, _IONBF, 0);  // Optional, for clean logs
    rclcpp::init(argc, argv);
    std::cout << "Checkpoint 0" << std::endl;
    // Initialize Qt
    QApplication app(argc, argv);
    std::cout << "Checkpoint 1" << std::endl;
    //auto gui_node = std::make_shared<GUI>();  old method
    auto robot_node = std::make_shared<RobotNode>();
    std::cout << "Checkpoint 2" << std::endl;
    GUI *gui = new GUI(robot_node);

    std::cout << "Checkpoint 3" << std::endl;
    // Create and show the GUI
   
    gui->show();
    std::cout << "Checkpoint 4" << std::endl;
    // Create an executor for ROS2
    rclcpp::executors::MultiThreadedExecutor executor;
    //executor.add_node(gui_node); /old method
    executor.add_node((robot_node));
    
    // Spin in a separate thread
    std::thread robotThread([&executor]() {
        executor.spin();
    });
    std::cout << "Checkpoint 5" << std::endl;
    // Run the Qt event loop
    int result = app.exec();
    std::cout << "Checkpoint 6" << std::endl;
    // Clean up ROS2
    rclcpp::shutdown();
    // Wait for the spin thread to finish
    robotThread.join();
    std::cout << "Checkpoint 7" << std::endl;
    return result;
}