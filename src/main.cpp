#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <string>
#include <thread>
#include <QApplication>
#include "GUI/GUI.h"  // Make sure this path is correct for your project structure

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto gui_node = std::make_shared<rclcpp::Node>("GUI");
    
    // Initialize Qt
    QApplication app(argc, argv);
    
    // Create and show the GUI
    GUI gui(gui_node);
    gui.show();
    
    // Create an executor for ROS2
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(gui_node);
    
    // Spin in a separate thread
    std::thread rosSpinThread([&executor]() {
        executor.spin();
    });
    
    // Run the Qt event loop
    int result = app.exec();
    
    // Clean up ROS2
    rclcpp::shutdown();
    
    // Wait for the spin thread to finish
    if (rosSpinThread.joinable()) {
        rosSpinThread.join();
    }
    
    return result;
}