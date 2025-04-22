#include "ur3e_mtc.h"
#include <iostream>
#include <sstream>
#include <vector>

// Create a pose from individual coordinates
geometry_msgs::msg::Pose createPose(double x, double y, double z) {
  geometry_msgs::msg::Pose pose;
  
  // Set position
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  
  // Default orientation (pointing downward)
  pose.orientation.x = 0.0;
  pose.orientation.y = 1.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 0.0;
  
  return pose;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_node = std::make_shared<UR3eMTC>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_node]() {
    executor.add_node(mtc_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_node->getNodeBaseInterface());
  });

  // Set up the planning scene
  mtc_node->setupPlanningScene(false);  // Don't add cylinder
  
  // Define Cartesian coordinates directly in code
  geometry_msgs::msg::Pose start_pose = createPose(0.4, 0.2, 0.2);
  geometry_msgs::msg::Pose end_pose = createPose(0.4, 0.3, 0.2);
  
  RCLCPP_INFO(rclcpp::get_logger("ur3e_mtc_main"), 
              "Moving from coordinates [%.3f, %.3f, %.3f] to [%.3f, %.3f, %.3f]", 
              start_pose.position.x, start_pose.position.y, start_pose.position.z,
              end_pose.position.x, end_pose.position.y, end_pose.position.z);
  
  // First move to home position for safety
  bool success = mtc_node->moveToNamedPosition("home");
  if (!success) {
    RCLCPP_ERROR(rclcpp::get_logger("ur3e_mtc_main"), 
                 "Failed to move to home position");
    spin_thread->join();
    rclcpp::shutdown();
    return 1;
  }
  
  // Move from start to end in one task
  success = mtc_node->moveToPositions(start_pose, end_pose);
  
  if (!success) {
    RCLCPP_ERROR(rclcpp::get_logger("ur3e_mtc_main"), 
                 "Failed to execute movement");
    spin_thread->join();
    rclcpp::shutdown();
    return 1;
  }
  
  // Return to home position
  success = mtc_node->moveToNamedPosition("home");
  if (!success) {
    RCLCPP_ERROR(rclcpp::get_logger("ur3e_mtc_main"), 
                 "Failed to return to home position");
  }
  
  RCLCPP_INFO(rclcpp::get_logger("ur3e_mtc_main"), "Movement sequence completed");
  
  // Allow time to view the result in RViz
  rclcpp::sleep_for(std::chrono::seconds(2));
  
  spin_thread->join();
  rclcpp::shutdown();
  return success ? 0 : 1;
}