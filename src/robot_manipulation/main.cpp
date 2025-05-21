#include "boardPos.h"
#include "robotControl.h"


//take in paramaters of positiosn and a boolean to represent taken status 
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // auto const moveit = std::make_shared<rclcpp::Node>(
  //   "moveit_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)  //alternative
  // );
  
  //auto node = std::make_shared<RobotControl>();
  // Run the demo
  auto node = RobotControl::create();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  // executor.add_node(moveit); //alternative
  auto spinner = std::thread([&executor]() {executor.spin();});

  // using moveit::planning_interface::MoveGroupInterface;
  
  //subscribe to a pose 
  geometry_msgs::msg::Pose input_pose;
  input_pose.position.x = 0.1;
  input_pose.position.y = 0.3;  
  input_pose.position.z = 0.3;


  node->moveRobot(input_pose.position.x, input_pose.position.y, input_pose.position.z);


  
  rclcpp::shutdown();
  spinner.join();
  return 0;
}