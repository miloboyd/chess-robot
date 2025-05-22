#include "boardPos.h"
#include "robotControl.h"


//take in paramaters of positiosn and a boolean to represent taken status 
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  /**
  auto const moveit = std::make_shared<rclcpp::Node>(
    "moveit_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)  //alternative
  );
  auto node = std::make_shared<RobotControl>();
  using moveit::planning_interface::MoveGroupInterface;
  executor.add_node(moveit); //alternative
  */

  //create 
  auto node = RobotControl::create();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() {executor.spin();});

  
  node->moveBoard();

  node->moveHome();


  
  rclcpp::shutdown();
  spinner.join();
  return 0;
}