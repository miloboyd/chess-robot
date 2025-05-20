#include "boardPos.h"
#include "robotControl.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotControl>();
  // Run the demo
  
  //subscribe to a pose 
  geometry_msgs::msg::Pose input_pose;
  input_pose.position.x = 0;
  input_pose.position.y = 0.2;
  input_pose.position.z = 0.1;


  node->moveRobot(input_pose.position.x, input_pose.position.y, input_pose.position.z);


  
  rclcpp::shutdown();
  return 0;
}