#include "boardPos.h"
#include "robotControl.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotControl>();
  // Run the demo
  node->run_demo();
  
  rclcpp::shutdown();
  return 0;
}