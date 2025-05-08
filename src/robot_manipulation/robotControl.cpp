#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>
#include <chrono>

class RobotControl : public rclcpp::Node
{
public:
  explicit RobotControl() : Node("ur3e_control_node")
  {
    RCLCPP_INFO(this->get_logger(), "Initializing UR3e Control Node");
    
    // Wait for MoveIt to come up
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }

  void run_demo()
  {
    // Initialize move group
    moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), "ur_manipulator");
    
    // Set lower velocity and acceleration limits for safety
    move_group.setPlannerId("RRTConnectkConfigDefault");

    move_group.setMaxVelocityScalingFactor(0.10);
    move_group.setMaxAccelerationScalingFactor(0.10);

    move_group.setPlanningTime(10.0);
    move_group.setNumPlanningAttempts(50);
    
    RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());
    
    // Move to home position
    //RCLCPP_INFO(this->get_logger(), "Moving to home position");
    //move_group.setNamedTarget("home");

    double pi = 3.141592654;


    RCLCPP_INFO(this->get_logger(), "test link 1");
    std::vector<double> test_pose = {90*(pi/180), 304*(pi/180), 64*(pi/180), -101*(pi/180), 270*(pi/180), 0.0};
    move_group.setJointValueTarget(test_pose);
    move_group.move();


    std::this_thread::sleep_for(std::chrono::seconds(5));



    
    /**    // If home position is not defined, use joint values
    if (!move_group.move())
    {
      RCLCPP_INFO(this->get_logger(), "Home position not defined, using joint values");
      std::vector<double> home_position = {0.0, -1.57, 0.0, -1.57, 0.0, 0.0};
      move_group.setJointValueTarget(home_position);
      move_group.move();
    }
    */


    /**
    // Move to different positions
    RCLCPP_INFO(this->get_logger(), "Moving to position 1");
    std::vector<double> joint_position1 = {0.5, -1.57, 0.0, -1.57, 0.0, 0.0};
    move_group.setJointValueTarget(joint_position1);
    move_group.move();
    */
    

    // Try a Cartesian move
    RCLCPP_INFO(this->get_logger(), "Moving to Cartesian target");
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.0;
    target_pose.position.y = 0.3;
    target_pose.position.z = 0.3;
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 1.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.0;
    
    move_group.setPoseTarget(target_pose);
    move_group.move();

    std::this_thread::sleep_for(std::chrono::seconds(10));


    
  
    // Return to home position
    RCLCPP_INFO(this->get_logger(), "Returning to home position");
    std::vector<double> home_position = {0.0, -1.57, 0.0, -1.57, 0.0, -3.14};
    move_group.setJointValueTarget(home_position);
    move_group.move();



    std::this_thread::sleep_for(std::chrono::seconds(3));


    
    RCLCPP_INFO(this->get_logger(), "Demo completed!");
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotControl>();
  
  // Run the demo
  node->run_demo();
  
  rclcpp::shutdown();
  return 0;
}
