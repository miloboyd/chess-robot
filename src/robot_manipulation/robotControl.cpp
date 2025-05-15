#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_msgs/msg/float64_multi_array.hpp>
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
    RCLCPP_INFO(this->get_logger(), "Creating MoveGroupInterface for ur_onrobot_manipulator");
    moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), "ur_onrobot_manipulator");

    //moveit::planning_interface::MoveGroupInterface gripper_group(shared_from_this(), "ur_onrobot_gripper");

    auto grip_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/finger_width_controller/commands", 10
    );

    auto open =  std_msgs::msg::Float64MultiArray();
    auto close =  std_msgs::msg::Float64MultiArray();
    open.data = {0.05};
    close.data = {0.025};

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // Set lower velocity and acceleration limits for safety
    /*
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setMaxVelocityScalingFactor(0.10);
    move_group.setMaxAccelerationScalingFactor(0.10);
    move_group.setPlanningTime(10.0);
    move_group.setNumPlanningAttempts(50);
    */
    /*
    const moveit::core::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup("ur_onrobot_manipulator");
    */

    RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());
    
    // Move to home position
    //RCLCPP_INFO(this->get_logger(), "Moving to home position");
    //move_group.setNamedTarget("home");

    double pi = 3.141592654;

    geometry_msgs::msg::Pose target_pose1;
    // Gripper pointing right (90 degrees around Z)
    // Gripper pointing backward (180 degrees around Z)
    target_pose1.orientation.x = 1.0;
    target_pose1.orientation.y = 0.0;
    target_pose1.orientation.z = 0.0;
    target_pose1.orientation.w = 0.0;

    target_pose1.position.x = 0;
    target_pose1.position.y = 0.2;
    target_pose1.position.z = 0.1;
    move_group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;



    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    // 1. Create floor
    moveit_msgs::msg::CollisionObject floor_object;
    floor_object.header.frame_id = "base_link";
    floor_object.id = "floor";
    
    shape_msgs::msg::SolidPrimitive floor_primitive;
    floor_primitive.type = floor_primitive.BOX;
    floor_primitive.dimensions.resize(3);
    floor_primitive.dimensions[floor_primitive.BOX_X] = 1.0;  // 1m wide in X
    floor_primitive.dimensions[floor_primitive.BOX_Y] = 1.0;  // 1m wide in Y
    floor_primitive.dimensions[floor_primitive.BOX_Z] = 0.02; // 2cm thick
    
    geometry_msgs::msg::Pose floor_pose;
    floor_pose.orientation.w = 1.0;
    floor_pose.position.x = 0.0;
    floor_pose.position.y = 0.0;
    floor_pose.position.z = -0.02; // Positioned at base level
    
    floor_object.primitives.push_back(floor_primitive);
    floor_object.primitive_poses.push_back(floor_pose);
    floor_object.operation = floor_object.ADD;
    collision_objects.push_back(floor_object);
    
    // 2. Create wall behind the robot
    moveit_msgs::msg::CollisionObject back_wall;
    back_wall.header.frame_id = "base_link";
    back_wall.id = "back_wall";
    
    shape_msgs::msg::SolidPrimitive back_wall_primitive;
    back_wall_primitive.type = back_wall_primitive.BOX;
    back_wall_primitive.dimensions.resize(3);
    back_wall_primitive.dimensions[back_wall_primitive.BOX_X] = 0.01; // 1cm thick
    back_wall_primitive.dimensions[back_wall_primitive.BOX_Y] = 1.0;  // 1m wide
    back_wall_primitive.dimensions[back_wall_primitive.BOX_Z] = 0.5;  // 50cm tall
    
    geometry_msgs::msg::Pose back_wall_pose;
    back_wall_pose.orientation.z = 1.0;
    back_wall_pose.position.x = 0;   // 20cm behind robot base
    back_wall_pose.position.y = -0.3;    // Centered
    back_wall_pose.position.z = 0.25;   // Midpoint of wall height
    
    back_wall.primitives.push_back(back_wall_primitive);
    back_wall.primitive_poses.push_back(back_wall_pose);
    back_wall.operation = back_wall.ADD;
    collision_objects.push_back(back_wall);
    
    // 3. Create ceiling
    moveit_msgs::msg::CollisionObject ceiling;
    ceiling.header.frame_id = "base_link";
    ceiling.id = "ceiling";
    
    shape_msgs::msg::SolidPrimitive ceiling_primitive;
    ceiling_primitive.type = ceiling_primitive.BOX;
    ceiling_primitive.dimensions.resize(3);
    ceiling_primitive.dimensions[ceiling_primitive.BOX_X] = 1.0;  // 1m in X
    ceiling_primitive.dimensions[ceiling_primitive.BOX_Y] = 1.0;  // 1m in Y
    ceiling_primitive.dimensions[ceiling_primitive.BOX_Z] = 0.01; // 1cm thick
    
    geometry_msgs::msg::Pose ceiling_pose;
    ceiling_pose.orientation.w = 1.0;
    ceiling_pose.position.x = 0.0;
    ceiling_pose.position.y = 0.0;
    ceiling_pose.position.z = 0.6;  // 60cm above base
    
    ceiling.primitives.push_back(ceiling_primitive);
    ceiling.primitive_poses.push_back(ceiling_pose);
    ceiling.operation = ceiling.ADD;
    collision_objects.push_back(ceiling);
    
    // Add all objects to the planning scene
    RCLCPP_INFO(this->get_logger(), "Adding collision objects to the scene");
    planning_scene_interface.addCollisionObjects(collision_objects);


    grip_pub->publish(open);

    std::this_thread::sleep_for(std::chrono::seconds(3));


    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(this->get_logger(), "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group.move();

    grip_pub->publish(close);


    
    //message.data = 

    

    
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = "gripper_tcp";
    ocm.header.frame_id = "base_link";
    ocm.orientation.x = 1.0;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;

    moveit_msgs::msg::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    move_group.setPathConstraints(test_constraints);
    
    move_group.setMaxVelocityScalingFactor(0.01);
    move_group.setMaxAccelerationScalingFactor(0.01);

    
    std::vector<geometry_msgs::msg::Pose> waypoints;
    target_pose1.position.z += 0.1;
    waypoints.push_back(target_pose1);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    RCLCPP_INFO(this->get_logger(), "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

    move_group.execute(trajectory);

    grip_pub->publish(open);





    //move_group.move();
  
    // Return to home position
    /*
    RCLCPP_INFO(this->get_logger(), "Returning to home position");
    std::vector<double> home_position = {0.0, -1.57, 0.0, -1.57, 0.0, -3.14};
    move_group.setJointValueTarget(home_position);
    move_group.move();
    */



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