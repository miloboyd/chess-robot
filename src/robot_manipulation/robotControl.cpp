#include "robotControl.h"

RobotControl::RobotControl() : Node("ur3e_control_node") {

  //initialise UR3e control node
  RCLCPP_INFO(this->get_logger(), "Initializing UR3e Control Node");
  std::this_thread::sleep_for(std::chrono::seconds(2));

  //initialise moveIt
  RCLCPP_INFO(this->get_logger(), "Creating MoveGroupInterface for ur_onrobot_manipulator");
  moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), "ur_onrobot_manipulator");

  move_group.setMaxVelocityScalingFactor(0.01);
  move_group.setMaxAccelerationScalingFactor(0.01);

  //initialise connection to gripper. Set global parameters to use 
  auto grip_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/finger_width_controller/commands", 10
  );

  auto open =  std_msgs::msg::Float64MultiArray();
  auto close =  std_msgs::msg::Float64MultiArray();
  open.data = {0.05};
  close.data = {0.025};

  setConstraints();

}

bool RobotControl::moveRobot(double x_coordinate, double y_coordinate, double z_coordinate) {
  
  RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());
  
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

  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(this->get_logger(), "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  move_group.move();

}

void RobotControl::setConstraints() {
  
}


bool RobotControl::pickUpPiece(double x_coordinate, double y_coordinate, double z_coordinate) {

  //cartesian path down to level to grasp piece
  geometry_msgs::msg::Pose Pose;
  Pose.position.x = x_coordinate;
  Pose.position.y = y_coordinate;
  Pose.position.z = z_coordinate;
  //define waypoint
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(Pose);

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  RCLCPP_INFO(this->get_logger(), "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0); 
   
  //execute motion
  move_group.execute(trajectory);

  //pick piece (grab)
  grip_pub->publish(close);

  //cartesian path up to level to manouver to original level 
  Pose.position.z += 0.2;
  waypoints.push_back(Pose);
  fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  RCLCPP_INFO(this->get_logger(), "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0); 

  return true;
}

bool RobotControl::placePiece(double x_coordinate, double y_coordinate, double z_coordinate) {
    //cartesian path down to level to grasp piece
    geometry_msgs::msg::Pose Pose;
    Pose.position.x = x_coordinate;
    Pose.position.y = y_coordinate;
    Pose.position.z = z_coordinate;
    //define waypoint
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(Pose);
  
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    RCLCPP_INFO(this->get_logger(), "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0); 
     
    //execute motion
    move_group.execute(trajectory);
  
    //pick piece (grab)
    grip_pub->publish(open);
  
    //cartesian path up to level to manouver to original level 
    Pose.position.z += 0.2;
    waypoints.push_back(Pose);
    fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    RCLCPP_INFO(this->get_logger(), "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0); 
  
    return true;
}

void RobotControl::setUpPlanningScene() {
  
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


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


}

