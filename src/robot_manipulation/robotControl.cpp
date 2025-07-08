#include "robotControl.h"

RobotControl::RobotControl(rclcpp::Node::SharedPtr node, SafetyManager* safety_manager) : node_(node), safety_manager_(safety_manager) {

  //initialise connection to gripper. Set global parameters to use 
  grip_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/finger_width_controller/commands", 10
  );

  open_.data = {0.025};
  close_.data = {0.004};

  RCLCPP_INFO(node_->get_logger(), "RobotControl created");
  //setUpPlanningScene();
}

bool RobotControl::initialise() {
  try {
      RCLCPP_INFO(node_->get_logger(), "Creating MoveGroupInterface...");
      
      // This addresses your initialization timing issue
      move_group_ptr_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
          node_, "ur_onrobot_manipulator");
      
      setupMoveItParameters();

      RCLCPP_INFO(node_->get_logger(), "RobotControl initialized successfully");
      return true;
      
      
  } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to initialize MoveGroupInterface: %s", e.what());
      return false;
  }
}

void RobotControl::setupMoveItParameters() {
  // Set default parameters (same as your original)
  move_group_ptr_->setMaxVelocityScalingFactor(0.1);
  move_group_ptr_->setMaxAccelerationScalingFactor(0.1);
  move_group_ptr_->setGoalPositionTolerance(0.005);
  move_group_ptr_->setGoalOrientationTolerance(0.1);

}

bool RobotControl::moveLinear(double target_x, double target_y, double target_z) {

  //set velocity parameters (cartesian movement is a little faster than normal move)
  move_group_ptr_->setMaxVelocityScalingFactor(0.02);
  move_group_ptr_->setMaxAccelerationScalingFactor(0.02);

  //get current position
  geometry_msgs::msg::Pose current_pose = move_group_ptr_->getCurrentPose().pose;
  geometry_msgs::msg::Quaternion locked_orientation = current_pose.orientation; // Define locked_orientation

  // RCLCPP_INFO(node_->get_logger(), "Current position: (%.3f, %.3f, %.3f)", 
  //             current_pose.position.x, current_pose.position.y, current_pose.position.z);


  //std::vector<double> joint_values = move_group_ptr_->getCurrentJointValues();
  //std::vector<std::string> joint_names = move_group_ptr_->getJointNames();

  // RCLCPP_INFO(node_->get_logger(), "Current joint positions:");
  // for (size_t i = 0; i < joint_names.size(); ++i) {

  //     RCLCPP_INFO(node_->get_logger(), "  %s: %.3f rad (%.1f deg)", 
  //                 joint_names[i].c_str(), joint_values[i], joint_values[i] * 180.0 / M_PI);
  // }

  //calculate movement distance
  double dx = target_x - current_pose.position.x;
  double dy = target_y - current_pose.position.y;
  double dz = target_z - current_pose.position.z;
  double total_distance = sqrt(dx*dx + dy*dy + dz*dz);
  //RCLCPP_INFO(node_->get_logger(), "Chess move: %.1fcm distance", total_distance * 100);

  std::vector<geometry_msgs::msg::Pose> waypoints;

  if (total_distance > 0.05) {
    
    const double step_size = 0.03;
    int num_steps = ceil(total_distance / step_size);
    //RCLCPP_INFO(node_->get_logger(), "Large movement: creating %d waypoints", num_steps);

    // Add intermediate waypoints
    for (int i = 1; i <= num_steps; i++) {
      double progress = (double)i / num_steps;
      
      geometry_msgs::msg::Pose waypoint;
      waypoint.position.x = current_pose.position.x + dx * progress;
      waypoint.position.y = current_pose.position.y + dy * progress;
      waypoint.position.z = current_pose.position.z;
      waypoint.orientation = locked_orientation;  // Same orientation for all waypoints
      
      waypoints.push_back(waypoint);
    }
  }
  else {

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = target_x;
    target_pose.position.y = target_y;
    target_pose.position.z = target_z;
    target_pose.orientation = locked_orientation; //preserve orientation;  // Same orientation for all waypoints

    waypoints.push_back(target_pose);
    //RCLCPP_INFO(node_->get_logger(), "Small movement: single waypoint");
  }

  bool success = executeCartesianPath(waypoints);

  if (!success) {
    geometry_msgs::msg::Pose final_target;
    final_target.position.x = target_x;
    final_target.position.y = target_y;
    final_target.position.z = target_z;
    final_target.orientation = locked_orientation;
    success = executeJointSpacePlan(final_target);
  } 

  std::vector<double> joint_values = move_group_ptr_->getCurrentJointValues();
  std::cout << "(" << joint_values[0] << "," << joint_values[1] << "," << joint_values[2] << "," << joint_values[3] << "," << joint_values[4] << "," << joint_values[5] << std::endl;

  return success;
}

bool RobotControl::executeJointSpacePlan(const geometry_msgs::msg::Pose& target) {
  
  move_group_ptr_->setPoseTarget(target);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  
  bool success = (move_group_ptr_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (success) {
    auto result = move_group_ptr_->execute(plan);
    return (result == moveit::core::MoveItErrorCode::SUCCESS);
  }
  return false;
}

bool RobotControl::executeCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints) {

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.005;

  double fraction = move_group_ptr_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  //RCLCPP_INFO(node_->get_logger(), "Cartesian Path Planning (%.2f%% achieved)", fraction * 100.0); 

  if (fraction >= 0.95) {  // Require near-perfect planning
    
    // SET LOOSER TOLERANCES BEFORE EXECUTION
    move_group_ptr_->setGoalPositionTolerance(0.005);    // 5mm position tolerance
    move_group_ptr_->setGoalOrientationTolerance(0.1);   // ~6 degree orientation tolerance
    

    //if (!safety_manager_->isEstopTriggered()) {
      auto result = move_group_ptr_->execute(trajectory);
      //RCLCPP_INFO(node_->get_logger(), "Executing cartesian path with relaxed tolerances");
    //} else {

    //}
    
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      //RCLCPP_INFO(node_->get_logger(), "Cartesian execution successful!");

      std::this_thread::sleep_for(std::chrono::seconds(2));
      
      geometry_msgs::msg::Pose new_pose = move_group_ptr_->getCurrentPose().pose;
      //RCLCPP_INFO(node_->get_logger(), "Position after translation: (%.3f, %.3f, %.3f)",
      //            new_pose.position.x, new_pose.position.y, new_pose.position.z);
      return true;

    } else {
      //RCLCPP_WARN(node_->get_logger(), "Cartesian execution failed, trying joint space");
      // Fall through to joint space planning
    }
    
  }
  return false;
}

bool RobotControl::pickUpPiece() {

  geometry_msgs::msg::Pose current_pose = move_group_ptr_->getCurrentPose().pose;
  geometry_msgs::msg::Quaternion locked_orientation = current_pose.orientation; // Define locked_orientation
  const double safe_height = current_pose.position.z; //remember safe height

  RCLCPP_INFO(node_->get_logger(), "Current position: (%.3f, %.3f, %.3f)", 
              current_pose.position.x, current_pose.position.y, current_pose.position.z);

  // STEP 1: move down to piece level
  geometry_msgs::msg::Pose down_pose;
  down_pose.position.x = current_pose.position.x;
  down_pose.position.y = current_pose.position.y;
  down_pose.position.z = 0.047; // piece level
  down_pose.orientation = locked_orientation;

  std::vector<geometry_msgs::msg::Pose> down_waypoints; // New vector for down movement
  down_waypoints.push_back(down_pose);

  if (!executeCartesianPath(down_waypoints)) {
    return false;
  }

  //STEP 2: Close Gripper
  RCLCPP_INFO(node_->get_logger(), "Closing gripper to grab piece"); 
  grip_pub_->publish(close_);
  std::this_thread::sleep_for(std::chrono::seconds(2)); // Delay movement to allow gripper time to grab

  //STEP 3: Move UP to safe height
  geometry_msgs::msg::Pose up_pose;
  up_pose.position.x = current_pose.position.x;
  up_pose.position.y = current_pose.position.y;
  up_pose.position.z = safe_height; // piece level
  up_pose.orientation = locked_orientation;

  std::vector<geometry_msgs::msg::Pose> up_waypoints; // New vector for down movement
  up_waypoints.push_back(up_pose);

  bool success = executeCartesianPath(up_waypoints);
  return success;
}

bool RobotControl::placePiece() {
  
  geometry_msgs::msg::Pose current_pose = move_group_ptr_->getCurrentPose().pose;
  geometry_msgs::msg::Quaternion locked_orientation = current_pose.orientation; // Define locked_orientation
  const double safe_height = current_pose.position.z; //remember safe height

  RCLCPP_INFO(node_->get_logger(), "Current position: (%.3f, %.3f, %.3f)", 
              current_pose.position.x, current_pose.position.y, current_pose.position.z);

  // STEP 1: move down to piece level
  geometry_msgs::msg::Pose down_pose;
  down_pose.position.x = current_pose.position.x;
  down_pose.position.y = current_pose.position.y;
  down_pose.position.z = 0.047; // piece level
  down_pose.orientation = locked_orientation;

  std::vector<geometry_msgs::msg::Pose> down_waypoints; // New vector for down movement
  down_waypoints.push_back(down_pose);

  if (!executeCartesianPath(down_waypoints)) {
    return false;
  }

  //STEP 2: Open Gripper
  RCLCPP_INFO(node_->get_logger(), "Closing gripper to grab piece"); 
  grip_pub_->publish(open_);
  std::this_thread::sleep_for(std::chrono::seconds(2)); // Delay movement to allow gripper time to grab

  //STEP 3: Move UP to safe height
  geometry_msgs::msg::Pose up_pose;
  up_pose.position.x = current_pose.position.x;
  up_pose.position.y = current_pose.position.y;
  up_pose.position.z = safe_height; // piece level
  up_pose.orientation = locked_orientation;

  std::vector<geometry_msgs::msg::Pose> up_waypoints; // New vector for down movement
  up_waypoints.push_back(up_pose);

  bool success = executeCartesianPath(up_waypoints);
  return success;
}

bool RobotControl::moveBoard() {

  //set velocity parameters for safe robot trajectory
  move_group_ptr_->setMaxVelocityScalingFactor(0.1);
  move_group_ptr_->setMaxAccelerationScalingFactor(0.1);

  //move to position to execute chess piece movement 
  std::vector<double> set_position = {83.6*M_PI/180, -85.3*M_PI/180, 88.2*M_PI/180, -93.0*M_PI/180, -89.9*M_PI/180, -5.8*M_PI/180};

  //open gripper whilst moving to position
  grip_pub_->publish(open_);

  //move arm to position
  move_group_ptr_->setJointValueTarget(set_position);
  move_group_ptr_->move();
  RCLCPP_INFO(node_->get_logger(), "Moved to setup position!");

  return true;
}

bool RobotControl::moveHome() {

  //set velocity parameters for safe robot trajectory
  move_group_ptr_->setMaxVelocityScalingFactor(0.1);
  move_group_ptr_->setMaxAccelerationScalingFactor(0.1);

  std::vector<double> home_position = {0.0, -1.57, 0.0, -1.57, 0.0, -3.14};
  move_group_ptr_->setJointValueTarget(home_position);
  move_group_ptr_->move();

  RCLCPP_INFO(node_->get_logger(), "Finishing robot turn!");
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
  floor_primitive.dimensions[floor_primitive.BOX_Z] = 0.01; // 2cm thick
  
  geometry_msgs::msg::Pose floor_pose;
  floor_pose.orientation.w = 1.0;
  floor_pose.position.x = 0.0;
  floor_pose.position.y = 0.0;
  floor_pose.position.z = 0.045; // Positioned at lowest possible height for robot arm to descend to 
  
  floor_object.primitives.push_back(floor_primitive);
  floor_object.primitive_poses.push_back(floor_pose);
  floor_object.operation = floor_object.ADD;
  collision_objects.push_back(floor_object);
  
  
  // Add all objects to the planning scene
  RCLCPP_INFO(node_->get_logger(), "Adding collision objects to the scene");
  planning_scene_interface.addCollisionObjects(collision_objects);

}

void RobotControl::setConstraints() {
  
}

