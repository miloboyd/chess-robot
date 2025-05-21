#include "robotControl.h"

RobotControl::RobotControl() : Node("ur3e_control_node") {

  //initialise UR3e control node
  RCLCPP_INFO(this->get_logger(), "Initializing UR3e Control Node");
  std::this_thread::sleep_for(std::chrono::seconds(2));

  /**
  //initialise moveIt
  RCLCPP_INFO(this->get_logger(), "Creating MoveGroupInterface for ur_onrobot_manipulator");
  
  move_group_ptr = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), "ur_onrobot_manipulator");
  //move_group(shared_from_this(), "ur_onrobot_manipulator") 
  */

  /**
  try {
    auto shared_ptr = shared_from_this();
    RCLCPP_INFO(this->get_logger(), "shared_from_this() succeeded");
    
    RCLCPP_INFO(this->get_logger(), "Creating MoveGroupInterface for ur_onrobot_manipulator");
    
    move_group_ptr = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        shared_ptr, "ur_onrobot_manipulator");
        
    RCLCPP_INFO(this->get_logger(), "MoveGroupInterface created successfully");
    
  } catch (const std::bad_weak_ptr& e) {
      RCLCPP_ERROR(this->get_logger(), "bad_weak_ptr error: %s", e.what());
      throw;
  } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Other error: %s", e.what());
      throw;
  }
  */
  
  //move_group_ptr->setMaxVelocityScalingFactor(0.01);
  //move_group_ptr->setMaxAccelerationScalingFactor(0.01);

  //initialise connection to gripper. Set global parameters to use 
  grip_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/finger_width_controller/commands", 10
  );

  open.data = {0.05};
  close.data = {0.025};

  //setConstraints();

}

bool RobotControl::moveRobot(double x_coordinate, double y_coordinate, double z_coordinate) {
  
  RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_ptr->getPlanningFrame().c_str());
  RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_ptr->getEndEffectorLink().c_str());
  
  /*
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
  move_group_ptr->setPoseTarget(target_pose1);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_ptr->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(this->get_logger(), "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  move_group_ptr->move();
  */
    //grip_pub->publish(close);

    move_group_ptr->setMaxVelocityScalingFactor(0.1);
    move_group_ptr->setMaxAccelerationScalingFactor(0.1);

    //cartesian path down to level to grasp piece
    geometry_msgs::msg::Pose current_pose = move_group_ptr->getCurrentPose().pose;
    RCLCPP_INFO(this->get_logger(), "Current position: (%.3f, %.3f, %.3f)", 
                current_pose.position.x, current_pose.position.y, current_pose.position.z);

  
    std::vector<double> joint_group_positions = {83.5*M_PI/180, -74.5*M_PI/180, 38.5*M_PI/180, -54.1*M_PI/180, -89.7*M_PI/180, -725.9*M_PI/180};


    std::vector<double> joint_values = move_group_ptr->getCurrentJointValues();
    std::vector<std::string> joint_names = move_group_ptr->getJointNames();

    RCLCPP_INFO(this->get_logger(), "Current joint positions:");
    for (size_t i = 0; i < joint_names.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "  %s: %.3f rad (%.1f deg)", 
                    joint_names[i].c_str(), joint_values[i], joint_values[i] * 180.0 / M_PI);
    }

    RCLCPP_INFO(this->get_logger(), "Current joint positions:");
    for (size_t i = 0; i < joint_values.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "  Joint %zu: %.3f radians (%.1f degrees)", 
                i, joint_values[i], joint_values[i] * 180.0 / M_PI);
    }


    //move_group_ptr->setJointValueTarget(joint_group_positions);
    //move_group_ptr->move();

    //moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    //bool success = (move_group_ptr->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    //move_group_ptr->execute(my_plan);


    

    geometry_msgs::msg::Pose Pose;
    Pose.position.x = x_coordinate;
    Pose.position.y = y_coordinate;
    Pose.position.z = z_coordinate;
    Pose.orientation.x = 1.0;  // Default
    Pose.orientation.y = 0.0;  // Default
    Pose.orientation.z = 0.0;  // Default  
    Pose.orientation.w = 0.0;  // Default 


    /**
     * base = 83.23 degrees
     * shoulder = -74.64 degrees
     * elbow = 38.38
     * wrist 1 = -53.93
     * wrist 2 -89.82
     * wrist 3 = 354.29 
     */
    std_msgs::msg::Float64MultiArray test;
    test.data = {0.088};  
    grip_pub->publish(test);



    
    //define waypoint
    RCLCPP_INFO(this->get_logger(), "Target position: (%.3f, %.3f, %.3f)", 
                Pose.position.x, Pose.position.y, Pose.position.z);
  

    std::vector<geometry_msgs::msg::Pose> waypoints;
    current_pose.position.z += 0.05;
    waypoints.push_back(current_pose);
  
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.00;
    const double eef_step = 0.0005;
    double fraction = move_group_ptr->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    RCLCPP_INFO(this->get_logger(), "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0); 

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    //execute motion
    move_group_ptr->execute(plan);
  
  return true;

}



void RobotControl::setConstraints() {
  
}

std::shared_ptr<RobotControl> RobotControl::create() {
  // Create the node using shared_ptr
  auto node = std::shared_ptr<RobotControl>(new RobotControl());
  
  // Now shared_from_this() will work because the object is managed by shared_ptr
  RCLCPP_INFO(node->get_logger(), "Creating MoveGroupInterface for ur_onrobot_manipulator");
  
  try {
      node->move_group_ptr = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
          node, "ur_onrobot_manipulator");
          
      node->move_group_ptr->setMaxVelocityScalingFactor(0.01);
      node->move_group_ptr->setMaxAccelerationScalingFactor(0.01);
      
      RCLCPP_INFO(node->get_logger(), "MoveGroupInterface created successfully");
      
  } catch (const std::exception& e) {
      RCLCPP_ERROR(node->get_logger(), "Failed to create MoveGroupInterface: %s", e.what());
      // You could return nullptr here or throw, depending on your error handling preference
      throw;
  }
  
  return node;
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
  double fraction = move_group_ptr->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  RCLCPP_INFO(this->get_logger(), "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0); 
   
  //execute motion
  move_group_ptr->execute(trajectory);

  //pick piece (grab)
  grip_pub->publish(close);

  //cartesian path up to level to manouver to original level 
  Pose.position.z += 0.2;
  waypoints.push_back(Pose);
  fraction = move_group_ptr->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
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
    double fraction = move_group_ptr->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    RCLCPP_INFO(this->get_logger(), "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0); 
     
    //execute motion
    move_group_ptr->execute(trajectory);
  
    //pick piece (grab)
    grip_pub->publish(open);
  
    //cartesian path up to level to manouver to original level 
    Pose.position.z += 0.2;
    waypoints.push_back(Pose);
    fraction = move_group_ptr->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
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

