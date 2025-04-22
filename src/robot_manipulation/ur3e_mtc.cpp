#include "ur3e_mtc.h"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ur3e_mtc");

UR3eMTC::UR3eMTC(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("ur3e_mtc_node", options) }
  , arm_group_name_("ur_manipulator")
  , eef_frame_("tool0")
{
  setupPlanners();
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr UR3eMTC::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void UR3eMTC::setupPlanners()
{
  // Configure planners
  sampling_planner_ = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  interpolation_planner_ = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  cartesian_planner_ = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner_->setMaxVelocityScalingFactor(0.5);
  cartesian_planner_->setMaxAccelerationScalingFactor(0.5);
  cartesian_planner_->setStepSize(.01);
}

void UR3eMTC::setupPlanningScene(bool add_cylinder)
{
  if (!add_cylinder) return;
  
  // Create a cylinder as a target object
  moveit_msgs::msg::CollisionObject object;
  object.id = "target_cylinder";
  object.header.frame_id = "base_link";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.1, 0.02 }; // Height, radius

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.4;
  pose.position.y = 0.2;
  pose.position.z = 0.1;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

mtc::Task UR3eMTC::createPointToPointTask(const geometry_msgs::msg::Pose& start_pose,
                                         const geometry_msgs::msg::Pose& end_pose)
{
  mtc::Task task;
  task.stages()->setName("ur3e point-to-point task");
  task.loadRobotModel(node_);

  // Set task properties
  task.setProperty("group", arm_group_name_);
  task.setProperty("ik_frame", eef_frame_);

  // Get the current state of the robot
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current state");
  task.add(std::move(stage_state_current));

  // Move to start pose
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move to start position", sampling_planner_);
    stage->setGroup(arm_group_name_);
    
    // Set the start pose
    geometry_msgs::msg::PoseStamped start;
    start.header.frame_id = "base_link";
    start.pose = start_pose;
    stage->setGoal(start);
    
    task.add(std::move(stage));
  }

  // Move to end pose
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move to end position", sampling_planner_);
    stage->setGroup(arm_group_name_);
    
    // Set the end pose
    geometry_msgs::msg::PoseStamped end;
    end.header.frame_id = "base_link";
    end.pose = end_pose;
    stage->setGoal(end);
    
    task.add(std::move(stage));
  }

  return task;
}

mtc::Task UR3eMTC::createMoveToTask(const geometry_msgs::msg::Pose& target_pose)
{
  mtc::Task task;
  task.stages()->setName("ur3e move-to task");
  task.loadRobotModel(node_);

  // Set task properties
  task.setProperty("group", arm_group_name_);
  task.setProperty("ik_frame", eef_frame_);

  // Get the current state of the robot
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current state");
  task.add(std::move(stage_state_current));

  // Move to target pose
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move to target position", sampling_planner_);
    stage->setGroup(arm_group_name_);
    
    // Set the target pose
    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = "base_link";
    target.pose = target_pose;
    stage->setGoal(target);
    
    task.add(std::move(stage));
  }

  return task;
}

mtc::Task UR3eMTC::createMoveToNamedPositionTask(const std::string& position_name)
{
  mtc::Task task;
  task.stages()->setName("ur3e move to named position task");
  task.loadRobotModel(node_);

  // Set task properties
  task.setProperty("group", arm_group_name_);
  task.setProperty("ik_frame", eef_frame_);

  // Get the current state of the robot
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current state");
  task.add(std::move(stage_state_current));

  // Move to named target position
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move to " + position_name, sampling_planner_);
    stage->setGroup(arm_group_name_);
    stage->setGoal(position_name);
    task.add(std::move(stage));
  }

  return task;
}

bool UR3eMTC::executeTask(mtc::Task& task, bool execute_motion)
{
  try {
    task.init();
  }
  catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return false;
  }

  if (!task.plan(5)) {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return false;
  }
  
  // Publish solution for visualization
  RCLCPP_INFO(LOGGER, "Publishing solution for visualization");
  task.introspection().publishSolution(*task.solutions().front());
  
  if (!execute_motion) {
    RCLCPP_INFO(LOGGER, "Task planning completed successfully. View the solution in RViz.");
    return true;
  }
  
  // Execute the motion
  RCLCPP_INFO(LOGGER, "Executing planned motion");
  auto result = task.execute(*task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return false;
  }

  RCLCPP_INFO(LOGGER, "Task execution completed successfully");
  return true;
}

bool UR3eMTC::moveToPositions(const geometry_msgs::msg::Pose& start_pose, 
                             const geometry_msgs::msg::Pose& end_pose)
{
  auto task = createPointToPointTask(start_pose, end_pose);
  return executeTask(task, false);  // Set to true if you want to execute the motion
}

bool UR3eMTC::moveToPosition(const geometry_msgs::msg::Pose& target_pose)
{
  auto task = createMoveToTask(target_pose);
  return executeTask(task, false);  // Set to true if you want to execute the motion
}

bool UR3eMTC::moveToNamedPosition(const std::string& position_name)
{
  auto task = createMoveToNamedPositionTask(position_name);
  return executeTask(task, false);  // Set to true if you want to execute the motion
}