#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ur3e_mtc_demo");
namespace mtc = moveit::task_constructor;

class UR3eMTCDemo
{
public:
  UR3eMTCDemo(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  
  // Helper function to add Cartesian movement stages to the task
  void addMoveToCartesianPoseStage(
      mtc::Task& task,
      const std::string& stage_name,
      const std::shared_ptr<mtc::solvers::CartesianPath>& planner,
      const std::string& arm_group_name,
      const std::string& eef_frame,
      double x, double y, double z,
      double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 1.0);
  
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr UR3eMTCDemo::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

UR3eMTCDemo::UR3eMTCDemo(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("ur3e_mtc_node", options) }
{
}

void UR3eMTCDemo::setupPlanningScene()
{
  // Create a cylinder as a target object
  moveit_msgs::msg::CollisionObject object;
  object.id = "target_cylinder";
  object.header.frame_id = "base_link";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.1, 0.02 }; // Height, radius

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.4;   // Adjust position based on your robot's workspace
  pose.position.y = 0.2;
  pose.position.z = 0.1;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

void UR3eMTCDemo::addMoveToCartesianPoseStage(
    mtc::Task& task,
    const std::string& stage_name,
    const std::shared_ptr<mtc::solvers::CartesianPath>& planner,
    const std::string& arm_group_name,
    const std::string& eef_frame,
    double x, double y, double z,
    double qx, double qy, double qz, double qw)
{
  // Create a new stage for moving to a Cartesian pose
  auto stage = std::make_unique<mtc::stages::GeneratePose>(stage_name);
  stage->properties().configureInitFrom(mtc::Stage::PARENT);
  stage->properties().set("marker_ns", "target_pose");
  
  // Set the target pose
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = "base_link";
  target_pose.pose.position.x = x;
  target_pose.pose.position.y = y;
  target_pose.pose.position.z = z;
  target_pose.pose.orientation.x = qx;
  target_pose.pose.orientation.y = qy;
  target_pose.pose.orientation.z = qz;
  target_pose.pose.orientation.w = qw;
  stage->setPose(target_pose);
  
  // Compute IK for the pose
  auto wrapper = std::make_unique<mtc::stages::ComputeIK>("IK: " + stage_name, std::move(stage));
  wrapper->setMaxIKSolutions(8);
  wrapper->setIKFrame(eef_frame);
  wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
  wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
  
  // Connect to the target pose using the planner
  auto connect = std::make_unique<mtc::stages::Connect>(
      "connect to " + stage_name,
      mtc::stages::Connect::GroupPlannerVector{{arm_group_name, planner}});
  connect->properties().configureInitFrom(mtc::Stage::PARENT);
  
  // Add both stages to the task
  task.add(std::move(wrapper));
  task.add(std::move(connect));
}

void UR3eMTCDemo::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  
  // Publish solution for visualization
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  RCLCPP_INFO(LOGGER, "Task execution completed successfully");
  return;
}

mtc::Task UR3eMTCDemo::createTask()
{
  mtc::Task task;
  task.stages()->setName("ur3e demo task");
  task.loadRobotModel(node_);

  // UR3e specific group names - use "ur_manipulator" from the SRDF
  const auto& arm_group_name = "ur_manipulator";
  const auto& eef_frame = "tool0";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("ik_frame", eef_frame);

  // Get the current state of the robot
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current state");
  task.add(std::move(stage_state_current));

  // Configure planners
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.5);
  cartesian_planner->setMaxAccelerationScalingFactor(0.5);
  cartesian_planner->setStepSize(.01);

  // Move to home position using named state from SRDF
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move to home", sampling_planner);
    stage->setGroup(arm_group_name);
    stage->setGoal("home"); // This named state exists in the SRDF
    task.add(std::move(stage));
  }

  // Now add a sequence of Cartesian movements to specific 3D coordinates
  
  // Move to first Cartesian position
  addMoveToCartesianPoseStage(
      task, "move to position 1", 
      sampling_planner, 
      arm_group_name, 
      eef_frame,
      0.4, 0.2, 0.3,  // x, y, z coordinates
      0.0, 0.0, 0.0, 1.0  // Quaternion orientation (default is no rotation)
  );
  
  // Move to second Cartesian position
  addMoveToCartesianPoseStage(
      task, "move to position 2", 
      sampling_planner, 
      arm_group_name, 
      eef_frame,
      0.4, -0.2, 0.3,  // x, y, z coordinates
      0.0, 0.0, 0.0, 1.0  // Quaternion orientation
  );
  
  // Move to third Cartesian position with a different orientation
  // This orientation would rotate the end effector 90 degrees around the Z axis
  addMoveToCartesianPoseStage(
      task, "move to position 3 with rotation", 
      sampling_planner, 
      arm_group_name, 
      eef_frame,
      0.4, 0.0, 0.2,  // x, y, z coordinates
      0.0, 0.0, 0.7071, 0.7071  // Quaternion for 90-degree rotation around Z
  );

  // Example of using Cartesian planner for a linear move down
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("move down", cartesian_planner);
    stage->setGroup(arm_group_name);
    stage->setIKFrame(eef_frame);

    // Move straight down
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "base_link";
    vec.vector.z = -0.05;  // Move down by 5cm
    stage->setDirection(vec);
    task.add(std::move(stage));
  }

  // Example of using Cartesian planner for a linear move up
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("move up", cartesian_planner);
    stage->setGroup(arm_group_name);
    stage->setIKFrame(eef_frame);

    // Move straight up
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "base_link";
    vec.vector.z = 0.05;  // Move up by 5cm
    stage->setDirection(vec);
    task.add(std::move(stage));
  }

  // Return to home position using named state
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return to home", sampling_planner);
    stage->setGroup(arm_group_name);
    stage->setGoal("home"); // Named state from SRDF
    task.add(std::move(stage));
  }

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_demo_node = std::make_shared<UR3eMTCDemo>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_demo_node]() {
    executor.add_node(mtc_demo_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_demo_node->getNodeBaseInterface());
  });

  mtc_demo_node->setupPlanningScene();
  mtc_demo_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}