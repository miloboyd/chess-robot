#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <geometry_msgs/msg/pose.hpp>
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

namespace mtc = moveit::task_constructor;

class UR3eMTC
{
public:
  UR3eMTC(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  // Movement between two Cartesian positions
  bool moveToPositions(const geometry_msgs::msg::Pose& start_pose, 
                      const geometry_msgs::msg::Pose& end_pose);
  
  // Movement to a single Cartesian position
  bool moveToPosition(const geometry_msgs::msg::Pose& target_pose);
  
  // Movement to a named position (for home position only)
  bool moveToNamedPosition(const std::string& position_name);
  
  // Setup planning scene with optional objects
  void setupPlanningScene(bool add_cylinder = true);

private:
  // Create a task with the given start and end poses
  mtc::Task createPointToPointTask(const geometry_msgs::msg::Pose& start_pose,
                                  const geometry_msgs::msg::Pose& end_pose);
  
  // Create a task to move to a specific pose
  mtc::Task createMoveToTask(const geometry_msgs::msg::Pose& target_pose);
  
  // Create a task to move to a named position
  mtc::Task createMoveToNamedPositionTask(const std::string& position_name);
  
  // Execute a planned task
  bool executeTask(mtc::Task& task, bool execute_motion = false);
  
  // Create and configure planners
  void setupPlanners();

  // Member variables
  std::shared_ptr<mtc::solvers::PipelinePlanner> sampling_planner_;
  std::shared_ptr<mtc::solvers::JointInterpolationPlanner> interpolation_planner_;
  std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner_;
  
  rclcpp::Node::SharedPtr node_;
  std::string arm_group_name_;
  std::string eef_frame_;
};