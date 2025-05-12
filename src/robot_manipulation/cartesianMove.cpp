#include <cartesianMove.h>

geometry_msgs::msg::Pose start_pose = move_group_interface_.getCurrentPose().pose;

// Define target pose
geometry_msgs::msg::Pose target_pose = start_pose;
target_pose.position.x += 0.2;  // Move 20 cm along X, for example
target_pose.position.y += 0.0;
target_pose.position.z += 0.0;

std::vector<geometry_msgs::msg::Pose> waypoints;
waypoints.push_back(target_pose);

// Plan Cartesian path
moveit_msgs::msg::RobotTrajectory trajectory;
const double eef_step = 0.01; // resolution of path in meters
const double jump_threshold = 0.0; // disable jump detection
double fraction = move_group_interface_.computeCartesianPath(
    waypoints, eef_step, jump_threshold, trajectory);

if (fraction > 0.9) {
  RCLCPP_INFO(this->get_logger(), "Cartesian path planned %.2f%% of the way", fraction * 100.0);
  moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
  cartesian_plan.trajectory_ = trajectory;
  move_group_interface_.execute(cartesian_plan);
} else {
  RCLCPP_WARN(this->get_logger(), "Only %.2f%% of the path was planned", fraction * 100.0);
}