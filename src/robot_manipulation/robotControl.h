/**
 * @file robotControl.h
 * @brief Header file for robotControl implementaiton
 * @author MiloBoyd
 * @date 2025-05-13
 */


#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <math.h>
#include <thread>
#include <chrono>
#include <memory>
#include <atomic>
#include <vector>
#include <fstream> 

#include "safetyManager.h"



class RobotControl {

public:

    /**
     * @brief Constructor that receives the nodes for the robot gripper and ur3e arm to manipulate.
     */
    explicit RobotControl(rclcpp::Node::SharedPtr node, SafetyManager* safety_manager);
    
    /**
     * @brief Destructor that is called at the end of robot task.
     */
    ~RobotControl() = default;

    bool initialise();

//////////////////////MOVEMENT COMMANDS//////////////////////

    /**
     * @brief Moves robot arm above chess piece positions via cartesian movement.
     */
    bool moveLinear(double x, double y, double z);

    /**
     * @brief Executes pick movement, descending to piece level and clamping on piece position. 
     * @returns Successful execution status. 
     */
    bool pickUpPiece();

    /**
     * @brief Exectures place movement, descending to piece level and placing piece.
     * @returns Successful execution status.
     */
    bool placePiece();

    /**
     * @brief Moves to hardcorded setup position in preparation for chess piece manipulation.
     */
    bool moveBoard(std::array<double, 6> jointValue);

    /**
     * @brief Moves to the initial home position to finish the robot turn. 
     */
    bool moveHome();

    /** 
     * @brief Constructs collision obstacles in workspace to restrict robot movement.
     * */ 
    void setUpPlanningScene();

    /**
     * @brief Restrict joint angle limits.
     * Joint angle limits will be restricted to prevent movement behind the workspace. This is to simplify possible path planning permutations. 
     */
    void setConstraints();



private:
    //initialise class member variables
    rclcpp::Node::SharedPtr node_;
    SafetyManager* safety_manager_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_ptr_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr grip_pub_;

    std_msgs::msg::Float64MultiArray open_;
    std_msgs::msg::Float64MultiArray close_;

    bool executeCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints);
    bool executeJointSpacePlan(const geometry_msgs::msg::Pose& target);
    void setupMoveItParameters();
};

 #endif // ROBOT_CONTROL_H