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
 #include <math.h>
 #include <thread>
 #include <chrono>
 #include <memory>


 class RobotControl : public rclcpp::Node {

public:

    static std::shared_ptr<RobotControl> create();

    /**
     * @brief Constructor that receives the nodes for the robot gripper and ur3e arm to manipulate.
     */
    explicit RobotControl();
    
    /**
     * @brief Destructor that is called at the end of robot task.
     */
    ~RobotControl() = default;

    /** 
     * @brief Executes robot movement command to a specified coordinate.
     * 
     * @param x_coordinate, y_coordinate, z_coordinate Grid coordinate 
     */
    bool moveRobot(double x_coordinate, double y_coordinate, double z_coordinate);

    /**
     * @brief Moves to hardcorded setup position in preparation for chess piece manipulation.
     */
    bool moveBoard();

    /**
     * @brief Moves to the initial home position to finish the robot turn. 
     */
    bool moveHome();

    /**
     * @brief Moves robot arm above chess piece positions via cartesian movement.
     */
    bool moveLinear(double x_coordinate, double y_coordinate, double z_coordinate);

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
     * @brief Constructs collision obstacles in workspace to restrict robot movement.
     * */ 
    void setUpPlanningScene();

    /**
     * @brief Restrict joint angle limits.
     * Joint angle limits will be restricted to prevent movement behind the workspace. This is to simplify possible path planning permutations. 
     */
    void setConstraints();

    bool moveJointSpace(double x, double y, double z);

private:
    //initialise class member variables
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr grip_pub;
    std_msgs::msg::Float64MultiArray open;
    std_msgs::msg::Float64MultiArray close;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_ptr;
    


};

 #endif // ROBOT_CONTROL_H