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
 #include <thread>
 #include <chrono>


 /** */
 class RobotControl {

public:

    /**
     * @brief Constructor that receives the nodes for the robot gripper and ur3e arm to manipulate
     */
    RobotControl();


    /**
     * @brief Executes robot movement command to a specified coordinate
     * 
     * @param x_coordinate, y_coordinate, z_coordinate Grid coordinate 
     */
    bool moveRobot(double x_coordinate, double y_coordinate, double z_coordinate);

    bool pickUpPiece();
    bool placePiece();
    bool moveHome();

 };

 #endif // ROBOT_CONTROL_H