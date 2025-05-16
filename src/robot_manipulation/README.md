This robot manipulation folder details the robot arm manipulation components using the ur3e robot arm and Onrobot RG2 Gripper. This file will detail several steps for downloading and running these files.


# UR3e Robot Control with ROS 2 and MoveIt

This guide explains how to set up and control a Universal Robots UR3e using ROS 2 Humble and MoveIt.

## System Requirements

- Ubuntu 22.04
- ROS 2 Humble

## 1. Install Required Packages

Install the necessary packages from the provided links:
- [Universal Robot ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble) - humble branch
    - [GZ ROS2 Control](https://github.com/ros-controls/gz_ros2_control/tree/humble) - humble branch (optional if complications arise through UR Driver
- [UR OnRobot ROS2 Driver](https://github.com/tonydle/UR_OnRobot_ROS2), which requires:
    - [OnRobot ROS2 Description](https://github.com/tonydle/OnRobot_ROS2_Description)
    - libnet1-dev (for Modbus TCP/Serial)
    - [Modbus](https://github.com/Mazurel/Modbus) C++ library (included as a submodule)
- [MoveIt Humble](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html) 

## 2. Set up Workspaces

### Create and Build UR Workspace

```bash
# Create the workspace
mkdir -p ~/ur_ws/src

# Clone the repositories
cd ~/ur_ws/src
git clone .... provided links above

# Build the workspace
cd ~/ur_ws
colcon build --symlink-install --packages-select rs2

# Source the workspace
source install/setup.bash
```


## 3. Connect to a Real UR3e Robot

### Step 1: Start robot
   ```sh
   ros2 launch ur_onrobot_control start_robot.launch.py ur_type:=ur3e onrobot_type:=rg2 robot_ip:=<robot_ip> launch_rviz:=false
   ```
Common arguments:
- `use_fake_hardware` (default: `false`): Use mock hardware interface for testing
- `launch_rviz` (default: `true`): Launch RViz with the robot model
- `tf_prefix` (default: `""`): Prefix for all TF frames

Replace <robot_IP> with your robot's actual IP address.

### Step 2: Launch MoveIt

In a new terminal:

```sh
ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py ur_type:=ur3e onrobot_type:=rg2
```

### Step 3: Execute the run file

In another terminal:

```bash
ros2 run rs2 robotControl
```

## 4. Simulation

If you want to test without a real robot, you can use a fake hardware system:

```bash
# Install simulation packages above

# build your workspace
cd ~/ur_ws
colcon build --symlink-install --packages-select rs2
source install/setup.bash

# Launch simulation
   ros2 launch ur_onrobot_control start_robot.launch.py ur_type:=ur3e onrobot_type:=rg2 robot_ip:=<robot_ip> launch_rviz:=false use_fake_hardware:=true

#Then launch MoveIt
ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py ur_type:=ur3e onrobot_type:=rg2
```

## 5. Useful Commands

### Check Topic Information

```bash
# List all topics
ros2 topic list

# See details of a specific topic
ros2 topic echo /joint_states

# Check topic publishing rate
ros2 topic hz /joint_states
```

### Check Robot Status

```bash
# See current controller status
ros2 control list_controllers

# Check robot state
ros2 topic echo /controller_manager/status
```

## 6. Common Issues and Solutions

### Robot Not Connecting
- Verify the robot IP address
- Check if the robot is powered on and in Remote Control mode
- Ping the robot: `ping 192.168.0.250` (example IP address)

### Move Group Not Planning
- Check if the robot is in an error state
- Verify joint states are being published: `ros2 topic echo /joint_states`
- Check for collisions in the planning scene

### MoveIt not working 
- Check that you have properly built the workspace and sourced after changes

```bash
colcon build --symlink-install --packages-select ~/path_to_workspace/..
source install/setup.bash
```

## 7. Additional Resources

- [Universal Robots ROS2 Driver GitHub](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [MoveIt Documentation](https://moveit.picknik.ai/humble/index.html)
- [ROS 2 Control Documentation](https://control.ros.org/humble/index.html)
