# ur3e_mtc.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('ur3e_mtc_demo')
    
    # MTC node with kinematics parameters
    mtc_node = Node(
        package="ur3e_mtc_demo",
        executable="ur3e_mtc",
        name="ur3e_mtc",
        output="screen",
        parameters=[
            {"robot_description_kinematics.ur_manipulator.kinematics_solver": 
             "kdl_kinematics_plugin/KDLKinematicsPlugin"},
            {"robot_description_kinematics.ur_manipulator.kinematics_solver_search_resolution": 0.005},
            {"robot_description_kinematics.ur_manipulator.kinematics_solver_timeout": 0.005}
        ],
    )
    return LaunchDescription([mtc_node])