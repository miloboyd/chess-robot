from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('ur3e_mtc_demo')
    
    # Load kinematics.yaml
    kinematics_yaml = load_yaml('ur3e_mtc_demo', 'config/kinematics.yaml')
    
    # Load joint_limits.yaml
    joint_limits_yaml = load_yaml('ur3e_mtc_demo', 'config/joint_limits.yaml')
    
    # Load ompl_planning.yaml
    ompl_planning_yaml = load_yaml('ur3e_mtc_demo', 'config/ompl_planning.yaml')
    
    # Create a dictionary for all parameters
    parameters = []
    if kinematics_yaml:
        parameters.append(kinematics_yaml)
    if joint_limits_yaml:
        parameters.append(joint_limits_yaml)
    if ompl_planning_yaml:
        parameters.append(ompl_planning_yaml)
    
    # Add direct kinematics parameters as a fallback
    parameters.append({
        "robot_description_kinematics.ur_manipulator.kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
        "robot_description_kinematics.ur_manipulator.kinematics_solver_search_resolution": 0.005,
        "robot_description_kinematics.ur_manipulator.kinematics_solver_timeout": 0.005,
        "robot_description_kinematics.ur_manipulator.kinematics_solver_attempts": 3
    })
    
    # MTC Demo node with explicit configurations
    ur3e_mtc_demo = Node(
        package="ur3e_mtc_demo",
        executable="ur3e_mtc_demo",
        output="screen",
        parameters=parameters,
    )

    return LaunchDescription([ur3e_mtc_demo])