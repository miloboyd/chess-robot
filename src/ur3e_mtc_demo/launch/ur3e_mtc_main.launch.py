import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Load configuration for the UR3e
    moveit_config = MoveItConfigsBuilder("ur").to_dict()

    # Include the simulation launch file
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ur_simulation_gz'), 'launch'),
            '/ur_sim_control.launch.py'
        ]),
        launch_arguments={
            'ur_type': 'ur3e',
            'launch_rviz': 'false'
        }.items()
    )

    # Include the MoveIt launch file
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ur_moveit_config'), 'launch'),
            '/ur_moveit.launch.py'
        ]),
        launch_arguments={
            'ur_type': 'ur3e',
            'use_sim_time': 'true'
        }.items()
    )

    # Add the Motion Planning Tasks display and configuration for MTC
    rviz_config_file = os.path.join(
        get_package_share_directory("ur3e_mtc_demo"),  # Replace with your package name
        "config",
        "ur3e_mtc.rviz"
    )
    
    # MTC Demo node
    ur3e_mtc_demo = Node(
        package="ur3e_mtc_demo",  # Change this to your package name
        executable="ur3e_mtc_demo",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )

    return LaunchDescription([
        simulation_launch,
        moveit_launch,
        ur3e_mtc_demo
    ])