from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # MTC Demo node - use the node parameters to get settings from MoveIt
    ur3e_mtc_demo = Node(
        package="ur3e_mtc_demo",
        executable="ur3e_mtc_demo",
        output="screen",
        # The parameters will be taken from the MoveIt node that's already running
    )

    return LaunchDescription([ur3e_mtc_demo])