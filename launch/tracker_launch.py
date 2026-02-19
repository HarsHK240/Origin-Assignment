import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Define the trajectory tracking node
    tracker_node = Node(
        package='assignment',
        executable='trajectory_node',
        name='robot_navigator',
        output='screen',
        emulate_tty=True, # Ensures log messages are properly formatted in the terminal
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(tracker_node)

    return ld
