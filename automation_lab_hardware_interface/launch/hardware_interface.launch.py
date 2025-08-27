from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': '<robot URDF content>'},  # Replace with your URDF content
                '/path/to/controllers.yaml'                      # Replace with your YAML file path
            ]
        ),
        Node(
            package='controller_manager',
            executable='spawner.py',
            arguments=['joint_state_controller'],
        ),
        Node(
            package='controller_manager',
            executable='spawner.py',
            arguments=['position_trajectory_controller'],
        ),
    ])

