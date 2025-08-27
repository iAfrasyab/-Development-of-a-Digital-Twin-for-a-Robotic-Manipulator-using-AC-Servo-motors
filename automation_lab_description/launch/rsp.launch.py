from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Path to URDF
    pkg_share = os.path.join(os.getenv('HOME'), 'ros2_ws', 'src', 'automation_lab_description')
    default_model_path = os.path.join(pkg_share, 'urdf', 'automation_lab_description.urdf')

    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'robot_description_file',
            default_value=default_model_path,
            description='Path to the URDF file',
        ),
    ]

    # Robot description
    robot_description_content = Command([
        'xacro ',
        LaunchConfiguration('robot_description_file')
    ])
    robot_description = {
        'robot_description': robot_description_content
    }

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    controller_manager_config = PathJoinSubstitution(
        [
            FindPackageShare('automation_lab_description'),
            'config',
            'controllers.yaml',
        ]
    )
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            controller_manager_config
        ],
        output="both",
    )

    rviz_config_path = os.path.join(pkg_share, 'rviz', 'default.rviz')

    # Corrected: Removed trailing comma to avoid creating a tuple
    state_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    trajectory_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
    )
    hardware_driver_node = Node(
    	package='automation_lab_description',
    	executable='motor_control',
    	output='screen'
    	)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )


    # Ensure all Nodes are properly returned in the LaunchDescription
    return LaunchDescription(declared_arguments + [
        robot_state_publisher_node,
        rviz_node,
        control_node,
        trajectory_controller_node,
        hardware_driver_node,
        state_controller_node

    ])

