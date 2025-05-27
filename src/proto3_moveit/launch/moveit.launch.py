from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import yaml
import os
from os.path import join

def generate_launch_description():

    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True",
    )

    is_sim = LaunchConfiguration("is_sim")

    # Load controller configuration to verify it's correct
    controller_yaml_path = os.path.join(
        get_package_share_directory("proto3_moveit"),
        "config",
        "moveit_controllers.yaml"
    )
    
    # Debug: Print the controller configuration
    with open(controller_yaml_path, 'r') as file:
        controller_config = yaml.safe_load(file)
        print("\n=== MoveIt Controller Configuration ===")
        print(yaml.dump(controller_config, default_flow_style=False))
        print("=====================================\n")

    moveit_config = (MoveItConfigsBuilder("proto3", package_name="proto3_moveit")
                     .robot_description(join(get_package_share_directory("proto3_description"), "urdf", "proto3.urdf.xacro"))
                     .robot_description_semantic(join("config/proto3.srdf"))
                     .trajectory_execution(file_path="config/moveit_controllers.yaml")
                     .to_moveit_configs()
                    )
    
    # Print what MoveIt thinks the controllers are
    print("\n=== MoveIt Configuration Summary ===")
    print("Robot Name:", moveit_config.robot_description_semantic.get('robot_name', 'Unknown'))
    print("Planning Group:", 'arm')
    
    # Additional parameters to ensure controller connection
    additional_params = {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            additional_params,
            {'use_sim_time': is_sim},
            {'publish_robot_description_semantic': True},
        ],
        arguments=['--ros-args', '--log-level', 'info'],  # Set to debug for more info
    )

    rviz_config = os.path.join(
        get_package_share_directory("proto3_moveit"), 'config', 'moveit.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([
        is_sim_arg,
        move_group_node,
        rviz_node,
    ])