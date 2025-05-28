from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    is_sim_arg = DeclareLaunchArgument("is_sim", default_value="True")
    is_sim = LaunchConfiguration("is_sim")

    # Use MoveItConfigsBuilder with correct paths
    moveit_config = (
        MoveItConfigsBuilder("proto3", package_name="proto3_moveit")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("proto3_description"), 
                "urdf", "proto3.urdf.xacro"
            )
        )
        .robot_description_semantic(
            file_path=os.path.join(
                get_package_share_directory("proto3_moveit"),
                "config", "proto3.srdf"
            )
        )
        .trajectory_execution(
            file_path=os.path.join(
                get_package_share_directory("proto3_moveit"),
                "config", "moveit_controllers.yaml"
            )
        )
        .robot_description_kinematics(
            file_path=os.path.join(
                get_package_share_directory("proto3_moveit"),
                "config", "kinematics.yaml"
            )
        )
        .planning_pipelines(
            pipelines=["ompl"]
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': is_sim},
        ],
    )

    rviz_config = os.path.join(
        get_package_share_directory("proto3_moveit"), 'config', 'moveit.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([is_sim_arg, move_group_node, rviz_node])