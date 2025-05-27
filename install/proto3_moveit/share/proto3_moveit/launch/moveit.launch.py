from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

from os.path import join

def generate_launch_description():

    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value= "True",
    )

    is_sim = LaunchConfiguration("is_sim")

    moveit_config = (MoveItConfigsBuilder("proto3", package_name="proto3_moveit")
                     .robot_description(join(get_package_share_directory("proto3_description"), "urdf", "proto3.urdf.xacro"))
                     .robot_description_semantic(join("config/proto3.srdf"))
                     .trajectory_execution(file_path="config/moveit_controllers.yaml")
                     .to_moveit_configs()
                    )
    
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': is_sim},
            {'publish_robot_description_semantic': True},
        ],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    rviz_config = join(
        get_package_share_directory("proto3_moveit"), 'config', 'moveit.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='QnionView',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits],
    )

    return LaunchDescription([
        is_sim_arg,
        move_group_node,
        rviz_node,
    ])

