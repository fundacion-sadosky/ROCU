from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_path = PathJoinSubstitution(
        [FindPackageShare("hibachi_description"), 'urdf', 'hibachi.urdf.xacro']
    )

    # https://articulatedrobotics.xyz/mobile-robot-12a-ros2-control-extra/
    robot_description = Command(['xacro ', urdf_path, ' is_sim:=', use_sim_time])

    node_robot_state_publisher = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output = 'screen',
        parameters = [{ 'robot_description': robot_description,
                        'use_sim_time': use_sim_time }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        node_robot_state_publisher,
    ])