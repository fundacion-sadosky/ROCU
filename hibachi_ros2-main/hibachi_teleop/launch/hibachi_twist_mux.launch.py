from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    twist_mux_params = PathJoinSubstitution(
        [FindPackageShare("hibachi_teleop"), 'config', 'twist_mux.yaml']
    )

    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': use_sim_time}],
            # Default publisher for twist_mux is /cmd_vel_out
            # Rename so it matches diff_drive_controller topic
            remappings=[('/cmd_vel_out','/hibachi_base_controller/cmd_vel_unstamped')]
        )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        twist_mux,
    ])