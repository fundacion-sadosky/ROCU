from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    joy_vel = LaunchConfiguration('joy_vel')
    joy_config = LaunchConfiguration('joy_config')
    joy_dev = LaunchConfiguration('joy_dev')
    config_filepath = LaunchConfiguration('config_filepath')
    

    ARGUMENTS = [
        DeclareLaunchArgument('use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('joy_vel', default_value='cmd_vel_joy'),
        DeclareLaunchArgument('joy_config', default_value='logitech_f710.config.yaml'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        DeclareLaunchArgument('config_filepath', default_value=[
            PathJoinSubstitution([FindPackageShare('hibachi_teleop'), 'config', joy_config])
        ]),
    ]

    spawn_joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'dev': joy_dev,
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }])

    
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[config_filepath, {'use_sim_time': use_sim_time}],
        remappings={('/cmd_vel', joy_vel)},
    )
    
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(spawn_joy_node)
    ld.add_action(teleop_twist_joy_node)

    return ld