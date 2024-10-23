from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_file = LaunchConfiguration('map')
    params_yaml_file = LaunchConfiguration('params')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution(
            [FindPackageShare('hibachi_navigation'),
             'maps',
             'turtlebot3_world.yaml']),
        description='Full path to map yaml file to load')
    
    declare_params_yaml_cmd = DeclareLaunchArgument(
        'params',
        default_value=PathJoinSubstitution(
            [FindPackageShare('hibachi_navigation'),
             'config',
             'nav2_params.yaml']),
        description='Full path to param yaml file to load')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'localization_launch.py']),
        ),
        launch_arguments = {'use_sim_time': use_sim_time,
                            'params_file': params_yaml_file,
                            'map': map_yaml_file}.items()
    )

    ld = LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
    ])
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_yaml_cmd)
    ld.add_action(localization)
    return ld
