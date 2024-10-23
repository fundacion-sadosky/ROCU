from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import math

def generate_launch_description():
    
    world_file = PathJoinSubstitution(
        [FindPackageShare("hibachi_gazebo"),
        "worlds",
        "turtlebot3_world.world"],
    )

    gazebo_launch = PathJoinSubstitution(
        [FindPackageShare("hibachi_gazebo"),
        "launch",
        "hibachi_gazebo.launch.py"],
    )

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments = {
            'world_path': world_file,
            'x': '0.0',
            'y': '-2.0',
            'yaw': str(math.pi/2.0),
        }.items(),
    )

    ld = LaunchDescription([])
    ld.add_action(gazebo_sim)

    return ld