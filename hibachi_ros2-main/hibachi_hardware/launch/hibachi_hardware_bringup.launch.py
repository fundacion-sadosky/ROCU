from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    ARGUMENTS = []

    # Launch robot_description
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('hibachi_description'), 'launch', 'hibachi_description.launch.py']),
        ),
        launch_arguments= {'use_sim_time': use_sim_time}.items(),
    )

    urdf_path = PathJoinSubstitution(
        [FindPackageShare("hibachi_description"), 'urdf', 'hibachi.urdf.xacro']
    )

    robot_description = Command(['xacro ', urdf_path, ' is_sim:=', use_sim_time])

    config_hibachi_velocity_controller = PathJoinSubstitution(
        [FindPackageShare("hibachi_hardware"),
        "config",
        "diff_drive_controller.yaml"],
    )

    node_robot_state_publisher = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output = 'screen',
        parameters = [{ 'robot_description': robot_description,
                        'use_sim_time': use_sim_time }]
    )

    node_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description, 'is_sim': use_sim_time}, config_hibachi_velocity_controller],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        arguments=['--ros-args', '--log-level', ['HibachiHardware:=', 'info']]  # log-level can be info or debug
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters = [{'use_sim_time': use_sim_time}],
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen",
    )

    spawn_hibachi_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        parameters = [{'use_sim_time': use_sim_time}],
        arguments=[
            "hibachi_base_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )


    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_controller_manager)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(spawn_hibachi_velocity_controller)
    return ld

