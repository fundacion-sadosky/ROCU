from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, TextSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():

    # DeclareLaunchArgument(
    #     "Parameter_launch_argument", default_value=TextSubstitution(text=str("Parameter_launch")),
    #     description="Parameter launch default value"
    # ),

    # DeclareLaunchArgument(
    #     name = "log_level",
    #     default_value = TextSubstitution(text=str("debug")),
    #     description="Logging level"
    # ),

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("hibachi_description"), "urdf", "hibachi.urdf.xacro"]
            ),
            " ",
            "name:=hibachi",
            " ",
            "prefix:=''",
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    config_hibachi_velocity_controller = PathJoinSubstitution(
        [FindPackageShare("hibachi_hardware"),
        "config",
        "diff_drive_controller.yaml"],
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    node_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, config_hibachi_velocity_controller],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        arguments=['--ros-args', '--log-level', ['HibachiHardware:=', 'debug']]
    )

    # spawn_controller = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster"],
    #     output="screen",
    # )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen",
    )

    spawn_husky_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        # arguments=["hibachi_base_controller"],
        arguments=[
            "hibachi_base_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_controller_manager)
    # ld.add_action(spawn_controller)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(spawn_husky_velocity_controller)

    return ld
