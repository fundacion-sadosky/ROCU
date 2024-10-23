"""Launch Gazebo server and client via include and launch the gazebo.launch.py file."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

def generate_launch_description():

    ARGUMENTS = [
        DeclareLaunchArgument('use_sim_time', default_value='true',
                          description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('world_path', default_value='',
                          description='The world path, by default is empty.world'),
        DeclareLaunchArgument('x', default_value='0',
                          description='x-axis initial position'),
        DeclareLaunchArgument('y', default_value='0',
                          description='y-axis initial position'),
        DeclareLaunchArgument('yaw', default_value='0',
                          description='YAW initial orientation'),
    ]
    
    world_path = LaunchConfiguration('world_path')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    gz_resource_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[
                                                EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
                                                '/usr/share/gazebo-11/models/:',
                                                PathJoinSubstitution([FindPackageShare("hibachi_gazebo"), "models"]),
                                                ])

    pkg_gazebo_ros = PathJoinSubstitution([FindPackageShare('gazebo_ros')])

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py']),
        ),
        launch_arguments = {'world': world_path}.items(),
    )

    # Launch robot_description
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('hibachi_description'), 'launch', 'hibachi_description.launch.py']),
        ),
        launch_arguments= {'use_sim_time': use_sim_time}.items(),
    )

    # config_hibachi_velocity_controller = PathJoinSubstitution(
    #     [FindPackageShare("hibachi_hardware"),
    #     "config",
    #     "diff_drive_controller.yaml"],
    # )

    # node_controller_manager = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[config_hibachi_velocity_controller],
    #     output={
    #         "stdout": "screen",
    #         "stderr": "screen",
    #     },
    #     arguments=['--ros-args', '--log-level', ['HibachiHardware:=', 'info']]
    # )

    spawn_hibachi_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        # arguments=["hibachi_base_controller"],
        arguments=[
            "hibachi_base_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
        parameters=[{ 'use_sim_time': use_sim_time}],
    )

    spawn_hibachi_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
        parameters=[{ 'use_sim_time': use_sim_time}],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "hibachi",
            "-topic", "robot_description",
            "-x", x,
            "-y", y,
            # "-z", ,
            "-Y", yaw,
        ],
        output="screen",
    )


    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_resource_path)
    ld.add_action(robot_description)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    ld.add_action(spawn_hibachi_joint_state_broadcaster)
    ld.add_action(spawn_hibachi_velocity_controller)

    return ld
