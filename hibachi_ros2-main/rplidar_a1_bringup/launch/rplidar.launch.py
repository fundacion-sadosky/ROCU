from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/rplidar-a1',
                'serial_baudrate': 115200,  # A1 / A2
                'frame_id': 'rplidar_a1',
                'inverted': False,
                'angle_compensate': True,
                # 'scan_mode': 'Standard',
            }],
        ),
    ])
