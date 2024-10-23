import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
   hibachi_hardware = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('hibachi_hardware'), 'launch'),
         '/hibachi_hardware.launch.py'])
      )

   ardusimple = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('gnss_bringup'), 'launch'),
         '/ardusimple.launch.py'])
      )

   xsens_mti630 = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('xsens_mti630_bringup'), 'launch'),
         '/mti630.launch.py'])
      )

   phidgets_spatial = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('phidgets_spatial_bringup'), 'launch'),
         '/spatial_imu_filter_component.launch.py'])
      )
#    turtlesim_world_2 = IncludeLaunchDescription(
#       PythonLaunchDescriptionSource([os.path.join(
#          get_package_share_directory('launch_tutorial'), 'launch'),
#          '/turtlesim_world_2.launch.py'])
#       )
#    broadcaster_listener_nodes = IncludeLaunchDescription(
#       PythonLaunchDescriptionSource([os.path.join(
#          get_package_share_directory('launch_tutorial'), 'launch'),
#          '/broadcaster_listener.launch.py']),
#       launch_arguments={'target_frame': 'carrot1'}.items(),
#       )
#    mimic_node = IncludeLaunchDescription(
#       PythonLaunchDescriptionSource([os.path.join(
#          get_package_share_directory('launch_tutorial'), 'launch'),
#          '/mimic.launch.py'])
#       )
#    fixed_frame_node = IncludeLaunchDescription(
#       PythonLaunchDescriptionSource([os.path.join(
#          get_package_share_directory('launch_tutorial'), 'launch'),
#          '/fixed_broadcaster.launch.py'])
#       )
#    rviz_node = IncludeLaunchDescription(
#       PythonLaunchDescriptionSource([os.path.join(
#          get_package_share_directory('launch_tutorial'), 'launch'),
#          '/turtlesim_rviz.launch.py'])
#       )

   return LaunchDescription([
      hibachi_hardware,
      ardusimple,
      # phidgets_spatial,
      xsens_mti630,
   ])