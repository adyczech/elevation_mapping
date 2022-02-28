"""Launch file for Elevation Mapping - Realsense D435."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

output_dest = "screen"
log_level = "INFO"

def generate_launch_description():
  """Return a launch.LaunchDescription object with an elevation_mapping node."""
  pkg_elevation_mapping_demos = get_package_share_directory('elevation_mapping_demos')

  config = os.path.join(
    pkg_elevation_mapping_demos,
    'config',
    'elevation_mapping',
    'simple_demo.yaml'
    )

  rviz2_config = os.path.join(
    pkg_elevation_mapping_demos,
    'rviz2',
    'elevation_map_visualization.rviz'
    )

  visualization_launch_path = os.path.join(get_package_share_directory("elevation_mapping_demos"),
                 'launch',
                 'visualization.launch.py')

  return LaunchDescription([
    Node(
      package='elevation_mapping',
      namespace='',
    #   name='elevation_mapping',
      executable='elevation_mapping_node',
      parameters = [config],
      output={"both": output_dest},
      emulate_tty=False,
      arguments=['--ros-args', '--log-level', log_level],
      respawn=True
      ),
    # Node(
    #   package='rviz2',
    #   namespace='',
    #   name='rviz2',
    #   executable='rviz2',
    #   parameters = [rviz2_config],
    #   output={"both": output_dest},
    #   emulate_tty=False,
    #   arguments=['--ros-args', '--log-level', 'INFO'],
    #   respawn=True
    #   ),
    IncludeLaunchDescription(PythonLaunchDescriptionSource(visualization_launch_path))
  ])
