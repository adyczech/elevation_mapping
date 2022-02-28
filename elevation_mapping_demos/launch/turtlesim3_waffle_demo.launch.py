"""Launch file for Elevation Mapping - Realsense D435."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

output_dest = "screen"
log_level = 'INFO'

def generate_launch_description():
  """Return a launch.LaunchDescription object with an elevation_mapping node."""
  pkg_elevation_mapping_demos = get_package_share_directory('elevation_mapping_demos')
  pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
  pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
  launch_dir = os.path.join(pkg_elevation_mapping_demos, 'launch')

  config = os.path.join(
    pkg_elevation_mapping_demos,
    'config',
    'waffle_robot.yaml'
    )

  gazebo_params = os.path.join(
    pkg_elevation_mapping_demos,
    'config',
    'gazebo.yaml'
    )

  print(config)

  # rviz2_config = os.path.join(
  #   pkg_elevation_mapping_demos,
  #   'rviz2',
  #   'elevation_map_visualization.rviz'
  #   )

  world = LaunchConfiguration('world')
  paused = LaunchConfiguration('paused')
  use_sim_time = LaunchConfiguration('use_sim_time')
  gui = LaunchConfiguration('gui')
  headless = LaunchConfiguration('headless')
  debug = LaunchConfiguration('debug')
  verbose = LaunchConfiguration('verbose')
  pose = {'x': LaunchConfiguration('x_pose', default='-3.00'),
          'y': LaunchConfiguration('y_pose', default='-1.00'),
          'z': LaunchConfiguration('z_pose', default='0.01'),
          'R': LaunchConfiguration('roll', default='0.00'),
          'P': LaunchConfiguration('pitch', default='0.00'),
          'Y': LaunchConfiguration('yaw', default='0.00')}
  robot_name = LaunchConfiguration('robot_name')
  robot_sdf = LaunchConfiguration('robot_sdf')

  declare_world_cmd = DeclareLaunchArgument(
    'world',
    default_value=os.path.join(pkg_turtlebot3_gazebo,
      'worlds/turtlebot3_house.world'),
    description='Full path to world model file to load')

  declare_paused_cmd = DeclareLaunchArgument(
    'paused',
    default_value='false',
    description='Start Gazebo in a paused state')

  declare_use_sim_time_cmd = DeclareLaunchArgument(
    'use_sim_time',
    default_value='true',
    description='Tells ROS nodes asking for time to get the Gazebo-published simulation time, published over the ROS topic')

  declare_gui_cmd = DeclareLaunchArgument(
    'gui',
    default_value='true',
    description='Launch the user interface window of Gazebo')

  declare_headless_cmd = DeclareLaunchArgument(
    'headless',
    default_value='false',
    description='Enable gazebo state log recording')

  declare_debug_cmd = DeclareLaunchArgument(
    'debug',
    default_value='false',
    description='Start gzserver (Gazebo Server) in debug mode using gdb')

  declare_verbose_cmd = DeclareLaunchArgument(
    'verbose',
    default_value='true',
    description='Start gazebo in verbose mode')

  declare_robot_name_cmd = DeclareLaunchArgument(
    'robot_name',
    default_value='turtlebot3_waffle',
    description='name of the robot')

  declare_robot_sdf_cmd = DeclareLaunchArgument(
    'robot_sdf',
    default_value=os.path.join(pkg_nav2_bringup, 'worlds', 'waffle.model'),  # turtlebot3_gazebo version does not have depth camera
    description='Full path to robot sdf file to spawn the robot in gazebo')

  # Load robot_description param for tf, rviz and gazebo spawn.
  urdf = os.path.join(pkg_nav2_bringup, 'urdf', 'turtlebot3_waffle.urdf')
  with open(urdf, 'r') as infp:
    robot_description = infp.read()

  # Publish turtlebot3 tf's
  start_robot_state_publisher_cmd = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[{'use_sim_time': use_sim_time,
                  'robot_description': robot_description}])

  # Start gazebo server with turtlebot3_house scene
  if (verbose):
    gazebo_server_cmd = cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
        '-s', 'libgazebo_ros_factory.so', robot_sdf, '--verbose', '--ros-args', '--params-file', gazebo_params]
  else:
    gazebo_server_cmd = cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
        '-s', 'libgazebo_ros_factory.so', robot_sdf, '--ros-args', '--params-file', gazebo_params]

  start_gazebo_server_cmd = ExecuteProcess(
    cmd=cmd,
    cwd=[launch_dir], output='screen')

  start_gazebo_client_cmd = ExecuteProcess(
    condition=IfCondition(gui),
    cmd=['gzclient'],
    cwd=[launch_dir], output='screen')

  # Spawn turtlebot into gazebo based on robot_description
  # start_gazebo_spawner_cmd = Node(
  #   package='gazebo_ros',
  #   executable='spawn_entity.py',
  #   output='screen',
  #   arguments=[
  #     '-entity', robot_name,
  #     '-file', robot_sdf,
  #     # '-robot_namespace', namespace,
  #     '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
  #     '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']]
  #     )

  # Publish tf 'base_footprint' as pose
  start_tf_to_pose_publisher_cmd = Node(
    package='elevation_mapping_demos',
    executable='tf_to_pose_publisher.py',
    output='screen',
    parameters=[{'from_frame': 'odom',
                  'to_frame': 'base_footprint',
                  'use_sim_time': True}]
  )

  # TODO(SivertHavso): Run a passthrough filter to down-sample the sensor point cloud

  # Launch elevation mapping node
  start_elevation_mapping_cmd = Node(
    package='elevation_mapping',
    executable='elevation_mapping_node',
    parameters = [config, {'use_sim_time': True}],
    output='screen',
    arguments=['--ros-args', '--log-level', log_level],
    # prefix=['valgrind'],
    # prefix=['gdb -ex run --args']
    prefix=['lldb-12 --arch' 'x86_64 -- ']
    # respawn=True
    )

  start_rviz2_cmd = Node(
    package='rviz2',
    executable='rviz2',
    # parameters = [rviz2_config],
    output='screen',
    arguments=['--ros-args', '--log-level', 'INFO'],
    # respawn=True
    )


  ld = LaunchDescription()
  ld.add_action(declare_world_cmd)
  ld.add_action(declare_paused_cmd)
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_gui_cmd)
  ld.add_action(declare_headless_cmd)
  ld.add_action(declare_debug_cmd)
  ld.add_action(declare_verbose_cmd)
  ld.add_action(declare_robot_name_cmd)
  ld.add_action(declare_robot_sdf_cmd)
  
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)
  # ld.add_action(start_gazebo_spawner_cmd)
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(start_tf_to_pose_publisher_cmd)
  # ld.add_action(start_elevation_mapping_cmd)
  # ld.add_action(start_rviz2_cmd)

  return ld
