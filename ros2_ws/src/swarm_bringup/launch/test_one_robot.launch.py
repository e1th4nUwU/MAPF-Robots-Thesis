"""
Minimal test: ONE robot in an EMPTY world.

Root cause of previous failures:
  Gazebo Harmonic does NOT load model-level system plugins (DiffDrive, etc.)
  from models spawned dynamically via 'ros_gz_sim create'. Plugins only
  load reliably when the model is declared inside the world SDF from the start.

Fix: build a complete world SDF that embeds the robot, then launch Gazebo
     with that world file. No separate spawn step needed.
"""

import os
import re
import subprocess
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

# Inline ground plane avoids relying on model://ground_plane being available
_GROUND_PLANE = """\
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient></material>
        </visual>
      </link>
    </model>
"""


def build_world_with_robot(urdf_string: str, robot_name: str,
                           x=0.0, y=0.0, z=0.5) -> str:
    """
    URDF -> SDF -> rename model -> embed in world SDF -> write temp file.
    Returns the path to the generated world SDF file.
    """
    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
        f.write(urdf_string)
        urdf_path = f.name
    try:
        result = subprocess.run(
            ['gz', 'sdf', '-p', urdf_path],
            capture_output=True, text=True, check=True
        )
    finally:
        os.unlink(urdf_path)

    # stdout = clean SDF XML; Gazebo warnings go to stderr
    model_sdf = result.stdout

    # Rename model and inject spawn pose
    model_sdf = model_sdf.replace(
        "<model name='justina'>",
        f"<model name='{robot_name}'>\n    <pose>{x} {y} {z} 0 0 0</pose>",
        1,
    )

    # Extract just the <model ...>...</model> element
    match = re.search(r'(<model\b.*?</model>)', model_sdf, re.DOTALL)
    if not match:
        raise RuntimeError('gz sdf -p produced no <model> element')
    model_xml = match.group(1)

    world_sdf = f"""\
<?xml version="1.0" ?>
<sdf version="1.11">
  <world name="default">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
    </light>
{_GROUND_PLANE}
    {model_xml}
  </world>
</sdf>
"""

    with tempfile.NamedTemporaryFile(
            mode='w', suffix='.sdf', delete=False, prefix='world_test_') as f:
        f.write(world_sdf)
        return f.name


def generate_launch_description():
    justina_pkg = get_package_share_directory('justina_description')

    robot_name = 'alvin'
    xacro_file = os.path.join(justina_pkg, 'urdf', 'justina.xacro')
    urdf = xacro.process_file(
        xacro_file, mappings={'body_color': '0.9 0.1 0.1 1.0'}
    ).toxml()

    # Build world SDF with robot embedded (plugins load correctly this way)
    world_path = build_world_with_robot(urdf, robot_name)

    # Launch Gazebo with the generated world (robot is already inside)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': f'-r -v 3 {world_path}'}.items(),
    )

    # Robot State Publisher (provides TF from URDF)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf, 'use_sim_time': True}],
        output='screen',
    )

    # Bridge: clock + odom (GZ->ROS) + cmd_vel (ROS->GZ)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
    )

    return LaunchDescription([gazebo, rsp, bridge])
