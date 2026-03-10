"""
3 differential-drive robots in Gazebo Harmonic.

Key insight discovered during debugging:
  Gazebo Harmonic does NOT load model system plugins (DiffDrive, etc.)
  from dynamically-spawned models. Plugins only load when the model is
  declared inside the world SDF from the start.
  Each robot gets absolute topic names so they don't collide:
    /alvin/cmd_vel, /teodoro/cmd_vel, /simon/cmd_vel
    /alvin/odom,    /teodoro/odom,    /simon/odom
"""

import os
import re
import subprocess
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
import xacro

ROBOTS = [
    {'name': 'alvin',   'color': '0.9 0.1 0.1 1.0', 'x': 0.0, 'y':  1.5, 'z': 0.5, 'yaw': 0.0},
    {'name': 'teodoro', 'color': '0.1 0.8 0.1 1.0', 'x': 0.0, 'y':  0.0, 'z': 0.5, 'yaw': 0.0},
    {'name': 'simon',   'color': '0.1 0.1 0.9 1.0', 'x': 0.0, 'y': -1.5, 'z': 0.5, 'yaw': 0.0},
]

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


def robot_to_model_xml(urdf_string: str, robot: dict) -> str:
    """Convert a URDF string to a renamed SDF model element with pose."""
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

    model_sdf = result.stdout
    name = robot['name']
    x, y, z, yaw = robot['x'], robot['y'], robot['z'], robot['yaw']

    model_sdf = model_sdf.replace(
        "<model name='justina'>",
        f"<model name='{name}'>\n    <pose>{x} {y} {z} 0 0 {yaw}</pose>",
        1,
    )
    match = re.search(r'(<model\b.*?</model>)', model_sdf, re.DOTALL)
    if not match:
        raise RuntimeError(f'gz sdf -p produced no <model> for {name}')
    return match.group(1)


def build_swarm_world(robots_urdf: list) -> str:
    """Build a world SDF with all robots embedded. Returns temp file path."""
    models_xml = '\n'.join(
        robot_to_model_xml(urdf, robot)
        for urdf, robot in robots_urdf
    )

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
    {models_xml}
  </world>
</sdf>
"""
    with tempfile.NamedTemporaryFile(
            mode='w', suffix='.sdf', delete=False, prefix='world_swarm_') as f:
        f.write(world_sdf)
        return f.name


def generate_launch_description():
    justina_pkg = get_package_share_directory('justina_description')
    xacro_file = os.path.join(justina_pkg, 'urdf', 'justina.xacro')

    # Process one URDF per robot (different color and namespaced topics)
    robots_urdf = []
    for robot in ROBOTS:
        name = robot['name']
        urdf = xacro.process_file(
            xacro_file,
            mappings={
                'body_color':    robot['color'],
                'cmd_vel_topic': f'/{name}/cmd_vel',
                'odom_topic':    f'/{name}/odom',
            }
        ).toxml()
        robots_urdf.append((urdf, robot))

    world_path = build_swarm_world(robots_urdf)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': f'-r -v 3 {world_path}'}.items(),
    )

    # Clock bridge (one global)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    nodes = [gazebo, clock_bridge]

    for robot, (urdf, _) in zip(ROBOTS, robots_urdf):
        name = robot['name']

        # RSP in namespace (for TF)
        nodes.append(GroupAction([
            PushRosNamespace(name),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{
                    'robot_description': urdf,
                    'use_sim_time': True,
                    'frame_prefix': f'{name}/',
                }],
                output='screen',
            ),
        ]))

        # Bridge: odom (GZ->ROS) + cmd_vel (ROS->GZ)
        nodes.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'bridge_{name}',
            output='screen',
            arguments=[
                f'/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                f'/{name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            ],
        ))

    return LaunchDescription(nodes)
