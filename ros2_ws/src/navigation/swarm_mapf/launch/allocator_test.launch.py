import importlib.util
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# When the scenario is not specified, the goal region is this one
FALLBACK_ZONE = {'x_min': 7.5, 'x_max': 9.0, 'y_min': -1.5, 'y_max': 1.5,
                 'spacing': 1.0}

# Head start of the scenario launch file, before the robots are spawned, to give
# the allocator time to start and subscribe to the topics
SCENARIO_WARMUP_S = 5.0
BARE_WARMUP_S = 2.0

ARGUMENTS = [
    ('scenario', '', 'Scenario to launch: towers, rocks, maze. '
                     'Empty means one is already running'),
    ('planner', 'none', 'Passed to the scenario. none leaves the per-robot '
                        'path planning out, so only the allocation is under '
                        'test; independent | whca | cbs turn it back on'),
    ('allocator', 'nearest', 'Allocation strategy: nearest'),
    ('min_separation', '', 'Minimum distance between goals in metres '
                           '(default: the spacing of the scenario)'),
    ('region_x_min', '', 'Goal region, min X in metres'),
    ('region_x_max', '', 'Goal region, max X in metres'),
    ('region_y_min', '', 'Goal region, min Y in metres'),
    ('region_y_max', '', 'Goal region, max Y in metres'),
    ('robots', "['alvin', 'teodoro', 'simon']",
     'Robot names, in descending priority order'),
    ('use_rviz', 'true', 'Open RViz with the allocation displays'),
]


def _scenario_goal_zones():
    """
    Read SCENARIO_GOAL_ZONES out of the installed scenario launch file

    Returns:
        The scenario -> zone mapping, or an empty dict if the file
        cannot be read
    """
    try:
        path = os.path.join(
            get_package_share_directory('swarm_bringup'),
            'launch', 'scenario.launch.py')
        spec = importlib.util.spec_from_file_location('scenario_launch', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module.SCENARIO_GOAL_ZONES
    except Exception:
        return {}


def _robot_colors(scenario, names):
    """
    Look up each robot's colour in the scenario that spawns it

    Args:
        scenario: scenario name, or empty when none is being launched
        names: robot names, in the order they are sent to the service

    Returns:
        The RGB triplets flattened into one list, or an empty list if
        the table or any of the names could not be resolved
    """
    try:
        path = os.path.join(
            get_package_share_directory('swarm_bringup'),
            'launch', 'scenario.launch.py')
        spec = importlib.util.spec_from_file_location('scenario_launch', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        table = module.SCENARIO_ROBOTS.get(scenario) or module._ROBOTS_OPEN
        by_name = {robot['name']: robot['color'] for robot in table}
        flat = []
        for name in names:
            # the table stores 'r g b a'; the alpha is set by the marker
            flat.extend(float(v) for v in by_name[name].split()[:3])
        return flat
    except Exception:
        return []


def _setup(context, *args, **kwargs):
    """
    Resolve the region from the scenario and build the nodes.
    """
    scenario = LaunchConfiguration('scenario').perform(context)
    zone = _scenario_goal_zones().get(scenario, FALLBACK_ZONE)

    def value(name, key):
        """Take the launch argument if given, else the scenario zone."""
        given = LaunchConfiguration(name).perform(context)
        return float(given) if given else float(zone[key])

    region = {
        'region_x_min': value('region_x_min', 'x_min'),
        'region_x_max': value('region_x_max', 'x_max'),
        'region_y_min': value('region_y_min', 'y_min'),
        'region_y_max': value('region_y_max', 'y_max'),
    }
    separation = value('min_separation', 'spacing')

    names = [n.strip(" '\"") for n in
             LaunchConfiguration('robots').perform(context)
             .strip('[]').split(',') if n.strip(" '\"")]
    goal_colors = _robot_colors(scenario, names)

    actions = []
    warmup = BARE_WARMUP_S
    if scenario:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('swarm_bringup'),
                    'launch', 'scenario.launch.py'])),
            launch_arguments={
                'scenario': scenario,
                'use_rviz': LaunchConfiguration('use_rviz'),
                'planner': LaunchConfiguration('planner'),
            }.items(),
        ))
        warmup = SCENARIO_WARMUP_S

    allocator = Node(
        package='swarm_mapf',
        executable='goal_allocator',
        name='goal_allocator',
        output='screen',
        parameters=[{
            'allocator': LaunchConfiguration('allocator'),
            'goal_colors': goal_colors,
            'use_sim_time': True,
        }],
    )

    client = Node(
        package='swarm_mapf',
        executable='allocation_client',
        name='allocation_client',
        output='screen',
        parameters=[{
            'robots': LaunchConfiguration('robots'),
            'min_separation': separation,
            'use_sim_time': True,
            **region,
        }],
    )

    # A second RViz only makes sense when the scenario did not open one
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_allocation',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('swarm_mapf'), 'rviz', 'allocation.rviz'])],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{'use_sim_time': True}],
    )

    actions.append(allocator)
    actions.append(TimerAction(period=warmup, actions=[client]))
    if not scenario:
        actions.append(rviz)
    return actions


def generate_launch_description():
    """Build the launch description for the allocation test bench."""
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=help_)
         for name, default, help_ in ARGUMENTS]
        + [OpaqueFunction(function=_setup)])
