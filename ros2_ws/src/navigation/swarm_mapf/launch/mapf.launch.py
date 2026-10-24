"""
The whole system: scenario, allocation and centralised MAPF planning

Replaces the old per robot whca_star nodes. The scenario is launched
with planner:=independent, which brings up a_star, path_smoothing and
pure_pursuit for every robot but none of the old coordination nodes:
the sequencing between robots now comes from mapf_planner, and the per
robot stack only executes the hop it is given
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# When no scenario is given, plan over this region
FALLBACK_ZONE = {'x_min': 7.5, 'x_max': 9.0, 'y_min': -1.5, 'y_max': 1.5,
                 'spacing': 1.0}

# Gazebo, the controllers and cost_map all have to be up before the
# planner asks for the map: cost_map precomputes the inflation and takes
# the best part of twenty seconds on the maze
SCENARIO_WARMUP_S = 25.0
BARE_WARMUP_S = 3.0

ARGUMENTS = [
    ('scenario', 'maze', 'Scenario to launch: towers, rocks, maze. '
                         'Empty means one is already running'),
    ('planner', 'whca', 'MAPF planner: whca. cbs once it exists'),
    ('allocator', 'nearest', 'Goal allocation strategy: nearest'),
    ('robots', "['alvin', 'teodoro', 'simon']",
     'Robot names, in descending priority order. The order is the '
     'priority: the first one never yields'),
    ('cell_size', '0.8', 'Side of a coarse planning cell, in metres. '
                         '0.8 is the value the maze was measured for; '
                         '1.0 closes its narrowest throat'),
    ('window', '8', 'Steps of cooperation, w, and how much of the plan '
                    'is executed before it is rebuilt'),
    ('leg', '2', 'Cells per goal. One cell parks the robot inside the '
                 'disc cost_map paints over it and it never moves '
                 'again; a whole window lets its own A* pick the route '
                 'and ignore the plan. Below 2 the node warns'),
    ('replan', 'true', 'Rebuild the plan every w steps from where the '
                       'robots really are. false plans once and runs it '
                       'whole, which is HCA* and not WHCA*'),
    ('max_rounds', '20', 'Give up after this many planning rounds'),
    ('arrival_tolerance', '0.30', 'How close to a cell centre counts as '
                                  'standing on it, in metres'),
    ('step_timeout', '60.0', 'Seconds to wait for the slowest robot to '
                             'finish a leg before giving up'),
    ('min_separation', '', 'Minimum distance between goals in metres '
                           '(default: the spacing of the scenario)'),
    ('region_x_min', '', 'Goal region, min X in metres'),
    ('region_x_max', '', 'Goal region, max X in metres'),
    ('region_y_min', '', 'Goal region, min Y in metres'),
    ('region_y_max', '', 'Goal region, max Y in metres'),
    ('use_rviz', 'true', 'Open RViz'),
]


def _scenario_module():
    """
    Load the installed scenario launch file

    Returns:
        The module, or None if it cannot be read
    """
    try:
        path = os.path.join(
            get_package_share_directory('swarm_bringup'),
            'launch', 'scenario.launch.py')
        spec = importlib.util.spec_from_file_location('scenario_launch', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    except Exception:
        return None


def _robot_colors(module, scenario, names):
    """
    Look up each robot's colour in the scenario that spawns it

    Args:
        module: the loaded scenario launch module, or None
        scenario: scenario name, or empty when none is being launched
        names: robot names, in the order they are sent to the service

    Returns:
        The RGB triplets flattened into one list, or an empty list
    """
    try:
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
    Resolve the region from the scenario and build the nodes
    """
    # Resolved here and not left as a LaunchConfiguration on purpose.
    # IncludeLaunchDescription shares the launch context, and the scenario
    # declares a `planner` argument of its own: including it with
    # planner:=independent overwrites this one, so anything read lazily
    # afterwards comes back as 'independent'
    scenario = LaunchConfiguration('scenario').perform(context)
    planner_name = LaunchConfiguration('planner').perform(context)
    allocator_name = LaunchConfiguration('allocator').perform(context)
    module = _scenario_module()
    zones = getattr(module, 'SCENARIO_GOAL_ZONES', {}) if module else {}
    zone = zones.get(scenario, FALLBACK_ZONE)

    def value(name, key):
        """Take the launch argument if given, else the scenario zone."""
        given = LaunchConfiguration(name).perform(context)
        return float(given) if given else float(zone[key])

    names = [n.strip(" '\"") for n in
             LaunchConfiguration('robots').perform(context)
             .strip('[]').split(',') if n.strip(" '\"")]

    actions = []
    warmup = BARE_WARMUP_S
    if scenario:
        # Scoped so the scenario's own arguments stay inside it
        actions.append(GroupAction(scoped=True, forwarding=False, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('swarm_bringup'),
                        'launch', 'scenario.launch.py'])),
                launch_arguments={
                    'scenario': scenario,
                    'use_rviz': LaunchConfiguration('use_rviz').perform(context),
                    # independent brings up the per robot stack without
                    # the old coordination nodes. This node is the
                    # coordination
                    'planner': 'independent',
                }.items(),
            )]))
        warmup = SCENARIO_WARMUP_S

    allocator = Node(
        package='swarm_mapf',
        executable='goal_allocator',
        name='goal_allocator',
        output='screen',
        parameters=[{
            'allocator': allocator_name,
            'goal_colors': _robot_colors(module, scenario, names),
            'use_sim_time': True,
        }],
    )

    planner = Node(
        package='swarm_mapf',
        executable='mapf_planner',
        name='mapf_planner',
        output='screen',
        parameters=[{
            'planner': planner_name,
            'robots': names,
            'cell_size': float(
                LaunchConfiguration('cell_size').perform(context)),
            'window': int(LaunchConfiguration('window').perform(context)),
            'leg': int(LaunchConfiguration('leg').perform(context)),
            'replan': LaunchConfiguration('replan').perform(context)
            in ('true', 'True', '1'),
            'max_rounds': int(
                LaunchConfiguration('max_rounds').perform(context)),
            'arrival_tolerance': float(
                LaunchConfiguration('arrival_tolerance').perform(context)),
            'step_timeout': float(
                LaunchConfiguration('step_timeout').perform(context)),
            'min_separation': value('min_separation', 'spacing'),
            'region_x_min': value('region_x_min', 'x_min'),
            'region_x_max': value('region_x_max', 'x_max'),
            'region_y_min': value('region_y_min', 'y_min'),
            'region_y_max': value('region_y_max', 'y_max'),
            'use_sim_time': True,
        }],
    )

    actions.append(LogInfo(msg='MAPF: planificador=%s, enrejado=%s m, w=%s'
                               % (planner_name,
                                  LaunchConfiguration('cell_size').perform(context),
                                  LaunchConfiguration('window').perform(context))))
    actions.append(allocator)
    actions.append(TimerAction(period=warmup, actions=[planner]))
    return actions


def generate_launch_description():
    """Build the launch description for the full MAPF system."""
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=help_)
         for name, default, help_ in ARGUMENTS]
        + [OpaqueFunction(function=_setup)])
