from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    mvn_planning_pkg = get_package_share_directory('motion_planning')
    default_map_config_file = os.path.join(mvn_planning_pkg, 'maps', 'appartment.yaml')
    rviz_config_file = os.path.join(mvn_planning_pkg, 'rviz', 'motion_planning.rviz')
    map_config_file = LaunchConfiguration('map_yaml')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml',
            default_value=default_map_config_file,
            description='Path to occupancy map yaml used by map_server'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2'
        ),
        # ── Shared infrastructure (one instance for all robots) ──────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file, '--ros-args', '-p', 'use_sim_time:=True'],
            condition=IfCondition(use_rviz),
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_config_file}, {'use_sim_time': True}]
        ),
        # NOTE: map→odom TF is published per-robot in scenario.launch.py
        # (each robot has its own map→{name}/odom static transform)
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='nav2_util',
                    executable='lifecycle_bringup',
                    output='screen',
                    arguments=['map_server']
                )
            ]
        ),
        # cost_map exposes /get_inflated_map and /get_cost_map globally
        # so all per-robot a_star nodes can share it
        Node(
            package='path_planner',
            executable='cost_map',
            name='cost_map',
            output='screen',
            parameters=[{'inflation_radius': 0.2}, {'cost_radius': 0.5}]
        ),
        # ── Per-robot navigation is launched separately via robot_nav.launch.py
        # Include it once per robot in scenario.launch.py (or test_one_robot).
    ])
