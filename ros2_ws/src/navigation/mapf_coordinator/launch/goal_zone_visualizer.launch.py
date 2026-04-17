"""
Goal Zone Visualizer — shows goal zone boundary in RViz as a green rectangle.

Usage:
  ros2 launch mapf_coordinator goal_zone_visualizer.launch.py \
    goal_zone_x_min:=7.5 goal_zone_x_max:=9.0 \
    goal_zone_y_min:=-1.0 goal_zone_y_max:=1.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'goal_zone_x_min',
            default_value='7.5',
            description='Goal zone min X (metres)'
        ),
        DeclareLaunchArgument(
            'goal_zone_x_max',
            default_value='9.0',
            description='Goal zone max X (metres)'
        ),
        DeclareLaunchArgument(
            'goal_zone_y_min',
            default_value='-1.0',
            description='Goal zone min Y (metres)'
        ),
        DeclareLaunchArgument(
            'goal_zone_y_max',
            default_value='1.0',
            description='Goal zone max Y (metres)'
        ),
        Node(
            package='mapf_coordinator',
            executable='goal_zone_visualizer',
            name='goal_zone_visualizer',
            output='screen',
            parameters=[{
                'goal_zone_x_min': LaunchConfiguration('goal_zone_x_min'),
                'goal_zone_x_max': LaunchConfiguration('goal_zone_x_max'),
                'goal_zone_y_min': LaunchConfiguration('goal_zone_y_min'),
                'goal_zone_y_max': LaunchConfiguration('goal_zone_y_max'),
                'use_sim_time': True,
            }],
        ),
    ])
