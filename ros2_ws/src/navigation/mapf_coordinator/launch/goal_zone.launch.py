"""
Goal Zone Setup — GNFC allocator + visualizer in one command.

This launches both the goal allocator (assigns goals) and visualizer
(shows rectangle in RViz) with a single command and single goal zone.

Usage:
  ros2 launch mapf_coordinator goal_zone.launch.py \
    x_min:=7.5 x_max:=9.0 y_min:=-1.0 y_max:=1.0 spacing:=0.3
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    x_min = LaunchConfiguration('x_min')
    x_max = LaunchConfiguration('x_max')
    y_min = LaunchConfiguration('y_min')
    y_max = LaunchConfiguration('y_max')
    spacing = LaunchConfiguration('spacing')

    return LaunchDescription([
        DeclareLaunchArgument('x_min', default_value='7.5', description='Zone min X (m)'),
        DeclareLaunchArgument('x_max', default_value='9.0', description='Zone max X (m)'),
        DeclareLaunchArgument('y_min', default_value='-1.0', description='Zone min Y (m)'),
        DeclareLaunchArgument('y_max', default_value='1.0', description='Zone max Y (m)'),
        DeclareLaunchArgument('spacing', default_value='0.3', description='Goal grid spacing (m)'),

        # GNFC Goal Allocator
        Node(
            package='mapf_coordinator',
            executable='gnfc_goal_allocator',
            name='gnfc_goal_allocator',
            output='screen',
            parameters=[{
                'goal_zone_x_min': x_min,
                'goal_zone_x_max': x_max,
                'goal_zone_y_min': y_min,
                'goal_zone_y_max': y_max,
                'goal_grid_spacing': spacing,
                'use_sim_time': True,
            }],
        ),

        # Goal Zone Visualizer (rectangle in RViz)
        Node(
            package='mapf_coordinator',
            executable='goal_zone_visualizer',
            name='goal_zone_visualizer',
            output='screen',
            parameters=[{
                'goal_zone_x_min': x_min,
                'goal_zone_x_max': x_max,
                'goal_zone_y_min': y_min,
                'goal_zone_y_max': y_max,
                'use_sim_time': True,
            }],
        ),
    ])
