"""
Goal Zone Visualizer Only — for debugging and RViz visualization without goal assignment.

Usage:
  ros2 launch mapf_coordinator visualizer_only.launch.py \
    x_min:=7.5 x_max:=9.0 y_min:=-1.0 y_max:=1.0
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

    return LaunchDescription([
        DeclareLaunchArgument('x_min', default_value='7.5', description='Zone min X (m)'),
        DeclareLaunchArgument('x_max', default_value='9.0', description='Zone max X (m)'),
        DeclareLaunchArgument('y_min', default_value='-1.0', description='Zone min Y (m)'),
        DeclareLaunchArgument('y_max', default_value='1.0', description='Zone max Y (m)'),

        # Goal Zone Visualizer ONLY
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
