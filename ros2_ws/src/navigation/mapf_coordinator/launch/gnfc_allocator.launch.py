"""
GNFC Goal Allocator with configurable goal zone.

Usage:
  ros2 launch mapf_coordinator gnfc_allocator.launch.py \
    goal_zone_x_min:=7.5 goal_zone_x_max:=9.0 \
    goal_zone_y_min:=-1.0 goal_zone_y_max:=1.0 \
    goal_grid_spacing:=0.3
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
        DeclareLaunchArgument(
            'goal_grid_spacing',
            default_value='0.3',
            description='Spacing between candidate goal points (metres)'
        ),
        Node(
            package='mapf_coordinator',
            executable='gnfc_goal_allocator',
            name='gnfc_goal_allocator',
            output='screen',
            parameters=[{
                'goal_zone_x_min': LaunchConfiguration('goal_zone_x_min'),
                'goal_zone_x_max': LaunchConfiguration('goal_zone_x_max'),
                'goal_zone_y_min': LaunchConfiguration('goal_zone_y_min'),
                'goal_zone_y_max': LaunchConfiguration('goal_zone_y_max'),
                'goal_grid_spacing': LaunchConfiguration('goal_grid_spacing'),
                'use_sim_time': True,
            }],
        ),
    ])
