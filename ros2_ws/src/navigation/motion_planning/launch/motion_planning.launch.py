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
            parameters=[{'yaml_filename':map_config_file}, {'use_sim_time':True}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_fallback_tf',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': True}]
        ),
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
        Node(
            package="path_planner",
            executable="cost_map",
            name='cost_map',
            output='screen',
            parameters=[{'inflation_radius':0.2}, {'cost_radius':0.5}]
        ),
        Node(
            package="path_planner",
            executable="a_star",
            name='a_star',
            output='screen',
            parameters=[{'diagonals':True}]
        ),
        Node(
            package="path_planner",
            executable="path_smoothing",
            name='path_smoothing',
            output='screen',
            parameters=[{'w1':0.95}, {'w2':0.05}]
        ),
        Node(
            package="path_follower",
            executable="pure_pursuit",
            name='pure_pursuit',
            output='screen',
            parameters=[{'alpha':0.1}, {'beta':0.1}]
        ),
    ])
