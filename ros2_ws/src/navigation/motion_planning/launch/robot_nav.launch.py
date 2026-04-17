"""Per-robot navigation stack: a_star + path_smoothing + pure_pursuit.

All nodes run inside the robot's namespace so their topics and services become:
  /{robot_name}/path_planning/plan_path
  /{robot_name}/path_planning/smooth_path
  /{robot_name}/cmd_vel   (bridges to Gazebo)
  /{robot_name}/goal_pose (send goals here)

The cost_map and map_server are shared globally — launch them separately.

Usage (standalone, one robot):
  ros2 launch motion_planning robot_nav.launch.py robot_name:=alvin
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    """
    Launch the navigation stack for a single robot.
    The robot name is passed as a launch argument and used as a namespace for all nodes, topics and services.
    """
    
    robot_name = LaunchConfiguration('robot_name')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            description='Robot namespace (e.g. alvin, teodoro, simon)',
        ),
        DeclareLaunchArgument(
            'diagonals',
            default_value='True',
            description='Allow diagonal moves in A*',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
        ),
        DeclareLaunchArgument(
            'planner',
            default_value='independent',
            description='MAPF mode: independent | whca | cbs',
        ),


        GroupAction([
            PushRosNamespace(robot_name),

            Node(
                package='path_planner',
                executable='a_star',
                name='a_star',
                output='screen',
                condition=IfCondition(PythonExpression(
                    ["'", LaunchConfiguration('planner'), "' == 'independent'"]
                )),
                parameters=[{
                    'diagonals': LaunchConfiguration('diagonals'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }],
            ),

            Node(
                package='path_planner',
                executable='path_smoothing',
                name='path_smoothing',
                output='screen',
                condition=IfCondition(PythonExpression(
                    ["'", LaunchConfiguration('planner'), "' != 'cbs'"]
                )),
                parameters=[{
                    'w1': 0.95,
                    'w2': 0.05,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }],
            ),

            Node(
                package='path_follower',
                executable='pure_pursuit',
                name='pure_pursuit',
                output='screen',
                condition=IfCondition(PythonExpression(
                    ["'", LaunchConfiguration('planner'), "' != 'cbs'"]
                )),
                parameters=[{
                    'alpha': 0.1,
                    'beta': 0.1,
                    'tol': 0.1,  # Tighter goal tolerance to force navigation
                    # No use_sim_time: rclpy.time.Time() uses SYSTEM_TIME by default,
                    # which conflicts with the ROS_TIME buffer when sim_time is on.
                    'robot_frame': [robot_name, '/base_link'],
                }],
            ),
        ]),
    ])
