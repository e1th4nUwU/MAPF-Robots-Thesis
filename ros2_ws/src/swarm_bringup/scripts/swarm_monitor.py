#!/usr/bin/env python3
"""
Swarm Monitor Node
==================
Subscribes to /alvin/odom, /teodoro/odom, /simon/odom and prints
the (x, y, theta) of each robot at 1 Hz.

USAGE:
  ros2 run swarm_bringup swarm_monitor.py
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


ROBOT_NAMES = ['alvin', 'teodoro', 'simon']


def quaternion_to_yaw(q):
    """Extract yaw (rotation around Z axis) from a quaternion.

    A quaternion (x, y, z, w) encodes 3D rotation. For a robot moving
    on a flat floor, only the Z-rotation (yaw) matters. This formula
    extracts it without needing a full quaternion library.
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class SwarmMonitor(Node):
    def __init__(self):
        super().__init__('swarm_monitor')

        # Store latest position for each robot.
        # None means we haven't received data yet.
        self.positions = {name: None for name in ROBOT_NAMES}

        # Create one subscriber per robot.
        # QoS 10 = keep last 10 messages in the buffer (standard for odom).
        for name in ROBOT_NAMES:
            # Use a lambda to capture the robot name in the callback.
            # Structure: /{robot_name}/odom
            self.create_subscription(
                Odometry,
                f'/{name}/odom',
                lambda msg, n=name: self._odom_callback(msg, n),
                10,
            )
            self.get_logger().info(f'Subscribed to /{name}/odom')

        # Timer: print positions every 1 second.
        # Without a timer, we'd print on every odom message (100 Hz = spam).
        self.create_timer(1.0, self._print_positions)

    def _odom_callback(self, msg: Odometry, robot_name: str):
        """Called every time an Odometry message arrives for a robot."""
        # Extract (x, y) position and yaw angle from the message and store it.
        pos = msg.pose.pose.position
        # Yaw is the rotation around the Z axis, which we can get from the
        # orientation quaternion.
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        # Store the position as a tuple (x, y, yaw) in the dictionary.
        self.positions[robot_name] = (pos.x, pos.y, yaw)

    def _print_positions(self):
        """Print a summary table of all robot positions."""
        self.get_logger().info('--- Swarm Positions ---')
        for name in ROBOT_NAMES:
            p = self.positions[name]
            if p is None:
                self.get_logger().info(f'  {name}: no data yet')
            else:
                self.get_logger().info(
                    f'  {name}: x={p[0]:+.3f}  y={p[1]:+.3f}  '
                    f'yaw={math.degrees(p[2]):+.1f} deg'
                )


def main(args=None):
    # Initialize ROS 2 and create the node.
    rclpy.init(args=args)
    # The node will keep running until we shut it down (e.g. Ctrl+C).
    node = SwarmMonitor()
    try:
        # Spin the node to process callbacks (like odom messages and timers).
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
