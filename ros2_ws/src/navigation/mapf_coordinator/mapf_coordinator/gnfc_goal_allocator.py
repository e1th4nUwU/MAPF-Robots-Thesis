#!/usr/bin/env python3
"""
Greedy Nearest-Free-Cell (GNFC) Goal Allocator for Multi-Robot Systems

Assigns unique goal positions within a goal zone to each robot.
Simple O(n·m) algorithm: each robot gets the nearest unassigned goal point.

Usage:
  ros2 run mapf_coordinator gnfc_goal_allocator
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
import numpy as np

ROBOT_NAMES = ['alvin', 'teodoro', 'simon']

# Default goal zone (can be overridden via parameters)
DEFAULT_GOAL_ZONE = {
    'x_min': 7.5,
    'x_max': 9.0,
    'y_min': -1.0,
    'y_max': 1.0,
}

# Grid spacing (metres) — determines candidate goal points
DEFAULT_GOAL_GRID_SPACING = 0.3


class GNFCGoalAllocatorNode(Node):
    def __init__(self):
        super().__init__('gnfc_goal_allocator')
        self.get_logger().info("INITIALIZING GNFC GOAL ALLOCATOR")

        # Declare parameters with defaults
        self.declare_parameter('goal_zone_x_min', DEFAULT_GOAL_ZONE['x_min'])
        self.declare_parameter('goal_zone_x_max', DEFAULT_GOAL_ZONE['x_max'])
        self.declare_parameter('goal_zone_y_min', DEFAULT_GOAL_ZONE['y_min'])
        self.declare_parameter('goal_zone_y_max', DEFAULT_GOAL_ZONE['y_max'])
        self.declare_parameter('goal_grid_spacing', DEFAULT_GOAL_GRID_SPACING)
        self.declare_parameter('publish_rate_hz', 1.0)

        # Read parameters
        self.goal_zone = {
            'x_min': self.get_parameter('goal_zone_x_min').value,
            'x_max': self.get_parameter('goal_zone_x_max').value,
            'y_min': self.get_parameter('goal_zone_y_min').value,
            'y_max': self.get_parameter('goal_zone_y_max').value,
        }
        self.grid_spacing = self.get_parameter('goal_grid_spacing').value
        publish_rate_hz = self.get_parameter('publish_rate_hz').value

        # Generate candidate goal points in goal zone
        self.candidate_goals = self._generate_goal_candidates()
        self.get_logger().info(
            f"Generated {len(self.candidate_goals)} candidate goal points in zone "
            f"[{self.goal_zone['x_min']:.1f}, {self.goal_zone['x_max']:.1f}] × "
            f"[{self.goal_zone['y_min']:.1f}, {self.goal_zone['y_max']:.1f}]"
        )

        # Current robot positions (updated via subscriptions)
        self.robot_positions = {name: np.array([0.0, 0.0]) for name in ROBOT_NAMES}

        # Assigned goals (one per robot)
        self.assigned_goals = {}

        # Publishers for each robot's goal
        self.goal_publishers = {
            name: self.create_publisher(PoseStamped, f'/{name}/goal_pose', 1)
            for name in ROBOT_NAMES
        }

        # Subscribe to each robot's odometry to track position
        from nav_msgs.msg import Odometry
        for name in ROBOT_NAMES:
            self.create_subscription(
                Odometry,
                f'/{name}/odom',
                lambda msg, n=name: self._odometry_callback(msg, n),
                10
            )

        # Timer: allocate goals and publish at regular interval
        self.create_timer(1.0 / publish_rate_hz, self._allocate_and_publish)

        self.get_logger().info("GNFC Goal Allocator ready")

    def _generate_goal_candidates(self):
        """Generate grid of candidate goal points within goal zone."""
        x_vals = np.arange(
            self.goal_zone['x_min'],
            self.goal_zone['x_max'] + self.grid_spacing,
            self.grid_spacing
        )
        y_vals = np.arange(
            self.goal_zone['y_min'],
            self.goal_zone['y_max'] + self.grid_spacing,
            self.grid_spacing
        )
        candidates = [
            np.array([x, y]) for x in x_vals for y in y_vals
        ]
        return candidates

    def _odometry_callback(self, msg, robot_name: str):
        """Update robot position from odometry (nav_msgs/Odometry)."""
        pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])
        self.robot_positions[robot_name] = pos

    def _allocate_goals(self):
        """
        Greedy Nearest-Free-Cell allocation:
        Each robot gets the nearest unassigned goal point.

        Returns:
            dict: {robot_name: goal_position (numpy array)}
        """
        assigned = {}
        remaining_candidates = self.candidate_goals.copy()

        # Sort robots by distance to goal zone (closer robots first)
        # This gives preference to robots already near the goal area
        robots_sorted = sorted(
            ROBOT_NAMES,
            key=lambda name: min(
                np.linalg.norm(self.robot_positions[name] - goal)
                for goal in remaining_candidates
            )
        )

        for robot_name in robots_sorted:
            if not remaining_candidates:
                self.get_logger().warn(f"No candidates left for {robot_name}!")
                break

            # Find nearest unassigned goal
            robot_pos = self.robot_positions[robot_name]
            distances = [
                np.linalg.norm(robot_pos - goal) for goal in remaining_candidates
            ]
            nearest_idx = np.argmin(distances)
            nearest_goal = remaining_candidates[nearest_idx]

            assigned[robot_name] = nearest_goal
            remaining_candidates.pop(nearest_idx)

            self.get_logger().debug(
                f"Assigned {robot_name} goal ({nearest_goal[0]:.2f}, {nearest_goal[1]:.2f}) "
                f"at distance {distances[nearest_idx]:.2f}m"
            )

        return assigned

    def _allocate_and_publish(self):
        """Allocate goals and publish them."""
        self.assigned_goals = self._allocate_goals()

        # Publish each goal
        for robot_name, goal_pos in self.assigned_goals.items():
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = float(goal_pos[0])
            msg.pose.position.y = float(goal_pos[1])
            msg.pose.position.z = 0.0

            self.goal_publishers[robot_name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    allocator = GNFCGoalAllocatorNode()
    rclpy.spin(allocator)
    allocator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
