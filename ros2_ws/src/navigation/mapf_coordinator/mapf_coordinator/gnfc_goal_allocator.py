#!/usr/bin/env python3
"""
Greedy Nearest-Free-Cell (GNFC) Goal Allocator for Multi-Robot Systems

Assigns unique goal positions within a goal zone to each robot.
Simple O(n·m) algorithm: each robot gets the nearest unassigned goal point.

Each robot's /odom topic is published in its own local frame (origin at spawn).
GNFC therefore needs the spawn position of each robot to convert odom poses
to the shared map frame; otherwise the first allocation tick sees every robot
at (0, 0) and assignments are non-deterministic.

Usage:
  ros2 run mapf_coordinator gnfc_goal_allocator
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Point
import numpy as np

# Latched QoS: publisher keeps the last message so late-joining subscribers
# (pure_pursuit nodes not yet discovered when GNFC first publishes) still
# receive their goal rather than missing it silently.
_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

ROBOT_NAMES = ['alvin', 'teodoro', 'simon']

# Default goal zone (can be overridden via parameters)
DEFAULT_GOAL_ZONE = {
    'x_min': 7.5,
    'x_max': 9.0,
    'y_min': -1.0,
    'y_max': 1.0,
}

# Grid spacing (metres) — must be >= robot circumscribed diameter (2 × 0.30 m = 0.60 m)
# so adjacent goals never require robots to physically overlap.
# 0.8 m gives 0.20 m of clearance between robot bodies at adjacent goals.
# Must clear PARKED_ROBOT_RADIUS_M (0.70 m) plus the 0.20 m parking tolerance,
# or a robot that stops short paints over its neighbour's goal.
DEFAULT_GOAL_GRID_SPACING = 1.0


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

        # Spawn positions in the map frame — declared per robot so the launch
        # file can hand them in. Without these, all robots appear at (0, 0) at
        # startup (odom origin) and the first allocation is a tie/random.
        for name in ROBOT_NAMES:
            self.declare_parameter(f'initial_x_{name}', 0.0)
            self.declare_parameter(f'initial_y_{name}', 0.0)

        # Read parameters
        self.goal_zone = {
            'x_min': self.get_parameter('goal_zone_x_min').value,
            'x_max': self.get_parameter('goal_zone_x_max').value,
            'y_min': self.get_parameter('goal_zone_y_min').value,
            'y_max': self.get_parameter('goal_zone_y_max').value,
        }
        self.grid_spacing = self.get_parameter('goal_grid_spacing').value
        publish_rate_hz = self.get_parameter('publish_rate_hz').value

        # Per-robot spawn offset (map frame). Used to convert each robot's
        # local-frame odom message to a map-frame position.
        self.spawn_positions = {
            name: np.array([
                self.get_parameter(f'initial_x_{name}').value,
                self.get_parameter(f'initial_y_{name}').value,
            ])
            for name in ROBOT_NAMES
        }
        for name, pos in self.spawn_positions.items():
            self.get_logger().info(
                f'Spawn {name}: ({pos[0]:.2f}, {pos[1]:.2f}) [map frame]'
            )

        # Generate candidate goal points in goal zone
        self.candidate_goals = self._generate_goal_candidates()
        self.get_logger().info(
            f"Generated {len(self.candidate_goals)} candidate goal points in zone "
            f"[{self.goal_zone['x_min']:.1f}, {self.goal_zone['x_max']:.1f}] × "
            f"[{self.goal_zone['y_min']:.1f}, {self.goal_zone['y_max']:.1f}]"
        )

        # Current robot positions in map frame — seeded with spawn so the
        # first allocation tick (which can fire before /odom arrives) has
        # deterministic, correct inputs.
        self.robot_positions = {
            name: self.spawn_positions[name].copy() for name in ROBOT_NAMES
        }

        # Assigned goals (one per robot) — locked until robot reaches goal
        self.assigned_goals = {}
        # Robots that need a new goal (initially all of them)
        self.needs_goal = set(ROBOT_NAMES)
        # Robots that have arrived at a goal-zone position — no further reassignment
        self.parked = set()

        # Publishers for each robot's goal — TRANSIENT_LOCAL so late subscribers
        # (pure_pursuit connecting after GNFC first publishes) get the cached goal.
        self.goal_publishers = {
            name: self.create_publisher(PoseStamped, f'/{name}/goal_pose', _LATCHED_QOS)
            for name in ROBOT_NAMES
        }

        # Subscribe to each robot's odometry to track position
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Bool
        for name in ROBOT_NAMES:
            self.create_subscription(
                Odometry,
                f'/{name}/odom',
                lambda msg, n=name: self._odometry_callback(msg, n),
                10
            )
            self.create_subscription(
                Bool,
                f'/{name}/navigation/goal_reached',
                lambda msg, n=name: self._goal_reached_cb(msg, n),
                10
            )

        # Timer: check if any robot needs a new goal
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
        """Update robot position in map frame = spawn + odom displacement."""
        local = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        ])
        self.robot_positions[robot_name] = self.spawn_positions[robot_name] + local

    def _goal_reached_cb(self, msg, robot_name: str):
        """When a robot reaches its goal, park it (no reassignment)."""
        if msg.data and robot_name not in self.parked:
            self.get_logger().info(f'{robot_name} reached goal zone — parking')
            self.parked.add(robot_name)
            self.needs_goal.discard(robot_name)
            # Keep assigned_goals entry so the spot stays reserved for this robot

    def _allocate_and_publish(self):
        """Only assign goals to robots that need one and aren't parked."""
        pending = self.needs_goal - self.parked
        if not pending:
            return

        # Candidates already taken by robots that still have active goals
        taken = list(self.assigned_goals.values())
        remaining = [g for g in self.candidate_goals
                     if not any(np.linalg.norm(g - t) < 0.01 for t in taken)]

        robots_sorted = sorted(
            pending,
            key=lambda name: min(
                (np.linalg.norm(self.robot_positions[name] - g) for g in remaining),
                default=float('inf')
            )
        )

        for robot_name in robots_sorted:
            if not remaining:
                self.get_logger().warn(f"No candidates left for {robot_name}!")
                break

            robot_pos = self.robot_positions[robot_name]
            distances = [np.linalg.norm(robot_pos - g) for g in remaining]
            nearest_idx = int(np.argmin(distances))
            goal = remaining.pop(nearest_idx)
            self.assigned_goals[robot_name] = goal

            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = float(goal[0])
            msg.pose.position.y = float(goal[1])
            msg.pose.position.z = 0.0
            self.goal_publishers[robot_name].publish(msg)
            self.get_logger().info(
                f'Assigned {robot_name} → ({goal[0]:.2f}, {goal[1]:.2f})'
            )

        self.needs_goal.clear()


def main(args=None):
    rclpy.init(args=args)
    allocator = GNFCGoalAllocatorNode()
    rclpy.spin(allocator)
    allocator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
