#!/usr/bin/env python3
"""
Swarm Health Monitor Node
=========================
Monitors each robot's odometry and publishes a RobotHealth message
when a robot stops moving (i.e. has likely died/crashed).

A robot is considered DEAD when its position hasn't changed by more
than MOVEMENT_THRESHOLD metres for DEATH_TIMEOUT seconds.

Published topics:
  /{robot}/health  →  navig_msgs/RobotHealth

USAGE:
  ros2 run swarm_bringup swarm_health_monitor
"""

import math
import rclpy
import rclpy.duration
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from navig_msgs.msg import RobotHealth
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


ROBOT_NAMES = ['alvin', 'teodoro', 'simon']

# A position delta smaller than this (metres) counts as "not moving".
MOVEMENT_THRESHOLD = 0.05

# Seconds without movement before a robot is declared dead.
DEATH_TIMEOUT = 3.0


class RobotState:
    """Holds the current health state for one robot."""

    def __init__(self, now):
        self.position = Point(x=0.0, y=0.0, z=0.0)  # default until first odom
        self.last_moved_time = now    # rclpy.Time when the robot last moved
        self.is_alive = True          # current declared state
        self.got_first_odom = False   # True once we've received at least one odom
        self.has_ever_moved = False   # True once robot moves >= MOVEMENT_THRESHOLD
                                      # (prevents false-positive death at spawn)
        self.is_killed = False        # True after GUI kill — blocks odom from resetting timer
        self.at_goal = False          # True after goal_reached — suppresses death check


class SwarmHealthMonitor(Node):
    def __init__(self):
        super().__init__('swarm_health_monitor')

        now = self.get_clock().now()
        self.states = {name: RobotState(now) for name in ROBOT_NAMES}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # One publisher per robot.
        self.health_pubs = {}
        for name in ROBOT_NAMES:
            self.health_pubs[name] = self.create_publisher(
                RobotHealth,
                f'/{name}/health',
                10,
            )

        # One subscriber per robot.
        for name in ROBOT_NAMES:
            self.create_subscription(
                Odometry,
                f'/{name}/odom',
                lambda msg, n=name: self._odom_callback(msg, n),
                10,
            )
            # When a robot reaches its goal, reset its movement state so it
            # doesn't get declared dead just for being stationary at the goal.
            self.create_subscription(
                Bool,
                f'/{name}/navigation/goal_reached',
                lambda msg, n=name: self._goal_reached_callback(msg, n),
                10,
            )
            # Manual kill signal from the GUI — mark dead immediately.
            self.create_subscription(
                Bool,
                f'/{name}/kill',
                lambda msg, n=name: self._kill_callback(msg, n),
                10,
            )
            self.get_logger().info(f'Subscribed to /{name}/odom')

        # Evaluate health at 2 Hz (every 0.5 s).
        self.create_timer(0.5, self._check_health)

        self.get_logger().info(
            f'Health monitor started. '
            f'Timeout={DEATH_TIMEOUT}s, threshold={MOVEMENT_THRESHOLD}m'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _odom_callback(self, msg: Odometry, robot_name: str):
        """Update the robot's last known position and movement timestamp."""
        state = self.states[robot_name]
        new_pos = msg.pose.pose.position
        now = self.get_clock().now()

        if not state.got_first_odom:
            # First message — seed position and reset the movement clock so
            # the robot doesn't immediately look dead from the default position.
            state.position = new_pos
            state.last_moved_time = now
            state.got_first_odom = True
            return

        # If killed, don't let odom jitter reset the death timer.
        if state.is_killed:
            return

        # Euclidean distance from last stored position.
        dx = new_pos.x - state.position.x
        dy = new_pos.y - state.position.y
        delta = math.sqrt(dx * dx + dy * dy)

        if delta >= MOVEMENT_THRESHOLD:
            # Robot moved enough — update position and reset the timer.
            state.position = new_pos
            state.last_moved_time = now
            state.has_ever_moved = True
            state.at_goal = False   # moving again means left goal area

    def _kill_callback(self, msg: Bool, robot_name: str):
        """GUI kill button — mark the robot dead instantly without waiting for timeout."""
        if msg.data:
            state = self.states[robot_name]
            state.is_alive = False
            state.is_killed = True        # blocks odom from resetting the timer
            state.at_goal = False
            state.has_ever_moved = True   # ensure death check is active
            # Force elapsed time past the timeout by backdating last_moved_time
            now = self.get_clock().now()
            state.last_moved_time = now - rclpy.duration.Duration(seconds=DEATH_TIMEOUT + 1)
            self.get_logger().warn(
                f'[HEALTH] {robot_name} KILLED via GUI at '
                f'({state.position.x:.2f}, {state.position.y:.2f})'
            )
            # Publish immediately — do not wait for the 0.5s timer tick
            self._publish_health(robot_name, state, now)

    def _goal_reached_callback(self, msg: Bool, robot_name: str):
        """Robot reached its goal — suppress death check while stationary at goal."""
        if msg.data:
            state = self.states[robot_name]
            if state.is_killed:
                # Kill takes priority — ignore stale goal_reached published just before death
                return
            state.at_goal = True
            state.is_alive = True
            state.last_moved_time = self.get_clock().now()
            self.get_logger().info(f'[HEALTH] {robot_name} reached goal — death check suppressed')

    def _check_health(self):
        """Declare robots alive/dead and publish RobotHealth messages."""
        now = self.get_clock().now()

        for name in ROBOT_NAMES:
            state = self.states[name]

            elapsed = (now - state.last_moved_time).nanoseconds / 1e9

            # Skip death check in these cases:
            # - haven't received odom yet
            # - never moved since spawn (waiting for first goal)
            # - at goal (stationary by design, not dead)
            if not state.got_first_odom or not state.has_ever_moved or state.at_goal:
                self._publish_health(name, state, now)
                continue
            was_alive = state.is_alive
            state.is_alive = elapsed < DEATH_TIMEOUT

            # Log state transitions so it's easy to spot in the terminal.
            if was_alive and not state.is_alive:
                self.get_logger().warn(
                    f'[HEALTH] {name} DIED at '
                    f'({state.position.x:.2f}, {state.position.y:.2f}) '
                    f'— no movement for {elapsed:.1f}s'
                )
            elif not was_alive and state.is_alive:
                self.get_logger().info(f'[HEALTH] {name} is ALIVE again')

            self._publish_health(name, state, now)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _publish_health(self, robot_name: str, state: RobotState, now):
        msg = RobotHealth()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'map'
        msg.robot_name = robot_name
        msg.is_alive = state.is_alive

        # Get position in map frame via TF so cost_map can paint the obstacle
        # correctly. Falls back to the stored odom-frame position on TF failure.
        try:
            t = self.tf_buffer.lookup_transform(
                'map', f'{robot_name}/base_link', rclpy.time.Time())
            msg.position = Point(
                x=t.transform.translation.x,
                y=t.transform.translation.y,
                z=0.0)
        except TransformException:
            msg.position = Point(x=state.position.x, y=state.position.y, z=0.0)

        self.health_pubs[robot_name].publish(msg)


# ----------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = SwarmHealthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
