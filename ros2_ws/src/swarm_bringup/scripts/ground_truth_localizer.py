#!/usr/bin/env python3
"""
Ground-truth localisation
=========================

Publishes the ``map -> {robot}/odom`` transform for every robot, taking the
place that AMCL would occupy in a stack with real sensing.

Why this node exists
--------------------
The scenario stack runs no localisation at all.  Before this node, ``map ->
{robot}/odom`` was a *static* transform pinned to the spawn pose, while
``{robot}/odom -> {robot}/base_link`` came from Gazebo's DiffDrive plugin,
which integrates wheel velocities.  Nothing ever corrected that integration,
so the pose every consumer read from TF was uncorrected dead reckoning:

  * ``pure_pursuit`` steered towards its lookahead using a drifted pose;
  * ``_should_yield`` compared two drifted poses, so ``EMERGENCY_STOP_M`` and
    ``YIELD_DISTANCE_M`` fired at the wrong moments;
  * ``whca_star._build_blocked_set`` located higher-priority robots on their
    reserved paths using a drifted pose;
  * RViz drew where the robots *believed* they were, which is why it diverged
    visibly from Gazebo.

Correcting map->odom (rather than publishing map->base_link directly) keeps the
standard REP-105 tree intact: ``base_link`` still has exactly one parent, and
``odom`` stays continuous, so nothing downstream needs to change.

Maths
-----
Given the simulator's ground truth ``T_map_base`` and the wheel-odometry
``T_odom_base`` read from TF::

    T_map_odom = T_map_base * inverse(T_odom_base)

which is exactly the correction AMCL publishes.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformBroadcaster, TransformException
from tf2_ros.transform_listener import TransformListener

ROBOT_NAMES = ['alvin', 'teodoro', 'simon']

# Rate at which the correction is republished (Hz).  Kept above the 20 Hz that
# the controller polls TF at, so a lookup never falls back on a stale entry.
PUBLISH_RATE_HZ = 50.0


def _quat_mul(a, b):
    """Hamilton product of two (x, y, z, w) quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_inv(q):
    """Inverse of a unit quaternion (x, y, z, w)."""
    x, y, z, w = q
    return (-x, -y, -z, w)


def _quat_rotate(q, v):
    """Rotate the vector v by the unit quaternion q."""
    qv = (v[0], v[1], v[2], 0.0)
    rx, ry, rz, _ = _quat_mul(_quat_mul(q, qv), _quat_inv(q))
    return (rx, ry, rz)


class GroundTruthLocalizerNode(Node):
    def __init__(self):
        super().__init__('ground_truth_localizer')

        self._truth = {}          # robot -> (position, quaternion) in map
        self._warned = set()      # robots whose missing TF was already logged

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Gazebo publishes at 50 Hz; BEST_EFFORT matches the bridge and avoids
        # queueing stale poses if this node ever falls behind.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        for name in ROBOT_NAMES:
            self.create_subscription(
                Odometry, f'/{name}/ground_truth',
                lambda msg, r=name: self._truth_cb(msg, r), qos)

        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_corrections)
        self.get_logger().info(
            'ground_truth_localizer ready — correcting map->odom for '
            + ', '.join(ROBOT_NAMES))

    def _truth_cb(self, msg: Odometry, robot: str):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._truth[robot] = ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    def _publish_corrections(self):
        now = self.get_clock().now()
        for robot, (p_mb, q_mb) in self._truth.items():
            try:
                t = self.tf_buffer.lookup_transform(
                    f'{robot}/odom', f'{robot}/base_link', rclpy.time.Time())
            except TransformException:
                # DiffDrive has not published odom->base_link yet.  Warn once
                # per robot instead of every tick.
                if robot not in self._warned:
                    self._warned.add(robot)
                    self.get_logger().warn(
                        f'{robot}: waiting for {robot}/odom -> '
                        f'{robot}/base_link before correcting map->odom')
                continue

            tr = t.transform.translation
            ro = t.transform.rotation
            p_ob = (tr.x, tr.y, tr.z)
            q_ob = (ro.x, ro.y, ro.z, ro.w)

            # T_map_odom = T_map_base * inverse(T_odom_base)
            q_mo = _quat_mul(q_mb, _quat_inv(q_ob))
            rotated = _quat_rotate(q_mo, p_ob)
            p_mo = (p_mb[0] - rotated[0],
                    p_mb[1] - rotated[1],
                    p_mb[2] - rotated[2])

            msg = TransformStamped()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = 'map'
            msg.child_frame_id = f'{robot}/odom'
            msg.transform.translation.x = p_mo[0]
            msg.transform.translation.y = p_mo[1]
            msg.transform.translation.z = p_mo[2]
            msg.transform.rotation.x = q_mo[0]
            msg.transform.rotation.y = q_mo[1]
            msg.transform.rotation.z = q_mo[2]
            msg.transform.rotation.w = q_mo[3]
            self.tf_broadcaster.sendTransform(msg)

    def drift_of(self, robot):
        """Current dead-reckoning error of one robot, in metres.

        Not used by the control loop — exposed so the experiment logger can
        report how far wheel odometry had drifted, which is the quantity H4
        cares about when moving to real hardware.
        """
        if robot not in self._truth:
            return None
        try:
            t = self.tf_buffer.lookup_transform(
                'map', f'{robot}/base_link', rclpy.time.Time())
        except TransformException:
            return None
        (px, py, _), _ = self._truth[robot]
        return math.hypot(t.transform.translation.x - px,
                          t.transform.translation.y - py)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthLocalizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
