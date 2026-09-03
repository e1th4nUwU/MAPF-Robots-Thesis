#
# MOBILE ROBOTS - FI-UNAM, 2026-2
# PATH FOLLOWING BY PURE PURSUIT
#
# Instructions:
# Write the code necessary to move the robot along a given path.
# Consider a differential base. Max linear and angular speeds
# must be 0.8 and 1.0 respectively.
#

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, String
from navig_msgs.msg import RobotHealth
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from navig_msgs.srv import ProcessPath
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from ament_index_python.packages import get_package_share_directory
import math
import time
import numpy

NAME = "Jorge Eithan Treviño Selles"

ROBOT_NAMES = ['alvin', 'teodoro', 'simon']

# Yield distance (m): if a higher-priority robot is closer than this AND
# in front of us (ahead along our heading), this robot stops to let it pass.
# 1.20 m = 2 robot radii (0.60 m) + 0.60 m reaction buffer at v_max=0.5 m/s
# (≈1.2 s of braking distance).  Larger than before (0.80 m) because the
# old value let robots get inside the WHCA* lookahead window before yielding,
# producing rear-end collisions in tight corridors.
YIELD_DISTANCE_M = 1.20

# Half-angle (rad) of the "front cone" used by _should_yield.  A higher-
# priority robot only triggers a yield if it lies within ±FRONT_CONE_RAD of
# our heading.  Without this, a higher-priority robot trailing behind would
# make us stop forever (it never overtakes — we never restart).
# 1.57 rad ≈ 90°, i.e. anything in our forward hemisphere counts as "ahead".
FRONT_CONE_RAD = 1.57

# Emergency stop (m): stop for ANY robot regardless of priority when this
# close. Prevents the highest-priority robot from driving into a stopped or
# slow lower-priority robot at a chokepoint. 0.65 m = contact distance
# (2 × 0.30 m radius) + 0.05 m margin.
EMERGENCY_STOP_M = 0.65

# How long a robot left without a coordinated route waits before trying again,
# instead of sitting until the coordinator's next tick.  The lowest-priority
# robot only gets a tick every replan_period_sec + stagger_sec * (k-1) ~= 5 s,
# so a corridor that clears right after a failure costs it the whole interval.
# Planning the maze costs ~220 ms measured, and a robot with no route publishes
# an empty reservation, so retrying early disturbs nobody and is nearly free.
EMPTY_PATH_RETRY_SEC = 1.0

SM_INIT = 0
SM_WAIT_FOR_NEW_GOAL = 10
SM_PLAN_PATH = 20
SM_SMOOTH_PATH = 30
SM_FOLLOWING_PATH = 40
SM_SAVE_DATA = 50

class PurePursuitNode(Node):
    def calculate_control(self, robot_x, robot_y, robot_a, goal_x, goal_y, alpha, beta, v_max, w_max):
        error_a = math.atan2(goal_y - robot_y, goal_x - robot_x) - robot_a
        # Normalize to (-pi, pi]
        while error_a >  math.pi: error_a -= 2*math.pi
        while error_a <= -math.pi: error_a += 2*math.pi
        v = v_max * math.exp(-error_a * error_a / alpha)
        w = w_max * (2.0 / (1.0 + math.exp(-error_a / beta)) - 1.0)
        return [v, w]

    def pure_pursuit(self, path, alpha, beta, v_max, w_max, tol):
        """
        Follow the given path using the pure pursuit control law with a lookahead point.

        Instead of chasing waypoints one by one, the robot always steers toward the
        point on the path that is exactly LOOKAHEAD_DIST metres ahead of its current
        position. This produces smooth, stable tracking without oscillation.

        Args:
            path: List of numpy arrays [x, y] — dense waypoints from A*
            alpha: Control parameter for linear velocity
            beta:  Control parameter for angular velocity
            v_max: Maximum linear velocity  (m/s)
            w_max: Maximum angular velocity (rad/s)
            tol:   Distance to final goal considered "reached" (m)
        """
        if not path:
            return

        LOOKAHEAD_DIST = 0.25  # metres ahead to target on the path

        # Index of the furthest path point we have "passed" — the lookahead
        # search starts from here so it always moves forward along the path.
        anchor_idx = 0

        while rclpy.ok() and self.robot_alive and not self.needs_replan:
            robot_p, robot_a = self.get_robot_pose()
            robot_x, robot_y = robot_p[0], robot_p[1]

            # ── Stop if final goal is close enough ──────────────────────────
            if numpy.linalg.norm(robot_p - path[-1]) < tol:
                break

            # ── Reactive yield: if a higher-priority robot is within
            # YIELD_DISTANCE_M, stop and let it pass.  This handles cases
            # where WHCA*'s spatial plans converge briefly at chokepoints.
            if self._should_yield(robot_p, robot_a):
                self._set_waiting(True)
                self.publish_and_save_data(robot_x, robot_y, robot_a,
                                           robot_x, robot_y, 0.0, 0.0)
                continue
            self._set_waiting(False)

            # ── Advance anchor to the closest path point to the robot ───────
            # This prevents the lookahead from ever going backwards.
            best_dist = float('inf')
            for i in range(anchor_idx, len(path)):
                d = numpy.linalg.norm(robot_p - path[i])
                if d < best_dist:
                    best_dist = d
                    anchor_idx = i

            # ── Find lookahead point: first path point >= LOOKAHEAD_DIST ────
            lookahead = path[-1]   # default: final goal
            for i in range(anchor_idx, len(path)):
                if numpy.linalg.norm(robot_p - path[i]) >= LOOKAHEAD_DIST:
                    lookahead = path[i]
                    break

            goal_x, goal_y = lookahead[0], lookahead[1]

            # ── Compute and publish control ──────────────────────────────────
            v, w = self.calculate_control(robot_x, robot_y, robot_a,
                                          goal_x, goal_y, alpha, beta, v_max, w_max)
            self.publish_and_save_data(robot_x, robot_y, robot_a, goal_x, goal_y, v, w)

    def _set_waiting(self, waiting: bool):
        """Publish the deliberate-stop flag, only when it changes."""
        if waiting == self._waiting:
            return
        self._waiting = waiting
        self.pub_waiting.publish(Bool(data=waiting))
        self.get_logger().info(
            'Waiting on purpose — death check suppressed' if waiting
            else 'Moving again — death check re-enabled')

    def publish_and_save_data(self, robot_x, robot_y, robot_a, goal_x, goal_y, v,w):
        self.nav_data.append([robot_x, robot_y, robot_a, goal_x, goal_y, v, w])
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub_cmd_vel.publish(msg)
        rclpy.spin_once(self)
        self.get_clock().sleep_for(Duration(seconds=0.005))

    def get_robot_pose(self):
        try:
            robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
            t = self.tf_buffer.lookup_transform("map", robot_frame, rclpy.time.Time())
            robot_x = t.transform.translation.x
            robot_y = t.transform.translation.y
            robot_pose = numpy.asarray([robot_x, robot_y])
            robot_a = math.atan2(t.transform.rotation.z, t.transform.rotation.w)*2
            self.robot_pose = robot_pose
            self.robot_a = robot_a
        except TransformException as ex:
            self.get_logger().info("Could not get robot pose")
            robot_pose = numpy.asarray([0.0,0.0])
            robot_a = 0.0
        return robot_pose, robot_a

    def _should_yield(self, robot_p, robot_a):
        """True when this robot should stop to avoid collision or yield priority.

        Three-layer check (uses TF positions in MAP frame for cross-robot
        comparison):
        - EMERGENCY_STOP_M  : stop for ANY robot regardless of priority/heading.
        - YIELD_DISTANCE_M  : proactive stop for higher-priority robots that are
                              *ahead* of us (within FRONT_CONE_RAD of heading).
                              Without the heading filter, a trailing higher-
                              priority robot would lock us in place forever.
        """
        my_idx = (self._priorities.index(self._robot_name)
                  if self._robot_name in self._priorities else 999)
        for r in ROBOT_NAMES:
            if r == self._robot_name:
                continue
            try:
                t = self.tf_buffer.lookup_transform(
                    'map', f'{r}/base_link', rclpy.time.Time())
                pos = numpy.asarray([t.transform.translation.x,
                                      t.transform.translation.y])
            except TransformException:
                continue
            delta = pos - robot_p
            dist = numpy.linalg.norm(delta)
            if dist < EMERGENCY_STOP_M:
                return True  # too close to any robot — stop regardless of priority
            if r in self._finished:
                # Parked on its goal: the emergency stop above still protects
                # against driving into it, but there is nothing to be gained by
                # yielding right of way to a robot that will never move.
                continue
            r_idx = (self._priorities.index(r)
                     if r in self._priorities else 999)
            if r_idx >= my_idx or dist >= YIELD_DISTANCE_M:
                continue
            # Higher priority + within yield range — only yield if the other
            # robot is ahead of us (would be in our way if we keep moving).
            if dist < 1e-6:
                return True
            bearing = math.atan2(delta[1], delta[0])
            rel_angle = bearing - robot_a
            while rel_angle >  math.pi: rel_angle -= 2 * math.pi
            while rel_angle <= -math.pi: rel_angle += 2 * math.pi
            if abs(rel_angle) < FRONT_CONE_RAD:
                return True  # higher-priority robot ahead of us — yield
        return False

    def _kill_callback(self, msg: Bool) -> None:
        if msg.data and self.robot_alive:
            self.robot_alive = False
            self.get_logger().warn("Robot killed — stopping navigation")

    def _replan_callback(self, msg: Bool) -> None:
        """Triggered by /replan_needed (robot death → cost map rebuilt → need to wait)."""
        if msg.data and self.robot_alive:
            self.needs_replan = True
            self.map_changed = True
            self.pub_cmd_vel.publish(Twist())
            self.get_logger().info("Map-change replan requested — will wait for rebuild")

    def _whca_replan_callback(self, msg: Bool) -> None:
        """Triggered by whca_replan (WHCA* sliding window — no map change, no wait needed)."""
        if msg.data and self.robot_alive:
            self.needs_replan = True
            self.pub_cmd_vel.publish(Twist())
            self.get_logger().info("WHCA* replan requested — replanning")

    def callback_goal_pose(self, msg):
        self.goal_pose = numpy.asarray([msg.pose.position.x, msg.pose.position.y])
        self.get_logger().info("Received new goal pose: " + str(self.goal_pose))
        self.new_goal_pose = True

    def __init__(self):
        super().__init__("pure_pursuit_node")
        self.get_logger().info("INITIALIZING PATH FOLLOWER NODE ...")
        self.nav_data = []
        self.data_file = get_package_share_directory('path_follower') + "/data.txt" 
        self.robot_pose = numpy.asarray([0.0,0.0])
        self.robot_a = 0.0
        self.new_goal_pose = False
        self.goal_pose = numpy.asarray([0.0,0.0])
        self.robot_alive = True
        self.needs_replan = False
        self.map_changed = False
        # Reactive yield: stop when a higher-priority robot is too close.
        # WHCA* gives MOSTLY-disjoint plans; this layer prevents collisions
        # when plans converge briefly at chokepoints.  Other robots' positions
        # are queried via TF (map frame) inside _should_yield.
        self._robot_name = self.get_namespace().lstrip('/')
        self._priorities = ROBOT_NAMES[:]
        # Robots parked on their goal.  Yielding priority to one achieves
        # nothing — it is never going to move out of the way — and because
        # priority_manager ranks by ascending distance to goal, a parked robot
        # holds the HIGHEST priority indefinitely.  With goal_grid_spacing at
        # 0.8 m and YIELD_DISTANCE_M at 1.20 m, every robot whose goal sits
        # next to a parked one would wait forever, 0.8 m short of arriving.
        self._finished = set()
        # time.monotonic() of the last empty plan, or None when we have a route.
        self._empty_path_since = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.declare_parameter('v_max', 0.5)
        self.declare_parameter('w_max', 0.5)
        self.declare_parameter('alpha', 1.0)
        self.declare_parameter('beta',  1.0)
        self.declare_parameter('tol',  0.3)
        self.declare_parameter('robot_frame', 'base_link')
        self.clt_plan_path = self.create_client(GetPlan, 'path_planning/plan_path')
        self.clt_smooth_path = self.create_client(ProcessPath, 'path_planning/smooth_path')
        self.create_subscription(Bool, 'kill', self._kill_callback, 10)
        self.create_subscription(Bool, '/replan_needed', self._replan_callback, 10)
        self.create_subscription(Bool, 'whca_replan', self._whca_replan_callback, 10)
        # Priority order for reactive yield (positions queried via TF inside
        # _should_yield, so no need to subscribe to other robots' /odom).
        _latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(String, '/robot_priorities',
            lambda msg: setattr(self, '_priorities', msg.data.split(',')), 10)
        # Track which robots are parked on their goal, and clear the flag when
        # one is given a new goal (GNFC may reassign).
        for _r in ROBOT_NAMES:
            self.create_subscription(
                Bool, f'/{_r}/navigation/goal_reached',
                lambda msg, r=_r: self._finished.add(r) if msg.data else None, 10)
            self.create_subscription(
                PoseStamped, f'/{_r}/goal_pose',
                lambda msg, r=_r: self._finished.discard(r), _latched)
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 1)
        # Declares "I am stopped on purpose".  swarm_health_monitor cannot tell
        # a robot that is yielding from one that has failed — both sit still
        # with zero cmd_vel — so it used to declare a legitimately waiting robot
        # dead after DEATH_TIMEOUT.  Yielding is the coordination working, not a
        # fault, so the controller states it explicitly.
        self.pub_waiting = self.create_publisher(Bool, 'navigation/waiting', 1)
        self._waiting = None
        self.pub_goal_reached = self.create_publisher(Bool, 'navigation/goal_reached', 1)
        self.sub_goal_pose = self.create_subscription(PoseStamped, 'goal_pose', self.callback_goal_pose, _latched)

    def spin(self):
        robot_pose_tf_ready = False
        self.get_logger().info("Waiting for plan path service...")
        while not self.clt_plan_path.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for plan path service...')
        self.get_logger().info("Plan path service is now available...")
        clt_timeout = 3
        self.get_logger().info("Waiting for smooth path service...")
        while not self.clt_smooth_path.wait_for_service(timeout_sec=0.5) and clt_timeout > 0:
            self.get_logger().info("Waiting for smooth path service...")
            clt_timeout -= 1
        if self.clt_smooth_path.wait_for_service(timeout_sec=0.5):
            self.get_logger().info("Smooth path service is available")
        else:
            self.get_logger().info("Smooth path service is not available")
        robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.get_logger().info("Waiting for robot pose tf to be available")
        while rclpy.ok() and not robot_pose_tf_ready:
            try:
                t = self.tf_buffer.lookup_transform("map", robot_frame, rclpy.time.Time())
                robot_pose_tf_ready = True
            except TransformException as ex:
                robot_pose_tf_ready = False
            rclpy.spin_once(self)
            self.get_clock().sleep_for(Duration(seconds=0.02))
        self.get_logger().info("Robot pose tf is now available")

        state = SM_INIT
        while rclpy.ok():
            robot_p, robot_a = self.get_robot_pose()
            if state == SM_INIT:
                self.get_logger().info("Ready to execute new path. Waiting for new goal...")
                state = SM_WAIT_FOR_NEW_GOAL

            elif state == SM_WAIT_FOR_NEW_GOAL:
                rclpy.spin_once(self, timeout_sec=0.1)
                if not self.robot_alive:
                    continue   # robot is dead — ignore new goals
                if self.new_goal_pose:
                    self.new_goal_pose = False
                    # Don't plan immediately — wait for the coordinator's staggered
                    # whca_replan so higher-priority robots publish their reservations
                    # first.  Timeout 3 s as fallback (independent mode / no coordinator).
                    deadline = time.monotonic() + 3.0
                    while rclpy.ok() and not self.needs_replan and time.monotonic() < deadline:
                        rclpy.spin_once(self, timeout_sec=0.1)
                    if not self.needs_replan:
                        self.get_logger().info("Coordinator timeout — planning independently")
                    self.needs_replan = False
                    state = SM_PLAN_PATH
                elif self.needs_replan and numpy.linalg.norm(self.goal_pose) > 0.01:
                    # Coordinator triggered a replan while stuck here (e.g. after empty path).
                    self.needs_replan = False
                    if self._robot_name in self._finished:
                        # Already parked on this goal.  cost_map paints a
                        # PARKED_ROBOT_RADIUS_M disc at our own position, so the
                        # goal now sits inside our own obstacle and every replan
                        # would come back empty.  Nothing to do until a new goal
                        # arrives, which unpaints us.
                        continue
                    state = SM_PLAN_PATH
                elif (self._empty_path_since is not None
                      and self._robot_name not in self._finished
                      and numpy.linalg.norm(self.goal_pose) > 0.01
                      and time.monotonic() - self._empty_path_since
                          >= EMPTY_PATH_RETRY_SEC):
                    # Left without a route last time.  Retry on our own instead
                    # of waiting for the coordinator's next staggered tick.
                    self._empty_path_since = time.monotonic()
                    state = SM_PLAN_PATH

            elif state == SM_PLAN_PATH:
                # Kill signal may arrive during the replan sleep — bail out if so.
                if not self.robot_alive:
                    state = SM_WAIT_FOR_NEW_GOAL
                    continue
                # Clear any replan flags accumulated while entering this state.
                # We are planning right now — a concurrent replan signal is redundant.
                self.needs_replan = False
                self.map_changed  = False
                self.get_logger().info("Trying to plan path from" + str(self.robot_pose) + " to "+ str(self.goal_pose))
                request = GetPlan.Request()
                request.start.pose.position.x = self.robot_pose[0]
                request.start.pose.position.y = self.robot_pose[1]
                request.goal.pose.position.x = self.goal_pose[0]
                request.goal.pose.position.y = self.goal_pose[1]
                future = self.clt_plan_path.call_async(request)
                rclpy.spin_until_future_complete(self, future)
                # Kill signal might have arrived during spin_until_future_complete.
                if not self.robot_alive:
                    state = SM_WAIT_FOR_NEW_GOAL
                    continue
                path = future.result().plan
                self.get_logger().info("Path planned with " + str(len(path.poses)) + " points")
                state = SM_SMOOTH_PATH

            elif state == SM_SMOOTH_PATH:
                req = ProcessPath.Request()
                req.path = path
                if self.clt_smooth_path.wait_for_service(timeout_sec=0.1):
                    future = self.clt_smooth_path.call_async(req)
                    rclpy.spin_until_future_complete(self, future)
                    path = future.result().processed_path
                    self.get_logger().info("Path smoothed succesfully")
                else:
                    self.get_logger().info("Smooth path service is not available")
                state = SM_FOLLOWING_PATH

            elif state == SM_FOLLOWING_PATH:
                v_max = self.get_parameter('v_max').get_parameter_value().double_value
                w_max = self.get_parameter('w_max').get_parameter_value().double_value
                alpha = self.get_parameter('alpha').get_parameter_value().double_value
                beta  = self.get_parameter('beta').get_parameter_value().double_value
                tol   = self.get_parameter('tol').get_parameter_value().double_value
                self.get_logger().info("Following path with [v_max, w_max, alpha, beta, tol]="+str([v_max, w_max, alpha, beta, tol]))
                path_points = [numpy.asarray([p.pose.position.x, p.pose.position.y]) for p in path.poses]
                if path_points:
                    self._empty_path_since = None
                if not path_points:
                    # No coordinated route right now: the robot yields and the
                    # coordinator retries. Deliberate, not a failure.
                    self.get_logger().warn("Empty path — no solution found, waiting for new goal")
                    self._set_waiting(True)
                    self._empty_path_since = time.monotonic()
                    state = SM_WAIT_FOR_NEW_GOAL
                    continue
                self.pure_pursuit(path_points, alpha, beta, v_max, w_max, tol)
                self.pub_cmd_vel.publish(Twist())
                goal_reached = numpy.linalg.norm(self.robot_pose - path_points[-1]) < tol * 2
                if not self.robot_alive:
                    self.get_logger().info("Robot killed during path following — not publishing goal_reached")
                    state = SM_WAIT_FOR_NEW_GOAL
                elif goal_reached:
                    # Reached the actual goal — ignore any pending replan requests
                    self.needs_replan = False
                    self.pub_goal_reached.publish(Bool(data=True))
                    self.get_logger().info("Global goal point reached")
                    state = SM_SAVE_DATA
                elif self.needs_replan:
                    self.needs_replan = False
                    self.pub_cmd_vel.publish(Twist())
                    if self.map_changed:
                        self.map_changed = False
                        self.get_logger().info("Replanning — waiting for map rebuild after robot death...")
                        self.get_clock().sleep_for(Duration(seconds=1.5))
                    else:
                        self.get_logger().info("Replanning (WHCA* window update)")
                    state = SM_PLAN_PATH
                else:
                    self.pub_goal_reached.publish(Bool(data=True))
                    self.get_logger().info("Global goal point reached")
                    state = SM_SAVE_DATA

            elif state == SM_SAVE_DATA:
                self._set_waiting(False)
                s = ""
                for d in self.nav_data:
                    s += str(d[0]) +","+ str(d[1]) +","+ str(d[2]) +","+ str(d[3]) +","+ str(d[4]) +","+ str(d[5]) +","+ str(d[6]) + "\n"
                f = open(self.data_file, "w")
                f.write(s)
                f.close()
                state = SM_INIT
                
            rclpy.spin_once(self)
            self.get_clock().sleep_for(Duration(seconds=0.005))


def main(args=None):
    rclpy.init(args=args)
    pure_pursuit_node = PurePursuitNode()
    pure_pursuit_node.spin()
    pure_pursuit_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

