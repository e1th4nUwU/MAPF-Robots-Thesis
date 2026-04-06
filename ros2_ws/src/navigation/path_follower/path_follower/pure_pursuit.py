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
from std_msgs.msg import Bool
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
import numpy

NAME = "Jorge Eithan Treviño Selles"

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

        LOOKAHEAD_DIST = 0.5   # metres ahead to target on the path

        # Index of the furthest path point we have "passed" — the lookahead
        # search starts from here so it always moves forward along the path.
        anchor_idx = 0

        while rclpy.ok() and self.robot_alive and not self.needs_replan:
            robot_p, robot_a = self.get_robot_pose()
            robot_x, robot_y = robot_p[0], robot_p[1]

            # ── Stop if final goal is close enough ──────────────────────────
            if numpy.linalg.norm(robot_p - path[-1]) < tol:
                break

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

    def _kill_callback(self, msg: Bool) -> None:
        if msg.data and self.robot_alive:
            self.robot_alive = False
            self.get_logger().warn("Robot killed — stopping navigation")

    def _replan_callback(self, msg: Bool) -> None:
        if msg.data and self.robot_alive:
            self.needs_replan = True
            # Stop immediately so the robot doesn't keep moving while replanning
            self.pub_cmd_vel.publish(Twist())
            self.get_logger().info("Replan requested — stopping and replanning")

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
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 1)
        self.pub_goal_reached = self.create_publisher(Bool, 'navigation/goal_reached', 1)
        self.sub_goal_pose = self.create_subscription(PoseStamped, 'goal_pose', self.callback_goal_pose, 1)

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
                    state = SM_PLAN_PATH

            elif state == SM_PLAN_PATH:
                self.get_logger().info("Trying to plan path from" + str(self.robot_pose) + " to "+ str(self.goal_pose))
                request = GetPlan.Request()
                request.start.pose.position.x = self.robot_pose[0]
                request.start.pose.position.y = self.robot_pose[1]
                request.goal.pose.position.x = self.goal_pose[0]
                request.goal.pose.position.y = self.goal_pose[1]
                future = self.clt_plan_path.call_async(request)
                rclpy.spin_until_future_complete(self, future)
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
                self.pure_pursuit(path_points, alpha, beta, v_max, w_max, tol)
                self.pub_cmd_vel.publish(Twist())
                if self.needs_replan:
                    # A teammate died — stop, wait for updated map, then replan
                    self.needs_replan = False
                    self.pub_cmd_vel.publish(Twist())  # ensure stopped
                    self.get_logger().info("Replanning due to map change (teammate died) — waiting for updated map...")
                    self.get_clock().sleep_for(Duration(seconds=1.5))  # let cost_map publish new map
                    self.get_logger().info("Replanning now")
                    state = SM_PLAN_PATH
                elif not self.robot_alive:
                    # Killed during path following — do NOT publish goal_reached
                    self.get_logger().info("Robot killed during path following — not publishing goal_reached")
                    state = SM_WAIT_FOR_NEW_GOAL
                else:
                    self.pub_goal_reached.publish(Bool(data=True))
                    self.get_logger().info("Global goal point reached")
                    state = SM_SAVE_DATA

            elif state == SM_SAVE_DATA:
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

