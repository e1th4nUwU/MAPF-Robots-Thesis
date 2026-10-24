"""
From a plan in cells and steps to something the robot can follow

The planners work in coarse cells and integer steps. pure_pursuit works
in metres and seconds. This is the only place where the two meet, and
the only place DT appears
"""

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from swarm_mapf import lattice

# Seconds one step of the plan lasts. Bounded by the turn and not by the
# advance: crossing a 0.8 m cell at the speed pure_pursuit really holds
# takes about 2.3 s, but turning 90 degrees at w_max = 0.5 rad/s takes
# 3.1 s, and a plan whose steps are shorter than the robot needs drifts
# out of step with its own reservations
DT = 3.2

NANOSECONDS = 1000000000


def cells_to_path(cells, info, cell_size, frame, start_stamp, dt=DT):
    """
    Turn a plan into a nav_msgs/Path with a stamp per pose

    Args:
        cells: the (i, j) coarse cells, one per step
        info: nav_msgs/MapMetaData of the underlying grid
        cell_size: side of a coarse cell, in metres
        frame: tf frame the path belongs to
        start_stamp: time the first cell corresponds to
        dt: seconds per step

    Returns:
        A nav_msgs/Path, one PoseStamped per cell, stamped start_stamp
        plus k * dt
    """
    path = Path()
    path.header.frame_id = frame
    path.header.stamp = start_stamp

    for step, cell in enumerate(cells):
        pose = PoseStamped()
        pose.header.frame_id = frame
        # The stamp is what carries the plan. Two identical poses in a
        # row are a wait, and without the stamps the follower cannot
        # tell that from a duplicated point it should skip
        pose.header.stamp = _stamp_plus(start_stamp, step * dt)

        x, y = lattice.cell_to_world(cell[0], cell[1], info, cell_size)
        pose.pose.position.x = x
        pose.pose.position.y = y
        # The plan says nothing about heading: the lattice has no
        # orientation and pure_pursuit derives its own from the next
        # point, so this is the identity and not a planned angle
        pose.pose.orientation.w = 1.0

        path.poses.append(pose)

    return path


def _stamp_plus(stamp, seconds):
    """
    Offset a ROS stamp by a number of seconds

    Done in integer nanoseconds and not in floats: a plan is tens of
    poses long and adding 3.2 s one float at a time drifts

    Args:
        stamp: the builtin_interfaces/Time to start from
        seconds: how many seconds to add

    Returns:
        A new builtin_interfaces/Time
    """
    total = (stamp.sec * NANOSECONDS + stamp.nanosec
             + int(round(seconds * NANOSECONDS)))
    return Time(sec=total // NANOSECONDS, nanosec=total % NANOSECONDS)
