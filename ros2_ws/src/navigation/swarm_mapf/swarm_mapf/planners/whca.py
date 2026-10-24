"""
Windowed Hierarchical Cooperative A*

Plans the robots one at a time, in order of priority, each one against
the reservations of everyone before it. Cooperation reaches only w steps
ahead: beyond the window a robot ignores the others, and the plan is
rebuilt often enough that the part being executed is always the part
that was planned cooperatively
"""

from swarm_mapf import reservations, space_time_astar

# How many steps ahead the robots cooperate
# By cooperation, we mean the distance by which a robot
# claims its path in the reservation table, so that the next robot
# sees it as an obstacle. Beyond that distance, the robot ignores the
# others and plans on its own without caring about collisions with other
# agents.
WINDOW = 8


def plan(starts, goals, occupancy_grid, cell_size, window=WINDOW):
    """
    Plan every robot, in order of priority, against a shared table

    The priority is the order of `starts`: the robots are planned as
    they come and the first one gets its way. There is no separate list
    of priorities, so whoever builds `starts` decides who yields, and
    building it from an unordered source silently changes the result

    Args:
        starts: robot name to its (i, j) coarse cell, highest priority
            first
        goals: robot name to the (i, j) cell it wants, read by name
        occupancy_grid: the nav_msgs/OccupancyGrid, already inflated
        cell_size: side of a coarse cell, in metres
        window: how many steps ahead the robots cooperate

    Returns:
        A map from robot name to its list of (i, j) cells, or None when
        some robot has no path at all
    """
    # One table for everyone: each agent reads it through a view
    # that hides its own entries, so it can keep the route it already had when replanning
    # before other agents block it
    table = reservations.ReservationTable()
    paths = {}

    for robot in starts:
        # The view is what hides the table from the search, and what lets
        # this robot through its own entries
        path = space_time_astar.plan(starts[robot], goals[robot],
                                     occupancy_grid, cell_size,
                                     table.view(robot))
        # One robot without a path means the whole assignment is unsolvable for
        # this planner
        if path is None:
            return None

        paths[robot] = path

        # Only the windowed part of the path is claimed. Beyond w the
        # robots are not cooperating, so claiming there would block
        # somebody over a plan that is going to be rebuilt anyway
        # TODO: Silver truncates the search at w too, and estimates the
        # rest with the heuristic. Here the search still runs to the
        # goal and only the reservation is windowed, which costs more
        # time per call; worth measuring both before writing chapter 6
        claimed = path[:window + 1]

        # hold_until matters only for a robot that arrives inside the
        # window: it parks on its goal and has to keep blocking it. One
        # that was cut by the window is still moving, and for it
        # arrival already equals window, so nothing extra is written
        table.reserve(robot, claimed, hold_until=window)

    return paths
