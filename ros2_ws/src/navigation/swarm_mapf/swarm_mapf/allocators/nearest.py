import math


def allocate(robot_cells, candidates, occupancy_grid, separation_cells):
    """
    Assign one candidate cell to each robot, nearest first. Greedy as it gets lmao.
    Will dump later but for now it'll work.

    Args:
        robot_cells: current (i, j) cell of each robot, ordered by
            descending priority
        candidates: admissible (i, j) cells, as returned by
            region.candidate_cells
        occupancy_grid: the nav_msgs/OccupancyGrid; unused here, taken
            so every allocator shares one signature
        separation_cells: minimum distance between goals, in cells

    Returns:
        A list with one (i, j) cell per robot, in the same order, or
        None if some robot found no candidate left; returning None is
        would mean that the allocator failed :(
    """
    # List of candidates that are still free to be assigned
    free = list(candidates)
    goals = []

    # Assign one goal to each robot, in order of priority
    for robot in robot_cells:
        
        # No free cells!
        if not free:
            return None
        
        # Find the closest free candidate to the robot
        best = min(free, key=lambda c: _distance(robot, c))
        goals.append(best)
        
        # Remove all candidates that are too close to the chosen goal
        # This is a bit inefficient, so is this entire algorithm =)
        for c in free[:]:
            if _distance(best, c) < separation_cells:
                free.remove(c)

    return goals


def _distance(a, b):
    """
    Calculates the Euclidean distance between two cells, in cells
    """
    return math.hypot(a[0] - b[0], a[1] - b[1])
