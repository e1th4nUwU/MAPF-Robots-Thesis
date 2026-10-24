import heapq

from swarm_mapf import lattice

# How many steps ahead the search is allowed to look; this is measured
# in steps, not seconds, and the caller must decide what to do when the
# search fails to find a path within that horizon
# TODO: make this a parameter of plan() and pass it down from the planner
MAX_STEPS = 200


def plan(start, goal, occupancy_grid, cell_size, constraints,
         start_time=0, max_steps=MAX_STEPS):
    """
    Find the earliest path from start to goal that obeys the constraints.

    Standard A*, but with the differences that make it a search over
    (i, j, t) states instead of only (i, j) states. The search is bounded
    in time, so it may fail to find a path even if one exists, and the
    caller must decide what to do then. The search is not aware of the
    robot's name, so it cannot ignore its own entries in the reservation
    table. The caller must wrap the table in a view that does that, or
    else the robot will see its own plan as an obstacle.

    Args:
        start: the (i, j) coarse cell the robot is on
        goal: the (i, j) coarse cell it wants
        occupancy_grid: the nav_msgs/OccupancyGrid, already inflated
        cell_size: side of a coarse cell, in metres
        constraints: anything answering cell_is_free and edge_is_free
        start_time: step the robot is at start
        max_steps: how many steps past start_time the search may look

    Returns:
        The list of (i, j) cells, one per step from start_time onwards,
        or None when no path exists within max_steps
    """
    # Data structures for the search
    open_set = []
    came_from = {}

    # Initial state and sanity check
    if not constraints.cell_is_free(start, start_time):
        return None

    # Structure: (i, j, t) states
    #   (i, j) are the coordinates of the cell
    #   t is the step it is reached at, which is also the g_score
    start_state = (start[0], start[1], start_time)
    visited = {start_state}
    h_score = _heuristic(start, goal)

    # Add the start state to the open set with its f_score (g + h) and h_score
    # Structure: (f_score, h_score, i, j, t)
    #   f_score is the total estimated cost (g + h)
    #   h_score is the heuristic cost to reach the goal from this state
    # The g_score is the same as the time step, so we use that for the f_score
    # heapq will prioritize the lowest f_score, and in case of ties, the lowest h_score, then the lowest i, j, t
    heapq.heappush(open_set, (start_time + h_score, h_score,
                              start[0], start[1], start_time))

    # Main loop
    while open_set:
        # Get the state with the lowest f_score
        _, _, i, j, t = heapq.heappop(open_set)
        current_cell = (i, j)
        current_state = (i, j, t)

        # Check if goal is reached. Arriving is enough for WHCA*, where a
        # robot that reaches its goal stays parked on it, and the planner
        # holds that cell in the table until the horizon
        
        # TODO(cbs): not enough once the goal itself can be constrained. A
        # constraint on goal at a later step makes this path invalid, since
        # the robot is still standing there; CBS needs the search to keep
        # going until t passes the last constraint on the goal cell
        if current_cell == goal:
            return _reconstruct(came_from, current_state)

        # Nothing beyond the horizon is worth expanding
        if t + 1 > start_time + max_steps:
            continue

        # Explore neighbors
        for neighbor in _successors(current_cell, t, occupancy_grid,
                                    cell_size, constraints):
            # Create the neighbor state with incremented time
            neighbor_state = (neighbor[0], neighbor[1], t + 1)

            # Skip if this state was already reached; the first time is
            # the cheapest time, so there is nothing to improve
            if neighbor_state in visited:
                continue

            # Add the neighbor state to the visited set and record its parent
            visited.add(neighbor_state)
            came_from[neighbor_state] = current_state
            # Add the neighbor state to the open set with its f_score and h_score
            g_score = t + 1
            h_score = _heuristic(neighbor, goal)
            heapq.heappush(open_set, (g_score + h_score, h_score,
                                      neighbor[0], neighbor[1], t + 1))

    # The open set ran dry: boxed in, or the goal is past the horizon
    return None


def _successors(cell, t, occupancy_grid, cell_size, constraints):
    """
    List the cells reachable from one state in a single step

    Args:
        cell: the (i, j) cell the robot is on
        t: the step it is there at
        occupancy_grid: the nav_msgs/OccupancyGrid, already inflated
        cell_size: side of a coarse cell, in metres
        constraints: anything answering cell_is_free and edge_is_free

    Returns:
        A list of (i, j) cells it can be on at t + 1, waiting included
    """
    # What the map allows plus the cell itself, which is always reachable by waiting
    candidates = lattice.orthogonal_neighbors(cell[0], cell[1],
                                              occupancy_grid, cell_size)
    candidates.append(cell)

    successors = []
    for candidate in candidates:
        # Free on arrival, which is t + 1 and not t
        # The cell may be taken when the robot leaves it, but it must be free when it arrives
        if not constraints.cell_is_free(candidate, t + 1):
            continue
        # No robot will come the other way during the step
        # Waiting asks about the edge from a cell to itself
        if not constraints.edge_is_free(cell, candidate, t):
            continue
        successors.append(candidate)

    return successors


def _heuristic(cell, goal):
    """
    Estimate the steps left to the goal, ignoring the other robots using
    Manhattan distance.

    Args:
        cell: the (i, j) cell being evaluated
        goal: the (i, j) cell wanted

    Returns:
        A lower bound on the number of steps from cell to goal
    """
    return abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])


def _reconstruct(came_from, state):
    """
    Walk the parent chain back to the start

    Args:
        came_from: map from state to the state it was reached from
        state: the final (i, j, t) state

    Returns:
        The list of (i, j) cells in order, one per step
    """
    path = []
    current = state
    while current in came_from:
        path.append((current[0], current[1]))
        current = came_from[current]
    path.append((current[0], current[1]))
    return list(reversed(path))
