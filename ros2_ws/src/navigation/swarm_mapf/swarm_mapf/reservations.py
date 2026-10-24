def _edge_key(origin, destination, t):
    """
    Build the key that identifies one transition, without a direction

    The two cells are sorted, so a robot going from a to b and one going
    from b to a land on the same entry: that head on swap is exactly the
    collision the cell entries cannot see.

    Args:
        origin: the (i, j) cell being left
        destination: the (i, j) cell being entered
        t: the step the move starts at

    Returns:
        A hashable key for the transition
    """
    return (tuple(sorted([origin, destination])), t)


class ReservationTable:
    """
    Cells and transitions claimed by the robots that already planned
    """

    def __init__(self):
        """
        Create an empty table. It is shared by all robots, and each robot
        sees a view of it that hides its own entries so it can keep the
        route it already had when it replans.
        """
        # Defines the table as a map from (cell, step) to robot name, and from (edge, step) to robot name
        self._cell_entries = {}
        # Defines the table as a map from (edge, step) to robot name, where edge is a tuple of two cells sorted to be direction-agnostic
        self._edge_entries = {}

    def reserve(self, robot, path, start_time=0, hold_until=None):
        """
        Write a whole plan into the table

        Args:
            robot: name of the robot the plan belongs to
            path: the (i, j) cells it visits, one per step
            start_time: step the first cell of the path corresponds to
            hold_until: step up to which the last cell stays claimed. A
                robot that arrives does not vanish, it stands on its goal
                and blocks it for everyone who plans later
        """
        # Traverse the path
        for i, cell in enumerate(path):
            # Calculate the time step for this cell
            t = start_time + i
            # Reserve the cell and edge entries
            self._cell_entries[(cell, t)] = robot
            # If this is not the first cell, reserve the edge entry
            if i > 0:
                prev_cell = path[i - 1]
                # Reserve the transition, keyed without a direction
                self._edge_entries[_edge_key(prev_cell, cell, t - 1)] = robot

        # A robot that arrives does not vanish: it stands on its goal.
        # Without this the cell stops being claimed the step after the
        # path ends and everyone who plans later routes straight through
        # a parked robot
        if hold_until is not None and path:
            arrival = start_time + len(path) - 1
            for t in range(arrival + 1, hold_until + 1):
                self._cell_entries[(path[-1], t)] = robot

    def release(self, robot):
        """
        Drop every entry belonging to one robot, before it replans

        Args:
            robot: name of the robot whose entries go away
        """
        # Remove all cell entries for this robot
        self._cell_entries = {k: v for k, v in self._cell_entries.items() if v != robot}
        # Remove all edge entries for this robot
        self._edge_entries = {k: v for k, v in self._edge_entries.items() if v != robot}

    def view(self, robot):
        """
        Hand the search what it needs, from the point of view of a robot

        The search never sees this table: it gets an object with
        cell_is_free and edge_is_free that already knows which robot is
        asking, so it can let that robot through its own reservations

        Args:
            robot: name of the robot about to plan

        Returns:
            An object answering cell_is_free(cell, t) and
            edge_is_free(origin, destination, t)
        """
        return _ReservationView(self, robot)


class _ReservationView:
    """
    One robot's reading of the table, as the search expects it

    Exists so space_time_astar does not have to know what a robot is, nor
    that there is a table behind this at all
    """

    def __init__(self, table, robot):
        self._table = table
        self._robot = robot

    def cell_is_free(self, cell, t):
        """
        Tell whether this robot may stand on a cell at a given step

        Args:
            cell: the (i, j) coarse cell
            t: the step

        Returns:
            True if nobody else claimed that cell at that step
        """
        # Its own entry is not an obstacle: a robot that replans has to
        # be free to keep the route it already had
        owner = self._table._cell_entries.get((cell, t))
        return owner is None or owner == self._robot

    def edge_is_free(self, origin, destination, t):
        """
        Tell whether this robot may cross between two cells at a step

        The collision this catches is the head on swap, which the cell
        entries cannot see

        Args:
            origin: the (i, j) cell being left
            destination: the (i, j) cell being entered
            t: the step the move starts at

        Returns:
            True if no other robot crosses that transition at that step
        """
        owner = self._table._edge_entries.get(
            _edge_key(origin, destination, t))
        return owner is None or owner == self._robot
