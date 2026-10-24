"""
The constraints CBS imposes on one agent, at one node of its tree

CBS does not keep a shared table. It plans every agent as if it were
alone, looks for the first collision between the resulting paths, and
splits the search into two branches: in one, agent A is forbidden to be
there at that instant; in the other, agent B is. Each node of that tree
carries the constraints accumulated down its branch, and this is what
one agent sees of them

Written to answer the same two questions as a reservation view, so the
same space_time_astar works for both planners
"""


class ConstraintSet:
    """
    What one agent may not do, at one node of the constraint tree

    A vertex constraint forbids a cell at a step. An edge constraint
    forbids a transition at a step. Both are the two kinds of collision
    CBS can find between a pair of paths
    """

    def __init__(self, vertices=None, edges=None):
        """
        Args:
            vertices: iterable of (cell, t) the agent may not occupy
            edges: iterable of (origin, destination, t) it may not cross
        """
        # TODO: keep them as sets. A node of the tree is built from its
        # parent plus one new constraint, so this has to be cheap to copy
        raise NotImplementedError

    def plus(self, vertex=None, edge=None):
        """
        Build the child of this node, with one constraint more

        Returns a new set instead of changing this one: the sibling
        branch keeps planning against the constraints of the parent

        Args:
            vertex: a (cell, t) to forbid, or None
            edge: an (origin, destination, t) to forbid, or None

        Returns:
            A new ConstraintSet
        """
        # TODO
        raise NotImplementedError

    def cell_is_free(self, cell, t):
        """
        Tell whether the agent may stand on a cell at a given step

        Args:
            cell: the (i, j) coarse cell
            t: the step

        Returns:
            True if no vertex constraint forbids it
        """
        # TODO
        raise NotImplementedError

    def edge_is_free(self, origin, destination, t):
        """
        Tell whether the agent may cross between two cells at a step

        Args:
            origin: the (i, j) cell being left
            destination: the (i, j) cell being entered
            t: the step the move starts at

        Returns:
            True if no edge constraint forbids it
        """
        # TODO: same ordering convention as the reservation table, or a
        # constraint written one way will not be read the other
        raise NotImplementedError


class Unconstrained:
    """
    Lets everything through

    The root of the CBS tree, and what you pass to the search to plan a
    robot as if it were alone. Also handy for testing the search without
    dragging a table along
    """

    def cell_is_free(self, cell, t):
        return True

    def edge_is_free(self, origin, destination, t):
        return True
