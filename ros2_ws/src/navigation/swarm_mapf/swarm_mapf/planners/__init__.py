"""
Registry of MAPF planners, one module each

Same shape as allocators: the service node names a planner by string and
never imports one directly, so comparing WHCA* against CBS is changing a
parameter and not touching the node. Every planner answers the same
signature:

    plan(starts, goals, occupancy_grid, cell_size) ->
        {robot: [(i, j), ...]} or None

`starts` and `goals` map a robot name to a coarse cell, and the order of
`starts` is the order of priority, highest first: a planner that cares
about priority reads it from there instead of taking a second list. The
answer is one path per robot, one cell per step, all starting at step 0
"""

from swarm_mapf.planners import whca

PLANNERS = {
    'whca': whca.plan,
}


def get(name):
    """
    Look up a planner by name

    Args:
        name: key of the planner

    Returns:
        The plan function, or None when the name is unknown
    """
    return PLANNERS.get(name)


def names():
    """
    List the planners available

    Returns:
        The keys, sorted, for error messages and for the node to log
    """
    return sorted(PLANNERS)
