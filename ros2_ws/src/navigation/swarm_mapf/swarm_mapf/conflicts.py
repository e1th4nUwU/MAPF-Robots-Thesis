"""
Collisions between a set of paths, which is what CBS branches on

A pair of paths collides in one of two ways: both robots on the same
cell at the same step, or both crossing the same edge in opposite
directions during the same step. The second kind is invisible to a
table that only reserves cells, and it is the whole reason
edge_is_free exists

A robot that has finished is not gone: it stands on its last cell, so
every path is read here as if it continued there forever
"""

from itertools import combinations


def _positions(paths, step):
    """
    Say where every robot is at one step

    A robot whose path is already over stays parked on its last cell,
    which is what it does in reality and what keeps it an obstacle for
    the robots that are still moving

    Args:
        paths: map from robot name to its list of (i, j) cells
        step: the step being looked at

    Returns:
        A map from robot name to the (i, j) cell it occupies
    """
    positions = {}
    for robot, path in paths.items():
        if step < len(path):
            positions[robot] = path[step]
        else:
            positions[robot] = path[-1]
    return positions


def _pairs(paths):
    """
    List every pair of robots once, in the same order on every run

    Sorted on purpose: CBS branches on the conflict this module reports,
    so taking the pairs in dictionary order would make two runs over the
    same input explore different trees

    Args:
        paths: map from robot name to its list of (i, j) cells

    Returns:
        A list of (robot, robot) pairs, each appearing once
    """
    return list(combinations(sorted(paths), 2))


def first_conflict(paths):
    """
    Find the earliest collision between a set of paths

    Args:
        paths: map from robot name to its list of (i, j) cells, one per
            step, all starting at the same step

    Returns:
        None when the paths are compatible, or a tuple describing the
        earliest collision: the two robots, the step, and what kind it
        is. CBS turns that tuple into the two constraints it branches
        on. For 'vertex' the step is when both robots are on the cell;
        for 'edge' it is the step the move starts at, so it can be
        handed to edge_is_free unchanged
    """
    pairs = _pairs(paths)
    horizon = max(len(path) for path in paths.values())

    previous = None
    for step in range(horizon):
        positions = _positions(paths, step)
        for first, second in pairs:
            # Both robots on the same cell at the same step
            if positions[first] == positions[second]:
                return (first, second, step, 'vertex')
            # Or each one taking the cell the other is leaving, which no
            # pair of cells on its own reveals. The move being reported
            # started at the previous step
            if previous is not None \
                    and positions[first] == previous[second] \
                    and positions[second] == previous[first]:
                return (first, second, step - 1, 'edge')
        previous = positions

    return None


def count_conflicts(paths):
    """
    Count every collision in a set of paths, not only the first

    This one is for the experiments and not for CBS: it says how bad a
    set of paths is, where first_conflict only says that it is bad

    Args:
        paths: map from robot name to its list of (i, j) cells

    Returns:
        How many robot pairs collide, counted once per pair and step
    """
    pairs = _pairs(paths)
    horizon = max(len(path) for path in paths.values())

    conflict_count = 0
    previous = None
    for step in range(horizon):
        positions = _positions(paths, step)
        for first, second in pairs:
            if positions[first] == positions[second]:
                conflict_count += 1
            # Not an elif on the same condition: a swap needs the two
            # robots to end up on different cells, so a pair can only
            # be one kind of conflict at a given step
            elif previous is not None \
                    and positions[first] == previous[second] \
                    and positions[second] == previous[first]:
                conflict_count += 1
        previous = positions

    return conflict_count
