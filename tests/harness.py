"""
Run the MAPF planner without ROS

The planning layer imports nothing from ROS: lattice, space_time_astar,
reservations, constraints, conflicts and planners talk to each other and
to math, heapq and itertools. The only ROS shaped thing they touch is an
OccupancyGrid, and only through six attributes, so the class below is
enough to run the whole planner in plain Python

That is the point of this folder. The real scenarios turned out to be
easy -- cooperation there costs +0 and +1 step and nobody ever waits --
and an instance that does not challenge the algorithm cannot show that
it works. Here the instance is drawn by hand and the right answer is
known before running anything
"""

import os
import sys

# The planner lives in the ROS package, but nothing here needs ROS built
# or sourced: this is a path, not a dependency
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', 'ros2_ws', 'src', 'navigation', 'swarm_mapf'))

# One metre per cell and one cell per character, so a lattice cell is
# exactly a character of the drawing and an index is readable by eye
CELL_SIZE = 1.0


class _Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class _Origin:
    def __init__(self, x, y):
        self.position = _Point(x, y)


class _Info:
    def __init__(self, width, height, resolution, origin):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = _Origin(*origin)


class Grid:
    """
    What nav_msgs/OccupancyGrid looks like to the planner

    Six attributes: info.width, info.height, info.resolution,
    info.origin.position.x, info.origin.position.y and data. Anything
    else the message carries is never read
    """

    def __init__(self, width, height, data,
                 resolution=CELL_SIZE, origin=(0.0, 0.0)):
        self.info = _Info(width, height, resolution, origin)
        self.data = data


def parse(drawing):
    """
    Turn a drawing into a grid, the robot starts and their goals

    The format is meant to be read on paper and pasted into the thesis:

        #  obstacle
        .  free
        A  where robot A starts, uppercase
        a  where robot A is going, lowercase

    The first line of the drawing is the top row, which is the highest y,
    so the rows are flipped: an OccupancyGrid counts rows upwards from
    its origin and a drawing counts them downwards from the top

    Args:
        drawing: the map as one multi-line string

    Returns:
        The tuple (grid, starts, goals), where starts and goals map a
        robot name to its (i, j) cell
    """
    rows = [line for line in drawing.strip('\n').split('\n') if line.strip()]
    height = len(rows)
    width = max(len(row) for row in rows)

    data = [0] * (width * height)
    starts = {}
    goals = {}

    for row_index, row in enumerate(rows):
        # Flip: the drawing reads downwards, the grid counts upwards
        j = height - 1 - row_index
        for i, character in enumerate(row):
            if character == '#':
                data[j * width + i] = 100
            elif character.isupper():
                starts[character] = (i, j)
            elif character.islower():
                goals[character.upper()] = (i, j)

    return Grid(width, height, data), starts, goals


def load(instance):
    """
    Build everything one instance needs to be planned

    Args:
        instance: an entry of instances.INSTANCES

    Returns:
        The tuple (grid, starts, goals), with starts ordered by the
        priority the instance declares, because whca reads the priority
        from the order of that dictionary and nowhere else
    """
    grid, starts, goals = parse(instance['drawing'])
    goals = dict(goals)
    goals.update(instance.get('goals', {}))

    priority = instance.get('priority') or sorted(starts)
    ordered = {}
    for name in priority:
        ordered[name] = starts[name]

    missing = set(ordered) - set(goals)
    if missing:
        raise ValueError('%s: sin meta para %s'
                         % (instance['name'], ', '.join(sorted(missing))))
    return grid, ordered, goals


def waits(path):
    """
    Count how many steps of a path are spent standing still

    Args:
        path: the list of (i, j) cells, one per step

    Returns:
        How many times the robot stayed where it was
    """
    return sum(1 for a, b in zip(path, path[1:]) if a == b)
