"""
Checks on the coarse lattice the MAPF planner searches over

Two kinds of test live here. The first ones use small grids built by
hand, so a failure points straight at the arithmetic. The last one loads
the real scenarios and asks the only question that matters in practice:
can every robot still reach its goal zone on this lattice

The inflation is redone here instead of asking cost_map for it, so the
test runs without a live system. The radius is the one the launch file
passes to cost_map, and that coupling is the point: if someone changes
the inflation and the maze stops being traversable, this test says so
"""

import importlib.util
import math
import os
from collections import deque

import pytest

from nav_msgs.msg import OccupancyGrid

from swarm_mapf import lattice

# The cell size the planner uses, decided by measuring the narrowest
# throat of the maze: 1.95 m of real gap, 0.75 m of free band once the
# obstacles grow by INFLATION_M. A coarser lattice steps over that throat
# and reports a passage that is not there
CELL_SIZE = 0.8

# Must match the inflation_radius that motion_planning.launch.py passes
# to cost_map
INFLATION_M = 0.6

RESOLUTION = 0.05


def _make_grid(width, height, resolution=RESOLUTION, origin=(0.0, 0.0)):
    """
    Build an empty OccupancyGrid to fill in by hand

    Args:
        width: columns of the fine grid
        height: rows of the fine grid
        resolution: metres per fine cell
        origin: (x, y) of the lower left corner, in metres

    Returns:
        A nav_msgs/OccupancyGrid with every cell free
    """
    grid = OccupancyGrid()
    grid.info.width = width
    grid.info.height = height
    grid.info.resolution = resolution
    grid.info.origin.position.x = float(origin[0])
    grid.info.origin.position.y = float(origin[1])
    grid.data = [0] * (width * height)
    return grid


def _occupy(grid, i, j):
    """Mark one fine cell as an obstacle"""
    grid.data[j * grid.info.width + i] = 100


# ── Conversions ───────────────────────────────────────────────────────────────


@pytest.mark.parametrize('cell', [(0, 0), (5, 3), (12, 7), (-3, -2)])
def test_round_trip(cell):
    """A cell converted to metres and back is the same cell"""
    grid = _make_grid(400, 240, origin=(-10.0, -6.0))
    x, y = lattice.cell_to_world(*cell, grid.info, CELL_SIZE)
    assert lattice.world_to_cell(x, y, grid.info, CELL_SIZE) == cell


def test_origin_is_not_ignored():
    """
    The lattice is anchored at the origin of the map

    Dropping the origin leaves cell_to_world and world_to_cell consistent
    with each other and wrong against the map, so the round trip above
    passes while every lookup lands somewhere else. This pins the centre
    of a map whose origin is not zero
    """
    grid = _make_grid(400, 240, origin=(-10.0, -6.0))
    # Cell (12, 7) of a 0.8 m lattice is the centre of a 20 x 12 m map
    x, y = lattice.cell_to_world(12, 7, grid.info, CELL_SIZE)
    assert x == pytest.approx(0.0)
    assert y == pytest.approx(0.0)


def test_cell_outside_the_map_is_not_free():
    """A cell beyond the border is not free, it is nothing"""
    grid = _make_grid(40, 40)
    assert not lattice.is_free(-1, 0, grid, CELL_SIZE)
    assert not lattice.is_free(100, 0, grid, CELL_SIZE)


# ── Neighbours ────────────────────────────────────────────────────────────────


def test_obstacle_on_the_centre_removes_the_cell():
    """A coarse cell whose centre is occupied is not a neighbour"""
    grid = _make_grid(80, 80)
    centre = lattice.cell_to_world(2, 1, grid.info, CELL_SIZE)
    i, j = int(centre[0] / RESOLUTION), int(centre[1] / RESOLUTION)
    _occupy(grid, i, j)
    assert (2, 1) not in lattice.orthogonal_neighbors(1, 1, grid, CELL_SIZE)


def test_midpoint_cuts_a_passage_that_is_not_there():
    """
    Two free centres with an obstacle between them are not connected

    This is the failure the coarse lattice invites: it samples the map
    one point per cell, so a wall thinner than the cell falls between two
    samples and the lattice promises a step the robot cannot take
    """
    grid = _make_grid(80, 80)
    left = lattice.cell_to_world(1, 1, grid.info, CELL_SIZE)
    right = lattice.cell_to_world(2, 1, grid.info, CELL_SIZE)
    mid_x = (left[0] + right[0]) / 2.0
    _occupy(grid, int(mid_x / RESOLUTION), int(left[1] / RESOLUTION))
    assert (2, 1) not in lattice.orthogonal_neighbors(1, 1, grid, CELL_SIZE)


# ── The scenarios ─────────────────────────────────────────────────────────────


def _scenario_module():
    """
    Load the installed scenario launch file

    The scenarios build their own map at launch time, so this is the only
    source of truth for what the robots will actually drive through
    """
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(
        get_package_share_directory('swarm_bringup'),
        'launch', 'scenario.launch.py')
    spec = importlib.util.spec_from_file_location('scenario_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _inflated_grid(scenario, module):
    """
    Rebuild a scenario map and grow its obstacles by INFLATION_M

    Args:
        scenario: name of the scenario
        module: the loaded scenario launch module

    Returns:
        A nav_msgs/OccupancyGrid matching what cost_map would serve
    """
    numpy = pytest.importorskip('numpy')
    image = module._build_scenario_map_image(scenario)
    # The pgm has row 0 at the top and the OccupancyGrid has it at the
    # bottom, and 89 is the value map_server reads as occupied
    occupied = numpy.flipud(image < 89)
    height, width = occupied.shape

    radius = int(round(INFLATION_M / module.MAP_RESOLUTION))
    offsets = numpy.ogrid[-radius:radius + 1, -radius:radius + 1]
    disk = (offsets[1] ** 2 + offsets[0] ** 2) <= radius ** 2
    inflated = numpy.zeros_like(occupied)
    for j, i in zip(*numpy.nonzero(occupied)):
        j0, j1 = max(0, j - radius), min(height, j + radius + 1)
        i0, i1 = max(0, i - radius), min(width, i + radius + 1)
        inflated[j0:j1, i0:i1] |= disk[j0 - (j - radius):j1 - (j - radius),
                                       i0 - (i - radius):i1 - (i - radius)]

    grid = _make_grid(width, height, module.MAP_RESOLUTION,
                      (-module.ARENA_X, -module.ARENA_Y))
    grid.data = [100 if v else 0 for v in inflated.reshape(-1).tolist()]
    return grid


def _reaches(start, zone, grid):
    """
    Tell whether a breadth first search gets from a cell into a zone

    Args:
        start: the (i, j) coarse cell the robot spawns on
        zone: the goal zone, as the scenario launch file declares it
        grid: the inflated nav_msgs/OccupancyGrid

    Returns:
        True if some cell of the zone is reachable
    """
    seen = {start}
    queue = deque([start])
    while queue:
        cell = queue.popleft()
        x, y = lattice.cell_to_world(*cell, grid.info, CELL_SIZE)
        if (zone['x_min'] <= x <= zone['x_max']
                and zone['y_min'] <= y <= zone['y_max']):
            return True
        for neighbour in lattice.orthogonal_neighbors(*cell, grid, CELL_SIZE):
            if neighbour not in seen:
                seen.add(neighbour)
                queue.append(neighbour)
    return False


@pytest.mark.parametrize('scenario', ['towers', 'rocks', 'maze'])
def test_every_robot_reaches_its_goal_zone(scenario):
    """
    On the working lattice, no scenario is cut in two

    A coarse lattice can disconnect what the map connects: the centres
    land on the walls of a corridor and the corridor disappears. The maze
    is the one that decides, it is the tightest of the three
    """
    module = _scenario_module()
    grid = _inflated_grid(scenario, module)
    zone = module.SCENARIO_GOAL_ZONES[scenario]

    for robot in module.SCENARIO_ROBOTS[scenario]:
        start = lattice.world_to_cell(
            robot['x'], robot['y'], grid.info, CELL_SIZE)
        assert lattice.is_free(*start, grid, CELL_SIZE), \
            f"{robot['name']} aparece sobre una celda ocupada en {scenario}"
        assert _reaches(start, zone, grid), \
            f"{robot['name']} no llega a la zona meta en {scenario}"
