import math

# Occupancy above this value is treated as an obstacle
FREE_THRESHOLD = 50


def world_to_cell(x, y, info):
    """
    Convert world coordinates to cell indices

    Args:
        x: abscissa in the map frame, in metres
        y: ordinate in the map frame, in metres
        info: nav_msgs/MapMetaData of the grid

    Returns:
        The (i, j) indices of the cell containing the point; they may
        fall outside the grid so check with in_bounds() before indexing
    """
    i = math.floor((x - info.origin.position.x) / info.resolution)
    j = math.floor((y - info.origin.position.y) / info.resolution)
    return i, j


def cell_to_world(i, j, info):
    """
    Convert cell indices to the world coordinates of the cell centre

    Args:
        i: column index
        j: row index
        info: nav_msgs/MapMetaData of the grid

    Returns:
        The (x, y) coordinates of the cell centre, in metres
    """
    x = info.origin.position.x + (i + 0.5) * info.resolution
    y = info.origin.position.y + (j + 0.5) * info.resolution
    return x, y


def in_bounds(i, j, info):
    """
    Tell whether a pair of indices names a cell of the grid

    Args:
        i: column index
        j: row index
        info: nav_msgs/MapMetaData of the grid

    Returns:
        True if the cell exists in the grid
    """
    return 0 <= i < info.width and 0 <= j < info.height


def cell_value(i, j, grid):
    """
    Read the occupancy stored for a cell

    Args:
        i: column index
        j: row index
        grid: the nav_msgs/OccupancyGrid, not just its info

    Returns:
        The occupancy in [0, 100], -1 if the cell is unknown, or None if
        the indices fall outside the grid
    """
    if not in_bounds(i, j, grid.info):
        return None
    return grid.data[j * grid.info.width + i]


def is_free(i, j, grid):
    """
    Tell whether a cell is known to be free

    Args:
        i: column index
        j: row index
        grid: the nav_msgs/OccupancyGrid, not just its info

    Returns:
        True if the cell exists, is known, and its occupancy is at most
        FREE_THRESHOLD
    """
    value = cell_value(i, j, grid)
    if value is None or value < 0:
        return False
    return value <= FREE_THRESHOLD


def cells_for_distance(distance, info):
    """
    Convert a distance in metres into that distance in cells

    Args:
        distance: a length in metres
        info: nav_msgs/MapMetaData of the grid

    Returns:
        The number of cells spanning at least that distance
    """
    return int(math.ceil(distance / info.resolution))
