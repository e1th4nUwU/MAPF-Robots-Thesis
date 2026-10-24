import math

from swarm_mapf import grid

# Steps to the four orthogonal and diagonal neighbors in (i,j) coordinates
ORTHOGONAL = [(1, 0), (-1, 0), (0, 1), (0, -1)]
DIAGONAL = [(1, 1), (1, -1), (-1, 1), (-1, -1)]


def cell_to_world(i, j, info, cell_size):
    """
    Convert coarse cell indices to the world coordinates of each cell centre

    The lattice is anchored at the origin of the map and never at anything
    belonging to a robot or to a region so that every robot uses the same
    lattice.

    Args:
        i: column index in the coarse lattice
        j: row index in the coarse lattice
        info: nav_msgs/MapMetaData of the underlying grid
        cell_size: side of a coarse cell, in metres

    Returns:
        The (x, y) coordinates of the cell centre, in metres
    """
    x = info.origin.position.x + (i + 0.5) * cell_size
    y = info.origin.position.y + (j + 0.5) * cell_size

    return x, y


def world_to_cell(x, y, info, cell_size):
    """
    Convert world coordinates to coarse cell indices

    Args:
        x: abscissa in the map frame, in metres
        y: ordinate in the map frame, in metres
        info: nav_msgs/MapMetaData of the underlying grid
        cell_size: side of a coarse cell, in metres

    Returns:
        The (i, j) indices of the coarse cell containing the point; they
        may fall outside the map so check before indexing
    """
    i = math.floor((x - info.origin.position.x) / cell_size)
    j = math.floor((y - info.origin.position.y) / cell_size)

    return i, j


def is_free(i, j, occupancy_grid, cell_size):
    """
    Tell whether a coarse cell can hold a robot

    A coarse cell is free when its centre is free in the inflated map; 
    the map is already inflated so the robot can stand on the centre
    without colliding with any obstacle.

    Args:
        i: column index in the coarse lattice
        j: row index in the coarse lattice
        occupancy_grid: the nav_msgs/OccupancyGrid, already inflated
        cell_size: side of a coarse cell, in metres

    Returns:
        True if a robot can stand on that cell
    """
    # Transform the coarse cell indices to world coordinates
    x, y = cell_to_world(i, j, occupancy_grid.info, cell_size)
    # Convert world coordinates to fine cell indices
    fine_i, fine_j = grid.world_to_cell(x, y, occupancy_grid.info)
    # Check if the fine cell is free
    return grid.is_free(fine_i, fine_j, occupancy_grid)



def orthogonal_neighbors(i, j, occupancy_grid, cell_size):
    """
    List the orthogonal neighbors of a coarse cell that are free and 
    reachable in one step.

    Args:
        i: column index in the coarse lattice
        j: row index in the coarse lattice
        occupancy_grid: the nav_msgs/OccupancyGrid, already inflated
        cell_size: side of a coarse cell, in metres

    Returns:
        A list of (i, j) cells, each free and reachable in one step
    """
    neighbors = []
    for di, dj in ORTHOGONAL:
        # Neighboring cell indices plus the step
        ni, nj = i + di, j + dj
        # Check if the neighboring cell is free and reachable
        if is_free(ni, nj, occupancy_grid, cell_size):
            # Check if the midpoint between the two cells is free
            if _midpoint_is_free(i, j, ni, nj, occupancy_grid, cell_size):
                neighbors.append((ni, nj))
                
    return neighbors


def _midpoint_is_free(i, j, ni, nj, occupancy_grid, cell_size):
    """
    Tell whether the straight segment between two coarse cells is clear.

    Args:
        i: column index of the cell being left
        j: row index of the cell being left
        ni: column index of the cell being entered
        nj: row index of the cell being entered
        occupancy_grid: the nav_msgs/OccupancyGrid, already inflated
        cell_size: side of a coarse cell, in metres

    Returns:
        True if the midpoint falls on a free fine cell
    """
    # Convert the coarse cell indices to world coordinates
    x1, y1 = cell_to_world(i, j, occupancy_grid.info, cell_size)
    x2, y2 = cell_to_world(ni, nj, occupancy_grid.info, cell_size)
    # Compute the midpoint in world coordinates
    mid_x = (x1 + x2) / 2
    mid_y = (y1 + y2) / 2
    # Convert the midpoint to fine cell indices
    mid_i, mid_j = grid.world_to_cell(mid_x, mid_y, occupancy_grid.info)
    # Check if the fine cell at the midpoint is free
    return grid.is_free(mid_i, mid_j, occupancy_grid)