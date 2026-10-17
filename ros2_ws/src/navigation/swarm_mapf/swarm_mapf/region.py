import math

from swarm_mapf import grid


def bounding_box(points):
    """
    Compute the axis-aligned bounding box of a set of points

    Args:
        points: sequence of geometry_msgs/Point32

    Returns:
        The tuple (x_min, y_min, x_max, y_max), in metres
    """
    xs = [p.x for p in points]
    ys = [p.y for p in points]
    return min(xs), min(ys), max(xs), max(ys)


def point_in_polygon(x, y, points):
    """
    Decide whether a point lies inside a polygon, by ray casting

    Args:
        x: abscissa of the query point, in metres
        y: ordinate of the query point, in metres
        points: vertices of the polygon as geometry_msgs/Point32 in order

    Returns:
        True if the point is inside the polygon
    """
    inside = False
    n = len(points)
    j = n - 1
    for i in range(n):
        # Check if the edge (j, i) crosses the horizontal line y
        xi, yi = points[i].x, points[i].y
        xj, yj = points[j].x, points[j].y
        # If the edge crosses the horizontal line, check if the crossing is to the right of the point
        if (yi > y) != (yj > y):
            # Compute the x coordinate of the intersection of the edge with the horizontal line y
            x_cross = xi + (y - yi) * (xj - xi) / (yj - yi)
            if x < x_cross:
                inside = not inside
        j = i
    return inside


def _lattice_count(extent, separation):
    """
    Count how many lattice points of a given pitch fit along an extent

    Args:
        extent: length of the bounding box along one axis, in metres
        separation: pitch of the lattice, in metres

    Returns:
        How many points fit, at least one
    """
    count = math.floor(extent / separation) + 1
    if count > 1 and abs(extent - (count - 1) * separation) < 1e-9:
        count -= 1
    return count


def candidate_cells(region_points, occupancy_grid, separation):
    """
    Build the set of admissible goal cells inside a region

    Args:
        region_points: vertices of the region as geometry_msgs/Point32
        occupancy_grid: the nav_msgs/OccupancyGrid, already inflated

    Returns:
        A list of distinct (i, j) cells, ordered row by row and then by
        column, so the result does not depend on iteration order
    """
    info = occupancy_grid.info
    x_min, y_min, x_max, y_max = bounding_box(region_points)

    n_x = _lattice_count(x_max - x_min, separation)
    n_y = _lattice_count(y_max - y_min, separation)

    # Centred in the bounding box, not anchored at its corner: anchoring
    # puts the first candidate on the edge of the region, which is where
    # the inflated obstacles are, and those candidates die
    offset_x = ((x_max - x_min) - (n_x - 1) * separation) / 2.0
    offset_y = ((y_max - y_min) - (n_y - 1) * separation) / 2.0

    cells = []
    seen = set()
    for b in range(n_y):
        for a in range(n_x):
            # Compute the world coordinates of the candidate point
            x = x_min + offset_x + a * separation
            y = y_min + offset_y + b * separation
            if not point_in_polygon(x, y, region_points):
                continue
            # The pitch is exactly s here, but snapping to the cell
            # moves each goal by up to half a cell per axis, so a pair
            # ends up at s - eps, eps <= sqrt(2) * resolution
            cell = grid.world_to_cell(x, y, info)
            if cell in seen:
                continue
            if not grid.is_free(cell[0], cell[1], occupancy_grid):
                continue
            seen.add(cell)
            cells.append(cell)
    return cells
