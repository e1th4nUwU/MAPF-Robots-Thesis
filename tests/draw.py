"""
Draw a plan as one static picture, with the step written in colour

A space-time plan does not fit in a flat drawing: the same cell means
different things at different steps. The way out is to put the time in
the colour. One hue per robot, and inside that hue the lightness ramps
with t, pale at the start and saturated at the end, with the step number
printed in the cell

That makes the two things worth seeing visible without an animation. A
cell labelled 3-4 is a robot that stood still, which is a robot giving
way. Two touching cells carrying the same step in different hues is a
crossing. Both are invisible in a picture that only draws the routes

An instance with no plan is still drawn, map, starts and goals, because
the cases where WHCA* fails are the ones worth printing
"""

import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

# Categorical slots, in fixed order, assigned by sorted robot name so a
# robot keeps its hue when the priority changes
SERIES = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100']

SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK_2 = '#52514e'
OBSTACLE = '#d8d7d2'
GRIDLINE = '#e4e2dc'


def _ramp(hex_colour, position):
    """
    Step one hue from pale to full

    Args:
        hex_colour: the robot's hue
        position: 0.0 at the first step, 1.0 at the last

    Returns:
        An (r, g, b) tuple
    """
    base = matplotlib.colors.to_rgb(hex_colour)
    # Never fully white: the earliest step still has to read as the robot
    weight = 0.28 + 0.72 * position
    return tuple(1.0 - weight * (1.0 - channel) for channel in base)


def _occupancy(path):
    """
    Group the steps a robot spends on each cell

    Args:
        path: the list of (i, j) cells, one per step

    Returns:
        A dict from cell to the sorted list of steps spent on it
    """
    steps = {}
    for step, cell in enumerate(path):
        steps.setdefault(cell, []).append(step)
    return steps


def _label(steps):
    """
    Write a list of steps the way it reads fastest

    Consecutive steps on one cell are a wait and collapse into a range,
    because what matters is that the robot stayed, not for how many
    separate instants

    Args:
        steps: the sorted steps spent on a cell

    Returns:
        The label, such as '4' or '4-6' or '2, 7'
    """
    runs = []
    for step in steps:
        if runs and step == runs[-1][-1] + 1:
            runs[-1].append(step)
        else:
            runs.append([step])
    return ', '.join(str(run[0]) if len(run) == 1
                     else '%d-%d' % (run[0], run[-1]) for run in runs)


def draw(instance, grid, starts, goals, paths, path_png):
    """
    Render one instance, with its plan when there is one

    Args:
        instance: the entry of instances.INSTANCES
        grid: the harness Grid
        starts: robot name to its (i, j) cell
        goals: robot name to the (i, j) cell it wants
        paths: the plan, or None when the planner found none
        path_png: where to write the figure

    Returns:
        The path written
    """
    width, height = grid.info.width, grid.info.height
    order = sorted(starts)

    fig, ax = plt.subplots(figsize=(1.0 + width * 0.72, 1.4 + height * 0.72),
                           facecolor=SURFACE)
    ax.set_facecolor(SURFACE)

    # The map first, so everything else sits on top of it
    for i in range(width):
        for j in range(height):
            occupied = grid.data[j * width + i] > 50
            ax.add_patch(Rectangle((i, j), 1, 1,
                                   facecolor=OBSTACLE if occupied else SURFACE,
                                   edgecolor=GRIDLINE, linewidth=0.8))

    handles = []
    colours = {name: SERIES[order.index(name) % len(SERIES)] for name in order}

    # Collected before anything is drawn, because a cell used by two
    # robots at different steps has to be split between them. Painting
    # them one robot at a time hides whichever was drawn first, and the
    # cells that get shared are exactly the interesting ones
    usage = {}
    for name in order:
        path = (paths or {}).get(name)
        if path is None:
            usage.setdefault(starts[name], []).append((name, [0], 1))
            continue
        last = max(len(path) - 1, 1)
        for cell, steps in _occupancy(path).items():
            usage.setdefault(cell, []).append((name, steps, last))

    for cell, entries in usage.items():
        i, j = cell
        entries.sort(key=lambda entry: order.index(entry[0]))
        band = 1.0 / len(entries)
        for index, (name, steps, last) in enumerate(entries):
            colour = colours[name]
            bottom = j + index * band
            ax.add_patch(Rectangle(
                (i, bottom), 1, band,
                facecolor=_ramp(colour, steps[0] / last),
                edgecolor=colour, linewidth=1.0, zorder=3))
            # The label carries the robot's letter as well as the step,
            # so identity never rests on the colour alone. Ink and not
            # the series colour, to stay readable over the whole ramp
            ax.text(i + 0.5, bottom + band / 2, '%s %s' % (name, _label(steps)),
                    ha='center', va='center', color=INK,
                    fontsize=8 if len(entries) == 1 else 7, zorder=5)

    for name in order:
        colour = colours[name]
        # The goal is drawn even when there is no plan: half of these
        # instances exist to show that the planner cannot reach it
        gi, gj = goals[name]
        ax.add_patch(Circle((gi + 0.5, gj + 0.5), 0.42, fill=False,
                            edgecolor=colour, linewidth=2.2, zorder=4))

        path = (paths or {}).get(name)
        label = name if path is None else '%s  %d pasos, %d esperas' % (
            name, len(path) - 1,
            sum(1 for a, b in zip(path, path[1:]) if a == b))
        handles.append(plt.Line2D([], [], marker='s', linestyle='',
                                  color=colour, markersize=9, label=label))

    verdict = 'sin solucion' if paths is None else 'resuelta'
    ax.set_title('%s  —  %s\n%s' % (instance['name'], verdict,
                                    instance['asks']),
                 color=INK, fontsize=10, loc='left', pad=10)
    ax.legend(handles=handles, loc='upper left', bbox_to_anchor=(0.0, -0.02),
              ncol=len(handles), frameon=False, fontsize=8, labelcolor=INK_2)

    ax.set_xlim(0, width)
    ax.set_ylim(0, height)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    for side in ax.spines.values():
        side.set_visible(False)

    fig.tight_layout()
    fig.savefig(path_png, dpi=150, facecolor=SURFACE,
                bbox_inches='tight')
    plt.close(fig)
    return path_png
