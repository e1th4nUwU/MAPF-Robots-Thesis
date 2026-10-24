"""Scenario-based swarm launch — 3 robots in Gazebo Harmonic.

Usage:
  ros2 launch swarm_bringup scenario.launch.py scenario:=towers
  ros2 launch swarm_bringup scenario.launch.py scenario:=rocks
  ros2 launch swarm_bringup scenario.launch.py scenario:=maze

Scenarios
---------
towers  Staggered cylindrical towers (rust/brick) in a 4-row slalom.
        Robots zig-zag east from x=-9 to the goal at x=9.

rocks   Scattered boulders (brown spheres + dark-grey rotated boxes).
        Robots navigate irregular terrain between x=-9 and x=9.

maze    Three-corridor maze with central convergence zone.
        North/middle/south corridors separated by sandstone walls with
        single central gaps (x=-1..1).  Each corridor has multiple orange
        chicanes forcing slalom navigation.  Goal zone sits in the east
        end of the middle corridor, so alvin (north) and simon (south)
        must enter the middle through central gaps — classic MAPF stress
        test where WHCA* coordination becomes essential.

All scenarios have a closed perimeter (grey-blue boundary walls) so robots
cannot escape the arena.

All robots:
  alvin — red   teodoro — green   simon — blue
  Start at x=-8 (west side), goal zone (green) on the east side.
"""

import math
import os
import re
import subprocess
import tempfile
import numpy as np

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    GroupAction,
    LogInfo,
)
from launch.launch_description_sources import (
    FrontendLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
import xacro

# ── Arena dimensions ───────────────────────────────────────────────────────────
#  x: -10 … +10  (20 m, robots start x=-8, goal x=8)
#  y:  -6 … + 6  (12 m)
ARENA_X  = 10.0   # half-length east/west
ARENA_Y  =  6.0   # half-width north/south
WALL_H   =  1.5   # boundary & obstacle wall height
WALL_T   =  0.2   # wall thickness

# ── Colors ─────────────────────────────────────────────────────────────────────
C_BOUNDARY  = ('0.45 0.50 0.55 1', '0.40 0.45 0.50 1')   # grey-blue concrete
C_TOWER     = ('0.75 0.28 0.12 1', '0.70 0.24 0.10 1')   # rust / brick-red
C_ROCK_SPH  = ('0.48 0.38 0.28 1', '0.44 0.34 0.24 1')   # warm brown
C_ROCK_BOX  = ('0.38 0.36 0.33 1', '0.33 0.31 0.28 1')   # dark charcoal
C_MAZE_WALL = ('0.80 0.72 0.55 1', '0.75 0.67 0.50 1')   # sandstone
C_MAZE_BAR  = ('0.92 0.55 0.05 1', '0.88 0.50 0.03 1')   # safety orange
C_GOAL      = ('0.10 0.90 0.20 0.7', '0.10 0.90 0.20 0.7')  # bright green

# ── Robot starting positions per scenario ─────────────────────────────────────

_ROBOTS_OPEN = [
    {'name': 'alvin',   'color': '0.9 0.1 0.1 1.0', 'x': -8.0, 'y':  1.5, 'z': 0.5, 'yaw': 0.0},
    {'name': 'teodoro', 'color': '0.1 0.8 0.1 1.0', 'x': -8.0, 'y':  0.0, 'z': 0.5, 'yaw': 0.0},
    {'name': 'simon',   'color': '0.1 0.1 0.9 1.0', 'x': -8.0, 'y': -1.5, 'z': 0.5, 'yaw': 0.0},
]

_ROBOTS_MAZE = [
    # One robot per corridor — north / middle / south.
    # Goal zone sits in middle corridor east end, so alvin & simon must
    # cross dividers via the central gaps to converge with teodoro.
    # Spawn y picked at the corridor centre so neither chicane catches the
    # robot on its very first step.
    {'name': 'alvin',   'color': '0.9 0.1 0.1 1.0', 'x': -8.0, 'y':  5.0, 'z': 0.5, 'yaw': 0.0},
    {'name': 'teodoro', 'color': '0.1 0.8 0.1 1.0', 'x': -8.0, 'y':  0.0, 'z': 0.5, 'yaw': 0.0},
    {'name': 'simon',   'color': '0.1 0.1 0.9 1.0', 'x': -8.0, 'y': -5.0, 'z': 0.5, 'yaw': 0.0},
]

SCENARIO_ROBOTS = {
    'towers': _ROBOTS_OPEN,
    'rocks':  _ROBOTS_OPEN,
    'maze':   _ROBOTS_MAZE,
}

# ── Fixed goal zones per scenario ─────────────────────────────────────────────
# Drives both the GNFC allocator (when planner != 'independent') and the
# green goal marker shown in Gazebo/RViz.  Picked so robots get the same
# deterministic goals every run — crucial for reproducible WHCA* tests.
SCENARIO_GOAL_ZONES = {
    # spacing must clear PARKED_ROBOT_RADIUS_M (0.70 m, cost_map.py) plus the
    # parking tolerance: pure_pursuit declares the goal reached within tol*2 =
    # 0.20 m, so a robot parks up to that far off and its painted disc shifts
    # with it.  At the old 0.80 m spacing only 0.10 m of margin was left, and a
    # robot that stopped short covered its neighbour's goal — the planner then
    # reported the goal 90-97% inside a map obstacle and returned no route.
    # 1.00 m leaves 0.10 m of margin beyond the worst-case parking error.
    'towers': {'x_min': 7.5, 'x_max': 9.0, 'y_min': -1.5, 'y_max': 1.5, 'spacing': 1.0},
    'rocks':  {'x_min': 8.0, 'x_max': 9.0, 'y_min': -1.5, 'y_max': 1.5, 'spacing': 1.0},
    # maze: widened to the whole usable east end of the middle corridor.
    # The corridor runs y in (-2, +2) with 0.2 m walls, so after the 0.6 m
    # inflation a robot centre fits in y in (-1.3, 1.3); to the east,
    # bar_m4 at x=6.0 inflates out to 6.7 and the perimeter wall inflates
    # in from 9.3. Staying 0.1 m inside both bounds gives the zone below,
    # which holds 3x3 candidates at 1.0 m spacing instead of 2x2.
    'maze':   {'x_min': 6.8, 'x_max': 9.2, 'y_min': -1.2, 'y_max': 1.2, 'spacing': 1.0},
}

# ── Shared bottlenecks ────────────────────────────────────────────────────────
# A funnel every robot must pass through.  Without one the fleet drives
# three parallel lines to three goals and the MAPF layer is never asked
# to do anything: robots meet only where they converge at the very end,
# past the cooperation window, and no robot ever yields.
#
# Gap arithmetic: cost_map inflates obstacles by 0.6 m, so a gap of width
# g leaves a band of g - 1.2 m for the robot centre.  g = 2.0 m leaves
# 0.8 m, which is exactly one cell of the planning lattice — single file,
# one robot at a time, which is the case WHCA* exists for.
#
# Declared here, once, because the Gazebo geometry and the map image are
# painted by two separate hand-written copies of the obstacle list.  A
# bottleneck present in only one of them is either a robot driving
# through a wall or a planner routing around one that is not there.
BOTTLENECK_GAP = 2.0

# (cx, cy, lx, ly) of each wall segment, in metres
SCENARIO_BOTTLENECKS = {
    # x = -7.0 for both, one metre east of the spawns, with the gap on
    # y = 0.  Searched rather than guessed: the position has to leave the
    # three robots connected to the goal zone, and most do not.  A gap on
    # y = 0 at x = -5.5 for instance opens onto the inflated tower at
    # (-4, 0) and dead-ends.  See the note above about the two copies
    'towers': [
        (-7.0, -(ARENA_Y + BOTTLENECK_GAP / 2) / 2, WALL_T,
         ARENA_Y - BOTTLENECK_GAP / 2),
        (-7.0, (ARENA_Y + BOTTLENECK_GAP / 2) / 2, WALL_T,
         ARENA_Y - BOTTLENECK_GAP / 2),
    ],
    'rocks': [
        (-7.0, -(ARENA_Y + BOTTLENECK_GAP / 2) / 2, WALL_T,
         ARENA_Y - BOTTLENECK_GAP / 2),
        (-7.0, (ARENA_Y + BOTTLENECK_GAP / 2) / 2, WALL_T,
         ARENA_Y - BOTTLENECK_GAP / 2),
    ],
    # The maze needed a second one.  Its divider gap is already a single
    # cell, but the north and south robots use different gaps and never
    # contend for either, and the middle corridor is 2.8 m wide after
    # inflation, which is three lattice columns: one lane per robot all
    # the way east.  Measured, not assumed: every goal assignment tried
    # on the old layout gave zero waits.  This pinches the middle
    # corridor down to one cell at x = 0, east of where the three merge,
    # so the fleet has to queue and WHCA* finally has to sequence it
    'maze': [
        (0.0, (ARENA_Y / 3 + BOTTLENECK_GAP / 2) / 2, WALL_T,
         ARENA_Y / 3 - BOTTLENECK_GAP / 2),
        (0.0, -(ARENA_Y / 3 + BOTTLENECK_GAP / 2) / 2, WALL_T,
         ARENA_Y / 3 - BOTTLENECK_GAP / 2),
    ],
}


# ── SDF primitive helpers ──────────────────────────────────────────────────────

def _box(name, cx, cy, cz, lx, ly, lz,
         roll=0.0, pitch=0.0, yaw=0.0, color=C_BOUNDARY):
    a, d = color
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{cx} {cy} {cz} {roll} {pitch} {yaw}</pose>
      <link name="link">
        <collision name="col">
          <geometry><box><size>{lx} {ly} {lz}</size></box></geometry>
        </collision>
        <visual name="vis">
          <geometry><box><size>{lx} {ly} {lz}</size></box></geometry>
          <material><ambient>{a}</ambient><diffuse>{d}</diffuse></material>
        </visual>
      </link>
    </model>"""


def _cylinder(name, cx, cy, cz, radius, length, color=C_TOWER):
    a, d = color
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{cx} {cy} {cz} 0 0 0</pose>
      <link name="link">
        <collision name="col">
          <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
        </collision>
        <visual name="vis">
          <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
          <material><ambient>{a}</ambient><diffuse>{d}</diffuse></material>
        </visual>
      </link>
    </model>"""


def _sphere(name, cx, cy, cz, radius, color=C_ROCK_SPH):
    a, d = color
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{cx} {cy} {cz} 0 0 0</pose>
      <link name="link">
        <collision name="col">
          <geometry><sphere><radius>{radius}</radius></sphere></geometry>
        </collision>
        <visual name="vis">
          <geometry><sphere><radius>{radius}</radius></sphere></geometry>
          <material><ambient>{a}</ambient><diffuse>{d}</diffuse></material>
        </visual>
      </link>
    </model>"""


# ── Reusable geometry blocks ───────────────────────────────────────────────────

def _boundary_walls(ax=ARENA_X, ay=ARENA_Y, h=WALL_H, t=WALL_T):
    """Four perimeter walls enclosing the arena (grey-blue concrete)."""
    span_x = ax * 2 + t   # full east-west span (closes corners)
    span_y = ay * 2
    return [
        _box('bnd_north',  0.0,  ay, h/2, span_x, t, h, color=C_BOUNDARY),
        _box('bnd_south',  0.0, -ay, h/2, span_x, t, h, color=C_BOUNDARY),
        _box('bnd_west',  -ax,  0.0, h/2, t, span_y, h, color=C_BOUNDARY),
        _box('bnd_east',   ax,  0.0, h/2, t, span_y, h, color=C_BOUNDARY),
    ]


def _goal_marker(name='goal', cx=8.0, cy=0.0, lx=1.0, ly=10.0):
    """Tall bright-green rectangle indicating the goal zone."""
    return _box(name, cx, cy, 0.75, lx, ly, 1.5, color=C_GOAL)


def _goal_marker_for_zone(name, zone):
    """Render a goal-zone rectangle from a SCENARIO_GOAL_ZONES entry."""
    cx = (zone['x_min'] + zone['x_max']) / 2.0
    cy = (zone['y_min'] + zone['y_max']) / 2.0
    lx = zone['x_max'] - zone['x_min']
    ly = zone['y_max'] - zone['y_min']
    return _box(name, cx, cy, 0.75, lx, ly, 1.5, color=C_GOAL)


# ── Scenario geometry ──────────────────────────────────────────────────────────

def _bottleneck_sdf(scenario):
    """Render the scenario's bottleneck walls from SCENARIO_BOTTLENECKS."""
    return [_box(f'choke_{scenario}_{i}', cx, cy, WALL_H / 2,
                 lx, ly, WALL_H, color=C_MAZE_WALL)
            for i, (cx, cy, lx, ly) in
            enumerate(SCENARIO_BOTTLENECKS.get(scenario, []))]


def _build_towers(show_goals=False):
    """4-row staggered slalom of rust-red cylinders.

    Each row alternates which y-positions are blocked, forcing robots to
    weave left and right on their way east.

        Row 1 (x=-4): y = -3,  0, +3   → gaps at y = ±1.5
        Row 2 (x=-1): y = -1.5, +1.5   → gap at y = 0
        Row 3 (x= 2): y = -3,  0, +3   → gaps at y = ±1.5
        Row 4 (x= 5): y = -1.5, +1.5   → gap at y = 0
    """
    positions = [
        (-4.0, -3.0), (-4.0,  0.0), (-4.0,  3.0),
        (-1.0, -1.5), (-1.0,  1.5),
        ( 2.0, -3.0), ( 2.0,  0.0), ( 2.0,  3.0),
        ( 5.0, -1.5), ( 5.0,  1.5),
    ]
    r, h = 0.28, 2.0
    parts = [_cylinder(f'tower_{i}', x, y, h/2, r, h, color=C_TOWER)
             for i, (x, y) in enumerate(positions)]
    parts += _bottleneck_sdf('towers')
    parts += _boundary_walls()
    if show_goals:
        parts.append(_goal_marker_for_zone('goal', SCENARIO_GOAL_ZONES['towers']))
    return '\n'.join(parts)


def _build_rocks(show_goals=False):
    """Scattered boulders: warm-brown spheres + dark-charcoal rotated boxes.

    Rocks are sized and placed so a ~0.5 m wide robot must actively plan
    a path rather than driving straight.
    """
    spheres = [
        # (cx, cy, radius)
        (-6.5,  1.8, 0.42),
        (-4.0, -3.0, 0.48),
        (-0.5,  3.5, 0.52),
        ( 2.5, -0.5, 0.38),
        ( 5.0,  2.0, 0.56),
        ( 3.5,  0.5, 0.42),
        (-2.5,  1.0, 0.35),
    ]
    boxes = [
        # (cx, cy, lx, ly, lz, yaw_deg)
        (-5.5, -1.5, 1.1, 0.7, 0.9,  30),
        (-4.5,  3.8, 1.6, 0.6, 0.8,  15),
        (-2.0,  0.5, 0.9, 1.3, 0.75, 45),
        (-1.5, -3.5, 1.1, 0.8, 0.85, 70),
        ( 1.0,  1.8, 1.4, 0.7, 0.9,  20),
        ( 2.0, -2.8, 1.0, 1.2, 0.8,  55),
        ( 4.5,  2.8, 1.3, 0.8, 0.85, 10),
        ( 5.5, -1.8, 1.1, 0.9, 0.75, 80),
        ( 6.5,  0.5, 0.9, 1.1, 0.8,  35),
    ]
    parts = []
    for i, (cx, cy, r) in enumerate(spheres):
        parts.append(_sphere(f'rock_s{i}', cx, cy, r, r, color=C_ROCK_SPH))
    for i, (cx, cy, lx, ly, lz, yd) in enumerate(boxes):
        parts.append(_box(f'rock_b{i}', cx, cy, lz/2,
                          lx, ly, lz, yaw=math.radians(yd), color=C_ROCK_BOX))
    parts += _bottleneck_sdf('rocks')
    parts += _boundary_walls()
    if show_goals:
        parts.append(_goal_marker_for_zone('goal', SCENARIO_GOAL_ZONES['rocks']))
    return '\n'.join(parts)


def _build_maze(show_goals=False):
    """Three-corridor maze with forced slalom and central convergence zone.

    Each corridor (north / middle / south) has FOUR alternating chicane
    bars (S-N-S-N pattern in north, mirrored in south).  Bars are 2.0 m
    tall — after the cost_map's 0.5 m inflation the south-attached and
    north-attached bars' impassable zones overlap, so a robot **cannot**
    cross the corridor in a straight line.  It must zig-zag between
    bars with ~1 m amplitude.

    Dividers leave a single 2 m gap at x=-5.5..-3.5 — 0.8 m wide after
    inflation, exactly one lattice cell, so two robots cannot pass
    simultaneously → WHCA* must sequence them.  The gap sits west of
    centre on purpose: the three robots spawn at x=-8 and reach it
    within the cooperation window, so they negotiate while the plan
    still covers them instead of converging past the horizon.

    Layout (top view, S = glued to south side, N = north):

        y=+6  ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓
              [alvin →]   ←y=+5
                  ║N            ║N            bar_n2,n4 (x=-3,+6;  y=4..6)
              ║S        ║S                   bar_n1,n3 (x=-6,+3; y=2..4)
        y=+2  ════ GAP ════════════════════════
                  ║N            ║N            bar_m1,m3 (x=-6,+3; y=0..2)
              [teodoro →]
                          ║S        ║S        bar_m2,m4 (x=-3,+6; y=-2..0)
        y=-2  ════ GAP ════════════════════════
              ║N        ║N                   bar_s1,s3 (x=-6,+3; y=-4..-2)
                  ║S            ║S            bar_s2,s4 (x=-3,+6; y=-6..-4)
              [simon →]   ←y=-5
        y=-6  ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓
              x=-10                     x=+10

    Goal zone: x∈[6.8,9.2], y∈[-1.2,1.2] — east end of middle corridor.
    """
    W = ARENA_X   # 10
    h = WALL_H    # 1.5
    t = WALL_T    # 0.2
    span = W * 2  # 20

    # Bar length tuned so south- and north-attached chicanes leave DISJOINT
    # passable bands (each 1 m tall for the robot centre, with a 1 m
    # uncrossable gap between them).  height=2.0 m gives:
    #   south-bar inflated y∈[1.5, 4.5] (north corridor) → free above ≥ 4.5
    #   north-bar inflated y∈[3.5, 6.5] (north corridor) → free below ≤ 3.5
    # so the robot must occupy y≈5 at south-bars and y≈3 at north-bars —
    # a forced zig-zag of ~2 m amplitude across the corridor.
    BAR = 2.0

    parts = _bottleneck_sdf('maze') + [
        # ── Horizontal sandstone dividers with central 2 m gaps ──
        _box('wall_n_left',  -7.75,  2.0, h/2,  4.5, t, h, color=C_MAZE_WALL),
        _box('wall_n_right',  3.25,  2.0, h/2, 13.5, t, h, color=C_MAZE_WALL),
        _box('wall_s_left',  -7.75, -2.0, h/2,  4.5, t, h, color=C_MAZE_WALL),
        _box('wall_s_right',  3.25, -2.0, h/2, 13.5, t, h, color=C_MAZE_WALL),

        # ── North corridor: S, N, S, N pattern (4 chicanes) ──
        _box('bar_n1', -6.0,  3.0, h/2, t, BAR, h, color=C_MAZE_BAR),  # S, y∈[2,4]
        _box('bar_n2', -3.0,  5.0, h/2, t, BAR, h, color=C_MAZE_BAR),  # N, y∈[4,6]
        _box('bar_n3',  3.0,  3.0, h/2, t, BAR, h, color=C_MAZE_BAR),  # S
        _box('bar_n4',  6.0,  5.0, h/2, t, BAR, h, color=C_MAZE_BAR),  # N

        # ── Middle corridor: N, S, N, S (mirror; ensures teodoro zig-zags) ──
        _box('bar_m1', -6.0,  1.0, h/2, t, BAR, h, color=C_MAZE_BAR),  # N, y∈[0,2]
        _box('bar_m2', -3.0, -1.0, h/2, t, BAR, h, color=C_MAZE_BAR),  # S, y∈[-2,0]
        _box('bar_m3',  3.0,  1.0, h/2, t, BAR, h, color=C_MAZE_BAR),  # N
        _box('bar_m4',  6.0, -1.0, h/2, t, BAR, h, color=C_MAZE_BAR),  # S

        # ── South corridor: N, S, N, S (mirror of north along y=0) ──
        _box('bar_s1', -6.0, -3.0, h/2, t, BAR, h, color=C_MAZE_BAR),  # N, y∈[-4,-2]
        _box('bar_s2', -3.0, -5.0, h/2, t, BAR, h, color=C_MAZE_BAR),  # S, y∈[-6,-4]
        _box('bar_s3',  3.0, -3.0, h/2, t, BAR, h, color=C_MAZE_BAR),  # N
        _box('bar_s4',  6.0, -5.0, h/2, t, BAR, h, color=C_MAZE_BAR),  # S

        # ── Grey-blue perimeter ──
        _box('bnd_west', -W, 0.0, h/2, t, ARENA_Y*2, h, color=C_BOUNDARY),
        _box('bnd_east',  W, 0.0, h/2, t, ARENA_Y*2, h, color=C_BOUNDARY),
        _box('bnd_north_out', 0.0,  ARENA_Y, h/2, span+t, t, h, color=C_BOUNDARY),
        _box('bnd_south_out', 0.0, -ARENA_Y, h/2, span+t, t, h, color=C_BOUNDARY),
    ]
    if show_goals:
        parts.append(_goal_marker_for_zone('goal', SCENARIO_GOAL_ZONES['maze']))
    return '\n'.join(parts)


SCENARIO_GEOMETRY = {
    'towers': _build_towers,
    'rocks':  _build_rocks,
    'maze':   _build_maze,
}

VALID_SCENARIOS = list(SCENARIO_GEOMETRY.keys())

# ── Scenario map generation for map_server/RViz coherence ────────────────────

MAP_RESOLUTION = 0.05
MAP_FREE = 254
MAP_OCC = 0


def _world_to_grid(x, y, width, height, res=MAP_RESOLUTION):
    col = int((x + ARENA_X) / res)
    row = int((ARENA_Y - y) / res)
    col = max(0, min(width - 1, col))
    row = max(0, min(height - 1, row))
    return row, col


def _paint_rect(grid, cx, cy, lx, ly):
    h, w = grid.shape
    x0 = cx - lx / 2.0
    x1 = cx + lx / 2.0
    y0 = cy - ly / 2.0
    y1 = cy + ly / 2.0
    r0, c0 = _world_to_grid(x0, y1, w, h)
    r1, c1 = _world_to_grid(x1, y0, w, h)
    rmin, rmax = min(r0, r1), max(r0, r1)
    cmin, cmax = min(c0, c1), max(c0, c1)
    grid[rmin:rmax + 1, cmin:cmax + 1] = MAP_OCC


def _paint_rotated_rect(grid, cx, cy, lx, ly, yaw):
    h, w = grid.shape
    half_x = lx / 2.0
    half_y = ly / 2.0
    radius = math.sqrt(half_x * half_x + half_y * half_y)
    r0, c0 = _world_to_grid(cx - radius, cy + radius, w, h)
    r1, c1 = _world_to_grid(cx + radius, cy - radius, w, h)
    rmin, rmax = min(r0, r1), max(r0, r1)
    cmin, cmax = min(c0, c1), max(c0, c1)
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    for row in range(rmin, rmax + 1):
        y = ARENA_Y - (row + 0.5) * MAP_RESOLUTION
        for col in range(cmin, cmax + 1):
            x = -ARENA_X + (col + 0.5) * MAP_RESOLUTION
            dx = x - cx
            dy = y - cy
            local_x = cos_y * dx + sin_y * dy
            local_y = -sin_y * dx + cos_y * dy
            if abs(local_x) <= half_x and abs(local_y) <= half_y:
                grid[row, col] = MAP_OCC


def _paint_circle(grid, cx, cy, radius):
    h, w = grid.shape
    r0, c0 = _world_to_grid(cx - radius, cy + radius, w, h)
    r1, c1 = _world_to_grid(cx + radius, cy - radius, w, h)
    rmin, rmax = min(r0, r1), max(r0, r1)
    cmin, cmax = min(c0, c1), max(c0, c1)
    rr = radius * radius
    for row in range(rmin, rmax + 1):
        y = ARENA_Y - (row + 0.5) * MAP_RESOLUTION
        for col in range(cmin, cmax + 1):
            x = -ARENA_X + (col + 0.5) * MAP_RESOLUTION
            if (x - cx) * (x - cx) + (y - cy) * (y - cy) <= rr:
                grid[row, col] = MAP_OCC


def _paint_boundaries(grid):
    span_x = ARENA_X * 2 + WALL_T
    span_y = ARENA_Y * 2
    _paint_rect(grid, 0.0, ARENA_Y, span_x, WALL_T)
    _paint_rect(grid, 0.0, -ARENA_Y, span_x, WALL_T)
    _paint_rect(grid, -ARENA_X, 0.0, WALL_T, span_y)
    _paint_rect(grid, ARENA_X, 0.0, WALL_T, span_y)


def _paint_bottleneck(grid, scenario):
    """Paint the scenario's bottleneck walls from SCENARIO_BOTTLENECKS."""
    for cx, cy, lx, ly in SCENARIO_BOTTLENECKS.get(scenario, []):
        _paint_rect(grid, cx, cy, lx, ly)


def _build_scenario_map_image(scenario):
    width = int((ARENA_X * 2.0) / MAP_RESOLUTION)
    height = int((ARENA_Y * 2.0) / MAP_RESOLUTION)
    grid = np.full((height, width), MAP_FREE, dtype=np.uint8)
    _paint_boundaries(grid)

    if scenario == 'towers':
        positions = [
            (-4.0, -3.0), (-4.0, 0.0), (-4.0, 3.0),
            (-1.0, -1.5), (-1.0, 1.5),
            (2.0, -3.0), (2.0, 0.0), (2.0, 3.0),
            (5.0, -1.5), (5.0, 1.5),
        ]
        for x, y in positions:
            _paint_circle(grid, x, y, 0.28)
        _paint_bottleneck(grid, 'towers')

    elif scenario == 'rocks':
        spheres = [
            (-6.5, 1.8, 0.42),
            (-4.0, -3.0, 0.48),
            (-0.5, 3.5, 0.52),
            (2.5, -0.5, 0.38),
            (5.0, 2.0, 0.56),
            (3.5, 0.5, 0.42),
            (-2.5, 1.0, 0.35),
        ]
        boxes = [
            (-5.5, -1.5, 1.1, 0.7, 30),
            (-4.5, 3.8, 1.6, 0.6, 15),
            (-2.0, 0.5, 0.9, 1.3, 45),
            (-1.5, -3.5, 1.1, 0.8, 70),
            (1.0, 1.8, 1.4, 0.7, 20),
            (2.0, -2.8, 1.0, 1.2, 55),
            (4.5, 2.8, 1.3, 0.8, 10),
            (5.5, -1.8, 1.1, 0.9, 80),
            (6.5, 0.5, 0.9, 1.1, 35),
        ]
        for x, y, r in spheres:
            _paint_circle(grid, x, y, r)
        for x, y, lx, ly, yaw_deg in boxes:
            _paint_rotated_rect(grid, x, y, lx, ly, math.radians(yaw_deg))
        _paint_bottleneck(grid, 'rocks')

    elif scenario == 'maze':
        _paint_bottleneck(grid, 'maze')
        BAR = 2.0
        # Horizontal dividers with the gap at x∈[-5.5,-3.5], west of
        # centre so the three corridors merge inside the window
        _paint_rect(grid, -7.75,  2.0,  4.5, WALL_T)
        _paint_rect(grid,  3.25,  2.0, 13.5, WALL_T)
        _paint_rect(grid, -7.75, -2.0,  4.5, WALL_T)
        _paint_rect(grid,  3.25, -2.0, 13.5, WALL_T)
        # North corridor — 4 alternating chicanes (S, N, S, N)
        _paint_rect(grid, -6.0,  3.0, WALL_T, BAR)
        _paint_rect(grid, -3.0,  5.0, WALL_T, BAR)
        _paint_rect(grid,  3.0,  3.0, WALL_T, BAR)
        _paint_rect(grid,  6.0,  5.0, WALL_T, BAR)
        # Middle corridor — N, S, N, S
        _paint_rect(grid, -6.0,  1.0, WALL_T, BAR)
        _paint_rect(grid, -3.0, -1.0, WALL_T, BAR)
        _paint_rect(grid,  3.0,  1.0, WALL_T, BAR)
        _paint_rect(grid,  6.0, -1.0, WALL_T, BAR)
        # South corridor — N, S, N, S (mirror of north)
        _paint_rect(grid, -6.0, -3.0, WALL_T, BAR)
        _paint_rect(grid, -3.0, -5.0, WALL_T, BAR)
        _paint_rect(grid,  3.0, -3.0, WALL_T, BAR)
        _paint_rect(grid,  6.0, -5.0, WALL_T, BAR)

    return grid


def _write_scenario_map_files(scenario):
    grid = _build_scenario_map_image(scenario)
    map_dir = tempfile.mkdtemp(prefix=f'map_{scenario}_')
    pgm_path = os.path.join(map_dir, f'{scenario}.pgm')
    yaml_path = os.path.join(map_dir, f'{scenario}.yaml')

    with open(pgm_path, 'wb') as f:
        f.write(f'P5\n{grid.shape[1]} {grid.shape[0]}\n255\n'.encode('ascii'))
        f.write(grid.tobytes())

    yaml_text = f"""image: {pgm_path}
mode: trinary
resolution: {MAP_RESOLUTION}
origin: [{-ARENA_X}, {-ARENA_Y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
    with open(yaml_path, 'w', encoding='ascii') as f:
        f.write(yaml_text)

    return yaml_path

# ── World / robot assembly ─────────────────────────────────────────────────────

_GROUND_PLANE = """\
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient></material>
        </visual>
      </link>
    </model>
"""

_WORLD_PLUGINS = """\
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
    </light>
"""


def _robot_to_sdf(urdf_string, robot):
    """URDF string → renamed SDF <model> element with pose."""
    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
        f.write(urdf_string)
        urdf_path = f.name
    try:
        result = subprocess.run(
            ['gz', 'sdf', '-p', urdf_path],
            capture_output=True, text=True, check=True,
        )
    finally:
        os.unlink(urdf_path)

    name = robot['name']
    x, y, z, yaw = robot['x'], robot['y'], robot['z'], robot['yaw']
    model_sdf = result.stdout.replace(
        "<model name='justina'>",
        f"<model name='{name}'>\n    <pose>{x} {y} {z} 0 0 {yaw}</pose>",
        1,
    )
    match = re.search(r'(<model\b.*?</model>)', model_sdf, re.DOTALL)
    if not match:
        raise RuntimeError(f'gz sdf -p produced no <model> for {name}')
    return match.group(1)


def _build_world(robots_sdf, scenario_xml):
    world = f"""\
<?xml version="1.0" ?>
<sdf version="1.11">
  <world name="default">
{_WORLD_PLUGINS}
{_GROUND_PLANE}
{scenario_xml}
{''.join(robots_sdf)}
  </world>
</sdf>
"""
    with tempfile.NamedTemporaryFile(
            mode='w', suffix='.sdf', delete=False,
            prefix='world_scenario_') as f:
        f.write(world)
        return f.name


# ── Launch entry point ─────────────────────────────────────────────────────────

def launch_setup(context, *args, **kwargs):
    scenario = LaunchConfiguration('scenario').perform(context)
    use_rviz = LaunchConfiguration('use_rviz')
    planner = LaunchConfiguration('planner').perform(context)
    show_gazebo_goals = LaunchConfiguration('show_gazebo_goals').perform(context) == 'true'
    if scenario not in VALID_SCENARIOS:
        raise ValueError(
            f"Unknown scenario '{scenario}'. Valid: {VALID_SCENARIOS}"
        )

    robots = SCENARIO_ROBOTS[scenario]
    justina_pkg = get_package_share_directory('justina_description')
    xacro_file = os.path.join(justina_pkg, 'urdf', 'justina.xacro')

    robots_urdf, robots_sdf = [], []
    for robot in robots:
        name = robot['name']
        urdf = xacro.process_file(
            xacro_file,
            mappings={
                'body_color':    robot['color'],
                'cmd_vel_topic': f'/{name}/cmd_vel',
                'odom_topic':    f'/{name}/odom',
                'odom_frame':    f'{name}/odom',
                'base_frame':    f'{name}/base_link',
                'ground_truth_topic': f'/{name}/ground_truth',
            },
        ).toxml()
        robots_urdf.append(urdf)
        robots_sdf.append(_robot_to_sdf(urdf, robot))

    scenario_xml = SCENARIO_GEOMETRY[scenario](show_goals=show_gazebo_goals)
    world_path = _build_world(robots_sdf, scenario_xml)
    if show_gazebo_goals:
        print(f"[DEBUG] show_gazebo_goals={show_gazebo_goals}")
        print(f"[DEBUG] Generated world with goal markers")
    map_yaml_path = _write_scenario_map_files(scenario)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py',
            ])
        ),
        launch_arguments={'gz_args': f'-r -v 3 {world_path}'}.items(),
    )

    # /clock gets a bridge of its own, and specifically ros_gz_bridge's, because
    # that one declares the ROS publisher with the CLOCK QoS profile:
    # best-effort, depth 1.  Gazebo emits /clock once per physics step — 1 kHz
    # with the 1 ms max_step_size below — and a reliable, queued bridge turns
    # any momentary CPU starvation (cost_map precomputing its static maps takes
    # ~15 s of one core at startup) into a backlog it never drains: measured
    # 69 s behind and still growing at +0.7 s/s.  Every node then runs tens of
    # seconds behind the TF and odometry stamps Gazebo is producing, so RViz
    # finds every transform far in the future, and the bursts of catch-up
    # clock samples read to it as time jumping backwards — it answers by
    # clearing its TF buffer, which is the flicker.  Depth 1 best-effort cannot
    # queue, so after a stall the clock simply resynchronises.
    clock_bridge = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_bridge'), 'launch', 'clock_bridge.launch',
            ])
        ),
        launch_arguments={'bridge_name': 'clock_bridge'}.items(),
    )

    # Global TF from Gazebo (all robots in one topic).  Kept in a separate
    # process so it never competes with the clock stream for the one thread a
    # parameter_bridge has.
    tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='tf_bridge',
        arguments=[
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        output='screen',
    )

    # Shared infrastructure: map_server, cost_map, rviz (one instance)
    planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('motion_planning'), 'launch', 'motion_planning.launch.py'
            ])
        ),
        launch_arguments={
            'map_yaml': map_yaml_path,
            'use_rviz': use_rviz,
        }.items(),
    )

    # Per-robot navigation: a_star + path_smoothing + pure_pursuit.
    # planner:=none leaves it out, which is useful when only the goal
    # allocation is under test: the robots then stay parked where they
    # spawned and nothing competes for /get_inflated_map or cmd_vel.
    robot_nav_nodes = [] if planner == 'none' else [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('motion_planning'), 'launch', 'robot_nav.launch.py'
                ])
            ),
            launch_arguments={
                'robot_name': robot['name'],
                'planner': planner
            }.items(),
        )
        for robot in robots
    ]

    teleop_gui = Node(
        package='swarm_bringup',
        executable='swarm_teleop_gui.py',
        name='swarm_teleop_gui',
        output='screen',
    )

    viz_bridge = Node(
        package='swarm_bringup',
        executable='swarm_viz_bridge.py',
        name='swarm_viz_bridge',
        output='screen',
        parameters=[
            {'scenario': scenario},
            {'use_sim_time': True},
        ],
    )

    ground_truth_localizer = Node(
        package='swarm_bringup',
        executable='ground_truth_localizer.py',
        name='ground_truth_localizer',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    health_monitor = Node(
        package='swarm_bringup',
        executable='swarm_health_monitor.py',
        name='swarm_health_monitor',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    nodes = [
        LogInfo(msg=f"Launching scenario: {scenario}"),
        LogInfo(msg=f"Using generated map yaml: {map_yaml_path}"),
        gazebo,
        clock_bridge,
        tf_bridge,
        planning,
        *robot_nav_nodes,
        teleop_gui,
        viz_bridge,
        ground_truth_localizer,
        health_monitor,
    ]

    if planner == 'whca':
        nodes.append(Node(
            package='mapf_coordinator',
            executable='priority_manager',
            name='priority_manager',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ))
        nodes.append(Node(
            package='mapf_coordinator',
            executable='whca_coordinator',
            name='whca_coordinator',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'replan_period_sec': 3.0,
                'stagger_sec': 1.0,
            }],
        ))
        for robot in robots:
            nodes.append(GroupAction([
                PushRosNamespace(robot['name']),
                Node(
                    package='mapf_coordinator',
                    executable='whca_star',
                    name='whca_star',
                    output='screen',
                    parameters=[{
                        'robot_name': robot['name'],
                        'window_size': 80,
                        'diagonals': True,
                        'use_sim_time': True,
                    }],
                ),
            ]))

    # GNFC + visualizer auto-launch for coordinated modes — uses the fixed
    # per-scenario goal zone and feeds each robot's spawn position so the
    # very first allocation tick is deterministic (no (0,0) tie at startup).
    if planner in ('whca', 'cbs'):
        zone = SCENARIO_GOAL_ZONES[scenario]
        gnfc_params = {
            'use_sim_time': True,
            'goal_zone_x_min': zone['x_min'],
            'goal_zone_x_max': zone['x_max'],
            'goal_zone_y_min': zone['y_min'],
            'goal_zone_y_max': zone['y_max'],
            'goal_grid_spacing': zone['spacing'],
        }
        for robot in robots:
            gnfc_params[f"initial_x_{robot['name']}"] = float(robot['x'])
            gnfc_params[f"initial_y_{robot['name']}"] = float(robot['y'])
        nodes.append(Node(
            package='mapf_coordinator',
            executable='gnfc_goal_allocator',
            name='gnfc_goal_allocator',
            output='screen',
            parameters=[gnfc_params],
        ))
        nodes.append(Node(
            package='mapf_coordinator',
            executable='goal_zone_visualizer',
            name='goal_zone_visualizer',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'goal_zone_x_min': zone['x_min'],
                'goal_zone_x_max': zone['x_max'],
                'goal_zone_y_min': zone['y_min'],
                'goal_zone_y_max': zone['y_max'],
            }],
        ))

    for robot, urdf in zip(robots, robots_urdf):
        name = robot['name']
        nodes.append(GroupAction([
            PushRosNamespace(name),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{
                    'robot_description': urdf,
                    'publish_robot_description': True,
                    'use_sim_time': True,
                    'frame_prefix': f'{name}/',
                }],
                output='screen',
            ),
        ]))
        nodes.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'bridge_{name}',
            output='screen',
            arguments=[
                f'/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                f'/{name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                # Ground-truth model pose, consumed by ground_truth_localizer
                # to correct map->odom.  Wheel odometry above still drives
                # odom->base_link, so the drift stays observable.
                f'/{name}/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            ],
        ))
        # map → {name}/odom is NOT static: it is published by
        # ground_truth_localizer, which corrects it from the simulator's real
        # pose the way AMCL would.  Pinning it to the spawn point (as it was
        # before) left wheel-odometry drift uncorrected, so every TF pose the
        # controller and the coordinator read was dead reckoning.

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario',
            default_value='towers',
            description='Scenario: towers | rocks | maze',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 with the scenario-aligned occupancy map',
        ),
        DeclareLaunchArgument(
            'planner',
            default_value='independent',
            description='MAPF mode: independent | whca | cbs | none '
                        '(none: no per-robot navigation at all)',
        ),
        DeclareLaunchArgument(
            'show_gazebo_goals',
            default_value='false',
            description='Render goal zone boxes in Gazebo',
        ),
        OpaqueFunction(function=launch_setup),
    ])
