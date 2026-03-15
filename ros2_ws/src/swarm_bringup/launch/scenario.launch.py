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

maze    Two-corridor maze (sandstone walls, orange chicane barriers).
        Top corridor (y>0): barrier at x=2 forces north detour past y=3.5.
        Bottom corridor (y<0): barrier at x=-2 forces south detour past y=-3.5.

All scenarios have a closed perimeter (grey-blue boundary walls) so robots
cannot escape the arena.

All robots:
  alvin — red   teodoro — green   simon — blue
  Start at x=-8 (west side), goal zone (green) at x=8 (east side).
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
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    # Two robots in top corridor, one in bottom
    {'name': 'alvin',   'color': '0.9 0.1 0.1 1.0', 'x': -8.0, 'y':  2.5, 'z': 0.5, 'yaw': 0.0},
    {'name': 'simon',   'color': '0.1 0.1 0.9 1.0', 'x': -8.0, 'y':  1.0, 'z': 0.5, 'yaw': 0.0},
    {'name': 'teodoro', 'color': '0.1 0.8 0.1 1.0', 'x': -8.0, 'y': -2.5, 'z': 0.5, 'yaw': 0.0},
]

SCENARIO_ROBOTS = {
    'towers': _ROBOTS_OPEN,
    'rocks':  _ROBOTS_OPEN,
    'maze':   _ROBOTS_MAZE,
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
    """Flat bright-green rectangle on the ground indicating the goal zone."""
    return _box(name, cx, cy, 0.05, lx, ly, 0.1, color=C_GOAL)


# ── Scenario geometry ──────────────────────────────────────────────────────────

def _build_towers():
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
    parts += _boundary_walls()
    parts.append(_goal_marker(cy=0.0, ly=ARENA_Y * 2))
    return '\n'.join(parts)


def _build_rocks():
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
    parts += _boundary_walls()
    parts.append(_goal_marker(cy=0.0, ly=ARENA_Y * 2))
    return '\n'.join(parts)


def _build_maze():
    """Two-corridor maze: sandstone structural walls, orange chicane barriers.

    Layout (top view):

        y=+6  ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓  boundary (grey-blue)
        y=+5  ════════════════════════  north corridor wall (sandstone)
              [alvin, simon →]  ┃ orange barrier x=+2, y=0..3.5
                                ┃ gap y=3.5..5 → detour north
        y= 0  ════════════════════════  central divider (sandstone)
              ┃ orange barrier x=-2, y=-3.5..0
              ┃ gap y=-5..-3.5 → detour south
        y=-5  ════════════════════════  south corridor wall (sandstone)
        y=-6  ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓  boundary (grey-blue)
              x=-10             x=+10

    Robots in the top corridor (alvin, simon) must squeeze through the
    1.5 m gap north of the orange barrier — prime MAPF conflict zone.
    Teodoro in the bottom corridor has a symmetric challenge at x=-2.
    """
    W = ARENA_X   # 10
    h = WALL_H    # 1.5
    t = WALL_T    # 0.2
    span = W * 2  # 20

    parts = [
        # ── Structural sandstone walls (corridor boundaries + divider) ──
        _box('wall_north',   0.0,  5.0, h/2, span, t, h, color=C_MAZE_WALL),
        _box('wall_south',   0.0, -5.0, h/2, span, t, h, color=C_MAZE_WALL),
        _box('wall_center',  0.0,  0.0, h/2, span, t, h, color=C_MAZE_WALL),

        # ── Orange chicane barriers ──
        # Top barrier: x=+2, y=0 to y=+3.5  (gap: y=3.5..5, width=1.5 m)
        _box('bar_top',  2.0,  1.75, h/2, t, 3.5, h, color=C_MAZE_BAR),
        # Bottom barrier: x=-2, y=-3.5 to y=0  (gap: y=-5..-3.5, width=1.5 m)
        _box('bar_bot', -2.0, -1.75, h/2, t, 3.5, h, color=C_MAZE_BAR),

        # ── Grey-blue perimeter (outer boundary, outside corridors) ──
        _box('bnd_west', -W, 0.0, h/2, t, ARENA_Y*2, h, color=C_BOUNDARY),
        _box('bnd_east',  W, 0.0, h/2, t, ARENA_Y*2, h, color=C_BOUNDARY),
        _box('bnd_north_out', 0.0,  ARENA_Y, h/2, span+t, t, h, color=C_BOUNDARY),
        _box('bnd_south_out', 0.0, -ARENA_Y, h/2, span+t, t, h, color=C_BOUNDARY),

        # ── Goal zones (one per corridor) ──
        _goal_marker('goal_top', cx=8.5, cy= 2.5, lx=1.0, ly=4.5),
        _goal_marker('goal_bot', cx=8.5, cy=-2.5, lx=1.0, ly=4.5),
    ]
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

    elif scenario == 'maze':
        _paint_rect(grid, 0.0, 5.0, ARENA_X * 2.0, WALL_T)
        _paint_rect(grid, 0.0, -5.0, ARENA_X * 2.0, WALL_T)
        _paint_rect(grid, 0.0, 0.0, ARENA_X * 2.0, WALL_T)
        _paint_rect(grid, 2.0, 1.75, WALL_T, 3.5)
        _paint_rect(grid, -2.0, -1.75, WALL_T, 3.5)

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
            },
        ).toxml()
        robots_urdf.append(urdf)
        robots_sdf.append(_robot_to_sdf(urdf, robot))

    world_path = _build_world(robots_sdf, SCENARIO_GEOMETRY[scenario]())
    map_yaml_path = _write_scenario_map_files(scenario)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py',
            ])
        ),
        launch_arguments={'gz_args': f'-r -v 3 {world_path}'}.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

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

    nodes = [
        LogInfo(msg=f"Launching scenario: {scenario}"),
        LogInfo(msg=f"Using generated map yaml: {map_yaml_path}"),
        gazebo,
        clock_bridge,
        planning,
        teleop_gui,
        viz_bridge,
    ]

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
            ],
        ))

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
        OpaqueFunction(function=launch_setup),
    ])
