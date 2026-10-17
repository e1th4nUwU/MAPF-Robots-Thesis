# Collaborative Robotic Swarm — Bachelor's Thesis

Fork of the **"Mobile Robots 2026-2"** course repository
(https://github.com/mnegretev/Mobile-Robots-2026-2) from the School of Engineering, UNAM,
adapted for a thesis project on **Multi-Agent Path Finding (MAPF) with ground robots**.

## Description

This project implements a collaborative robotic swarm with multiple differential-drive robots
in simulation (ROS 2 Jazzy + Gazebo Harmonic). Robots must coordinate to reach goal zones
while avoiding collisions with both obstacles and other robots.

### Test Scenarios

| Scenario | Description |
|---|---|
| `towers` | 4-row slalom of cylindrical towers — open-field navigation with conflicts at narrow gaps |
| `rocks`  | Scattered boulders + rotated rock boxes — irregular terrain that forces path interleaving |
| `maze`   | Three-corridor maze with central convergence zone — robots from N/M/S corridors must funnel through 2 m gaps and converge at a single goal area (prime MAPF stress test) |

### Fixed Goal Zones per Scenario

Each scenario has a **deterministic goal area** (the green rectangle in RViz/Gazebo).
When using `planner:=whca` or `planner:=cbs`, the legacy **GNFC allocator** auto-launches
with that zone and each robot's spawn position, so goal assignment is reproducible
across runs — essential for testing WHCA*/CBS behaviour.

| Scenario | Goal Zone (x_min, x_max, y_min, y_max) | Spacing | Notes |
|---|---|---|---|
| `towers` | (7.5, 9.0, -1.5, 1.5) | 0.8 m | Wide vertical strip on east side |
| `rocks`  | (8.0, 9.0, -1.5, 1.5) | 0.8 m | Shifted east (x_min=8.0) to clear rock-box inflation at x≈7.7 |
| `maze`   | (7.5, 9.0, -1.0, 0.6) | 0.8 m | y_max=0.6 keeps goals below divider inflation (y=1.4 blocked with 0.6 m inflation) |

## Stack

- **ROS 2** Jazzy Jalisco
- **Gazebo** Harmonic (gz-sim 8)
- **Ubuntu** 24.04

## Requirements

- Ubuntu 24.04
- Internet access and `sudo` permissions
- ROS 2 Jazzy and Gazebo Harmonic

## Installation

```bash
sudo apt update
sudo apt install -y git
git clone <url>
cd Mobile-Robots-2026-2
chmod +x Setup.sh
./Setup.sh
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
```

`Setup.sh` is intended for a clean Ubuntu 24.04 setup. It:

- configures the official ROS 2 Jazzy repository
- installs ROS 2 Desktop, Gazebo Harmonic, and project dependencies
- runs `rosdep install` on `ros2_ws/src`
- builds the workspace with `colcon build`

If `rosdep` was already initialized on your machine, the script reuses it.

### Goal Zone Visualization

Goal zones can be visualized in **both RViz and Gazebo**:

**RViz visualization** (always available):
- Green rectangle outlines automatically publish to the `/goal_zone_marker` topic
- Use the visualizer-only launcher to show zones without assigning goals (see below)

**Gazebo visualization** (optional):
- Add `show_gazebo_goals:=true` to render green goal zone boxes in the simulation
- Example: `ros2 launch swarm_bringup scenario.launch.py scenario:=towers show_gazebo_goals:=true`
- Goal zones appear as semi-transparent green boxes at the arena's east side

## Quick Verification

```bash
ros2 pkg list | grep swarm_bringup
ros2 launch swarm_bringup scenario.launch.py scenario:=towers
```

## Usage

### Launch a Scenario

```bash
ros2 launch swarm_bringup scenario.launch.py scenario:=towers
ros2 launch swarm_bringup scenario.launch.py scenario:=rocks
ros2 launch swarm_bringup scenario.launch.py scenario:=maze
```

The scenario launch also starts:

- **shared infrastructure**: `map_server`, `cost_map` (one instance for all robots)
- **per-robot navigation stack**: `a_star`, `path_smoothing`, `pure_pursuit` — each robot runs its own independent instance under its namespace (`/alvin/`, `/teodoro/`, `/simon/`)
- teleoperation GUI (`swarm_teleop_gui.py`)
- RViz2 (optional, enabled by default) with a map aligned to the selected scenario

Optional parameters:

```bash
# Hide RViz window
ros2 launch swarm_bringup scenario.launch.py scenario:=towers use_rviz:=false

# Show goal zone boxes in Gazebo (green semi-transparent areas)
ros2 launch swarm_bringup scenario.launch.py scenario:=towers show_gazebo_goals:=true

# Combine options
ros2 launch swarm_bringup scenario.launch.py scenario:=maze show_gazebo_goals:=true planner:=whca
```

### Testing WHCA* coordination (rocks / maze)

When you pass `planner:=whca`, the scenario launch **automatically** starts:

- `priority_manager` (publishes `/robot_priorities`)
- `whca_coordinator` (debounced replans, staggered by priority)
- One `whca_star` per robot (space-time A* serving `path_planning/plan_path`)
- `gnfc_goal_allocator` configured with the scenario's fixed goal zone **and each robot's spawn position** (so the first allocation tick is deterministic — no `(0, 0)` tie at startup)
- `goal_zone_visualizer` (green rectangle in RViz / Gazebo)

**Recommended test commands:**

```bash
# Rocks — irregular terrain, plenty of natural conflicts
ros2 launch swarm_bringup scenario.launch.py scenario:=rocks planner:=whca show_gazebo_goals:=true

# Maze — three corridors converging at a single goal zone in the middle
ros2 launch swarm_bringup scenario.launch.py scenario:=maze planner:=whca show_gazebo_goals:=true

# Baseline (no coordination) for comparison
ros2 launch swarm_bringup scenario.launch.py scenario:=rocks planner:=independent
ros2 launch swarm_bringup scenario.launch.py scenario:=maze planner:=independent
```

What each scenario stresses:

- **rocks** — robots all start spread on the west (y ≈ ±1.5, 0) and converge on the east-side goal strip.  Scattered rocks block straight paths and force interleaving in the middle of the arena.
- **maze** — alvin starts in the north corridor (y=+4), teodoro in the middle (y=0), simon in the south (y=-4).  The goal is a single zone inside the middle corridor on the east, so alvin and simon must descend / ascend through 2 m central gaps (x ≈ 0) and merge with teodoro on the way east.  Three chicanes per corridor add a slalom on top.

### Send Navigation Goals

Each robot has its own namespaced `goal_pose` topic. After launching a scenario:

```bash
ros2 topic pub --once /alvin/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 8.0, y: 1.5}}}"

ros2 topic pub --once /teodoro/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 8.0, y: 0.0}}}"

ros2 topic pub --once /simon/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 8.0, y: -1.5}}}"
```

RViz shows each robot's planned path in its color (red = alvin, green = teodoro, blue = simon).

### Automated Goal Zone Assignment (GNFC, legacy)

**Greedy Nearest-Free-Cell (GNFC)** assigns each robot a unique goal point within a
rectangular zone: the nearest unassigned free cell wins. It is the allocator the WHCA*
runs have used so far, and it is being superseded by the `swarm_mapf` service below.

With `planner:=whca` or `planner:=cbs`, GNFC is **launched automatically** with the scenario's fixed goal zone and each robot's spawn position — no extra terminal needed.

For `planner:=independent` (or to override the zone), run GNFC manually in a second terminal:

```bash
# Terminal 1 — launch the scenario
ros2 launch swarm_bringup scenario.launch.py scenario:=towers planner:=independent

# Terminal 2 — define goal zone and auto-assign goals
ros2 launch mapf_coordinator goal_zone.launch.py \
  x_min:=7.5 x_max:=9.0 y_min:=-1.0 y_max:=1.0 spacing:=0.8
```

**What happens:**
- GNFC allocator assigns each robot a unique goal point within the zone
- Visualizer displays a **green rectangle** in RViz and Gazebo showing the goal zone boundaries
- Each robot receives its goal via `/{robot}/goal_pose` and begins navigation

**Why spawn positions matter:** every robot's `/odom` is published in its own local frame whose origin is the spawn point.  Without seeding the allocator with each robot's spawn pose, the first allocation tick (which can fire before any `/odom` message arrives) sees all robots at `(0, 0)` and assigns goals by tie-break, producing different paths every launch.  The scenario-wide auto-launch supplies these as `initial_x_<robot>` / `initial_y_<robot>` parameters.

**Parameters:**
- `x_min`, `x_max`: zone boundaries in X (metres)
- `y_min`, `y_max`: zone boundaries in Y (metres)
- `spacing`: grid spacing between candidate goal points (metres)

### Goal Allocation Service (`swarm_mapf`)

The allocator is being rebuilt as a **service** instead of a node that publishes goals on a
timer. `swarm_mapf/goal_allocator` answers `~/get_goal_points`, so whoever decides to send
the swarm somewhere asks for the goals and gets them back, with the reason when it fails:

```
navig_msgs/srv/GetGoalPoints
  geometry_msgs/Polygon      region          # goal region, any polygon
  nav_msgs/OccupancyGrid     map             # already inflated
  geometry_msgs/Point32[]    robots          # in priority order
  float32                    min_separation  # metres between goals
  ---
  bool                       success
  string                     error_message
  geometry_msgs/Point32[]    goals            # one per robot, same order
```

Candidates are the points of a lattice of pitch `min_separation`, centred in the bounding
box of the region, that fall inside the polygon and on a free cell. Taking the pitch equal
to the separation is what makes a pairwise separation check unnecessary.

**Allocators** live in `swarm_mapf/allocators/` behind a registry, one function each, so a
new strategy is a new file plus one line:

| Name | Status | What it does |
|---|---|---|
| `nearest` | implemented | Closest free candidate per robot, in priority order. The Chapter 6 baseline |
| `grpa` | pending | Greedy Reachability-Preserving Assignment: picks goals that do not cut off the robots still unassigned |

**Visual test bench** — scenario, RViz and a one-shot client that builds the request out of
the running system (inflated map from `/get_inflated_map`, robot poses from tf):

```bash
ros2 launch swarm_mapf allocator_test.launch.py scenario:=maze
```

RViz shows the goal region as a cyan polygon, every admissible candidate as a yellow cube,
and the assigned goals as spheres in each robot's colour. When the answer is a failure, the
region and the candidates are published anyway — seeing them is the whole diagnosis.
Path planning is off by default here (`planner:=none`).

---

### Visualizing Goal Zones (without goal assignment)

To visualize a goal zone **without automatically assigning goals** to robots, use the visualizer-only launcher:

```bash
# Terminal 1 — launch the scenario
ros2 launch swarm_bringup scenario.launch.py scenario:=towers

# Terminal 2 — visualize goal zone (RViz + Gazebo)
ros2 launch mapf_coordinator visualizer_only.launch.py \
  x_min:=7.5 x_max:=9.0 y_min:=-1.0 y_max:=1.0
```

This displays a **green rectangle** showing the goal zone boundaries in both RViz and Gazebo (if `ros-jazzy-gazebo-msgs` is installed), without triggering automatic goal assignment to robots.

### Testing Pure Pursuit (Single Robot)

To debug or test one robot in isolation without launching the full swarm:

```bash
# Terminal 1 — full simulation + shared infrastructure
ros2 launch swarm_bringup scenario.launch.py scenario:=towers

# Terminal 2 — navigation stack for one robot only
ros2 launch motion_planning robot_nav.launch.py robot_name:=alvin

# Terminal 3 — send a goal
ros2 topic pub --once /alvin/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 8.0, y: 1.5, z: 0.0}}}"
```

Tune the controller gains in [robot_nav.launch.py](ros2_ws/src/navigation/motion_planning/launch/robot_nav.launch.py) (`alpha`, `beta`) if the robot is too jerky or too slow to turn.

### Per-Robot Navigation Topics

| Topic | Description |
|---|---|
| `/{robot}/goal_pose` | Send a navigation goal |
| `/{robot}/path_planning/plan_path` | A* service |
| `/{robot}/path_planning/path` | Planned path (visualized in RViz) |
| `/{robot}/cmd_vel` | Velocity commands to Gazebo |

### Remote Control + Monitoring

```bash
ros2 run swarm_bringup swarm_teleop_gui.py
```

Note: this command is still available if you want to launch the GUI manually,
but the scenario launch now starts it automatically.

The GUI shows real-time position `(x, y, theta)`, velocities `(vx, omega)`, and heading
for all 3 robots, with a D-pad for manual control of each robot.

## Repository Structure

```
ros2_ws/src/
├── hardware/
│   └── justina_description/   # Justina differential robot URDF
│       └── urdf/justina_base.xacro  # odom_frame/base_frame args for namespaced TF
├── navigation/
│   ├── motion_planning/       # Motion-planning launch files
│   │   └── launch/
│   │       ├── motion_planning.launch.py  # Shared infra: map_server + cost_map
│   │       └── robot_nav.launch.py        # Per-robot: a_star + path_smoothing + pure_pursuit
│   ├── mapf_coordinator/      # WHCA*/CBS coordination + legacy GNFC allocator
│   ├── navig_msgs/            # Custom ROS messages/services
│   ├── swarm_mapf/            # Goal allocation service (GetGoalPoints)
│   │   ├── goal_allocator.py  # The service node
│   │   ├── grid.py            # world <-> cell conversions
│   │   ├── region.py          # ray casting + candidate lattice
│   │   └── allocators/        # One file per strategy (nearest, grpa)
│   ├── path_follower/         # Trajectory follower (Pure Pursuit)
│   │   └── pure_pursuit.py    # robot_frame param for namespaced TF lookup
│   └── path_planner/          # Path planners (A*, RRT, Potential Fields)
└── swarm_bringup/             # Main thesis package
    ├── launch/
    │   ├── scenario.launch.py # Launch any of the 3 scenarios + full nav stack
    │   └── swarm.launch.py    # 3 robots in an empty world (dev/debug)
    └── scripts/
        ├── swarm_teleop_gui.py # Control + monitoring GUI
        └── swarm_monitor.py    # Console monitor
```

---

## Credits

This repository is a fork of work by **Dr. Marco Negrete**:

> Dr. Marco Negrete
> Full Professor A - Head of the Signal Processing Department
> School of Engineering, UNAM
> marco.negrete@ingenieria.unam.edu
> https://mnegretev.info

Original repository: https://github.com/mnegretev/Mobile-Robots-2026-2
