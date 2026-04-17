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
| `towers` | Cylindrical towers - open-field navigation with obstacles |
| `rocks`  | Scattered rocks - irregular terrain simulation |
| `maze`   | Two-corridor maze - MAPF conflicts in narrow spaces |

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

### Automated Goal Zone Assignment (GNFC)

Instead of manually sending individual goals, use **Greedy Nearest-Free-Cell (GNFC)** to automatically assign unique goal points within a rectangular zone. This is useful for testing multi-robot coordination.

**Two-terminal workflow:**

```bash
# Terminal 1 — launch the scenario
ros2 launch swarm_bringup scenario.launch.py scenario:=towers planner:=independent

# Terminal 2 — define goal zone and auto-assign goals
ros2 launch mapf_coordinator goal_zone.launch.py \
  x_min:=7.5 x_max:=9.0 y_min:=-1.0 y_max:=1.0 spacing:=0.3
```

**What happens:**
- GNFC allocator automatically assigns each robot a unique goal point within the zone
- Visualizer displays a **green rectangle** in RViz and Gazebo showing the goal zone boundaries
- Each robot receives its goal via `/{robot}/goal_pose` and begins navigation

**Customize the goal zone by scenario:**

Towers (open field):
```bash
ros2 launch mapf_coordinator goal_zone.launch.py \
  x_min:=7.5 x_max:=9.0 y_min:=-1.0 y_max:=1.0 spacing:=0.3
```

Rocks (tighter zone):
```bash
ros2 launch mapf_coordinator goal_zone.launch.py \
  x_min:=8.0 x_max:=8.8 y_min:=-0.8 y_max:=0.8 spacing:=0.2
```

Maze (spread out):
```bash
ros2 launch mapf_coordinator goal_zone.launch.py \
  x_min:=6.0 x_max:=8.0 y_min:=-2.0 y_max:=2.0 spacing:=0.4
```

**Parameters:**
- `x_min`, `x_max`: zone boundaries in X (metres)
- `y_min`, `y_max`: zone boundaries in Y (metres)
- `spacing`: grid spacing between candidate goal points (metres)

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
│   ├── navig_msgs/            # Custom ROS messages/services
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
