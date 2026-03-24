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

To run without RViz:

```bash
ros2 launch swarm_bringup scenario.launch.py scenario:=towers use_rviz:=false
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
