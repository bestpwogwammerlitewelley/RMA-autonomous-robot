# Simulation Environment — Setup & Reproduction Guide

Development guide for the sim environment of the RMA autonomous robot. This document lets
anyone stand up the TurtleBot3 simulation from scratch, run the
obstacle-avoidance node, and run autonomous point-to-point navigation with SLAM +
Nav2 in Gazebo (without hardware).

The simulation exists to prove the autonomy logic of the code (obstacle avoidance, mapping,
navigation), intended to be done before and independently of the physical robot, as thought out in the project's
sim-first workflow.

---

## 1. Scope & design contract

The simulated robot is a stand-in for the real 2WD differential-drive robot. It is
not a URDF-accurate model of our hardware, and that is intentional for this
phase — proving the control logic matters more than a matching chassis. A custom
URDF is a planned later upgrade.

The one hard design rule the sim enforces:

> The obstacle-avoidance node depends only on the scan topic
> (`sensor_msgs/msg/LaserScan`) and the velocity topic (`/cmd_vel`). It contains
> no TurtleBot3-specific dependencies.

This is what allows the same node to run unchanged on the real robot as its
rangefinder evolves (HC-SR04 ultrasonic → TFmini-S ToF → lidar), because each
sensor publishes the same message types.

---

## 2. Prerequisites

| Component | Version / value | Notes |
|-----------|-----------------|-------|
| OS | Ubuntu 24.04 | |
| ROS 2 | **Jazzy** | Native install. The real robot targets Humble — see §9. |
| Simulator | Gazebo (new "Gz Sim", not Gazebo Classic) | Installed with `ros_gz` |
| Robot model | TurtleBot3 **burger** | |
| Workspace | `~/RMA-autonomous-robot/ros2_ws` | Colcon workspace inside the repo |

### 2.1 Package installation

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-turtlebot3 \
  ros-jazzy-turtlebot3-simulations \
  ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-bringup \
  ros-jazzy-navigation2
```

### 2.2 Environment configuration

Add the following to `~/.bashrc` so every new terminal is correctly configured:

```bash
source /opt/ros/jazzy/setup.bash
source ~/RMA-autonomous-robot/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
```

> **Note:** `~/.bashrc` changes only affect terminals opened after the edit.
> If `TURTLEBOT3_MODEL` is unset in the terminal that launches Gazebo, no robot
> spawns (see §8). When in doubt, `export TURTLEBOT3_MODEL=burger` explicitly
> before launching.

---

## 3. Repository layout (simulation-relevant)

```
RMA-autonomous-robot/
├── docs/
│   └── sim.md                        # this file
├── ros2_ws/
│   ├── src/
│   │   └── obstacle_avoidance/       # ament_python package
│   │       ├── obstacle_avoidance/
│   │       │   └── avoider.py        # the avoidance node
│   │       ├── setup.py              # console_scripts entry point
│   │       └── package.xml
│   └── nav2_params.yaml              # Nav2 params override (see §6.3)
└── .gitignore                        # excludes build/ install/ log/
```

`build/`, `install/`, and `log/` are colcon-generated and must never be committed.

---

## 4. Building the workspace

Run from the workspace root. **Required after every edit to `avoider.py`** — `ros2 run`
executes the installed copy, not the source file, so an unbuilt edit has no effect.

```bash
cd ~/RMA-autonomous-robot/ros2_ws
colcon build --packages-select obstacle_avoidance
source install/setup.bash
```

---

## 5. Running the obstacle avoider

This is the minimal configuration: robot roams the arena and avoids walls autonomously.

**Terminal 1 — Gazebo world**
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
Press **Play** in the Gazebo GUI. The simulation must be unpaused before `/odom`
and other topics publish (see §8.2).

`turtlebot3_world` (small walled arena with cylindrical obstacles) is used rather
than `turtlebot3_house` because the current avoidance logic can become trapped in
the tight room corners of the house world. See §7 for hardening notes.

**Terminal 2 — Avoider node**
```bash
ros2 run obstacle_avoidance avoider
```

Expected: the log prints `Obstacle avoider started` and the robot begins driving
forward, stopping and rotating when an obstacle enters the front arc.

### 5.1 Node behaviour

`avoider.py` subscribes to `/scan` and publishes to `/cmd_vel`:

- Samples the front arc of the `LaserScan`, filtering invalid returns
  (`inf`, and values outside `[0.1, 3.5]` m).
- If the minimum valid front distance `< 0.4 m`: set `linear.x = 0`, `angular.z`
  to a fixed turn rate (rotate in place until clear).
- Otherwise: drive forward at the configured `linear.x`, `angular.z = 0`.

### 5.2 `/cmd_vel` message type — critical

TurtleBot3 on **Jazzy** expects **`geometry_msgs/msg/TwistStamped`** on `/cmd_vel`,
**not** plain `Twist`. The node publishes `TwistStamped`: it stamps
`cmd.header.stamp` with the current time and nests velocities under `cmd.twist.*`
(e.g. `cmd.twist.linear.x`).

Symptom if this is wrong: `ros2 topic echo /cmd_vel` fails with
`contains more than one type: [Twist, TwistStamped]`, and the robot does not move.

### 5.3 Velocity ceiling

The TurtleBot3 burger is capped at **~0.22 m/s** in its model configuration. The
Gazebo diff-drive controller clamps any higher `linear.x` back to this limit, so
raising the value in `avoider.py` above ~0.22 has no visible effect in sim. This
cap is a burger-model artifact and does not apply to the real robot.

---

## 6. Running SLAM + Nav2 (mapping and autonomous navigation)

This configuration builds a live map with SLAM Toolbox and navigates to goal
coordinates with Nav2.

> **Run SLAM and Nav2 as separate launches. Do NOT use the combined
> `nav2_bringup bringup_launch.py ... slam:=True`.** On this stack the combined
> isolated-container bringup **segfaults on startup** (`exit code -11`) while
> configuring the `route_server`, taking the entire stack down with it. Running
> the two as independent node processes avoids the crash and isolates faults.

### 6.1 Localization model

We use **SLAM Toolbox for localization**, not AMCL. SLAM Toolbox publishes the
`map → odom` transform. Consequently the RViz "Navigation 2" panel will show
**`Localization: inactive`** — this is **expected and correct** for a SLAM-based
setup (that field tracks AMCL, which we deliberately do not run). Only the
`Navigation` state matters.

### 6.2 Launch sequence

**Terminal 1 — Gazebo world**
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
Press **Play**.

**Terminal 2 — SLAM Toolbox**
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

**Terminal 3 — Nav2** (see §6.3 for the params file this depends on)
```bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=True \
  autostart:=True \
  params_file:=/home/<user>/RMA-autonomous-robot/ros2_ws/nav2_params.yaml
```
Wait for the log line `Managed nodes are active`. Use an absolute path for
`params_file:=` — a leading `~` is not expanded inside launch arguments.

**Terminal 4 — RViz**
```bash
rviz2
```
Add displays via **Add → By topic**: `/map` (Map), `/scan` (LaserScan), and
`/global_costmap/costmap` (Map). Set **Fixed Frame** to `map`.

**Terminal 5 (optional) — Avoider, to build the map autonomously**
```bash
ros2 run obstacle_avoidance avoider
```
Let the robot roam to populate the map, then stop the avoider (`Ctrl+C`)
before issuing Nav2 goals — see §6.5.

### 6.3 Nav2 parameters — the `enable_stamped_cmd_vel` fix (critical)

By default, Nav2's velocity publishers (`collision_monitor` is the final publisher
in the chain, plus `controller_server`, `velocity_smoother`, `docking_server`)
emit plain **`geometry_msgs/msg/Twist`**. The `ros_gz_bridge` subscribes as
**`TwistStamped`**. The types do not match, so the bridge silently drops every
command: Nav2 plans a path, believes it is driving, but the robot never moves and
the goal terminates in **`ABORTED`** (with `error_code: 0`).

**Fix:** a params override sets `enable_stamped_cmd_vel: true` on the relevant
nodes so Nav2 publishes `TwistStamped`.

Generate the params file (one-time):
```bash
cp /opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml \
   ~/RMA-autonomous-robot/ros2_ws/nav2_params.yaml
```

Then add `enable_stamped_cmd_vel: true` under the `ros__parameters:` block of each
of these nodes in that file (match existing indentation; if the key already exists
set it to `true` rather than duplicating):

- `controller_server`
- `collision_monitor`
- `velocity_smoother`
- `docking_server`

> Setting this parameter **live** via `ros2 param set` does **not** work — the
> publisher is created at node initialization, so the parameter must be present at
> launch. It must come from the params file.

### 6.4 Issuing navigation goals

**Via RViz:** click **2D Goal Pose** (Jazzy's renamed "2D Nav Goal"), then click
and drag on a **white (known)** area of the map — the drag direction sets the
arrival heading. Goals in grey (unknown) or black (occupied) space are rejected.

**Via CLI** (useful for verification, bypasses the RViz tool):
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.5, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}}"
```
Expected result: `Goal finished with status: SUCCEEDED` and the robot drives the
planned path.

### 6.5 Do not run the avoider and Nav2 simultaneously

Both publish to `/cmd_vel` and will fight for control, producing jitter or a robot
that will not follow the planned path. Use the avoider **only** for the mapping
run; stop it (`Ctrl+C`) before issuing Nav2 goals. Nav2 performs its own obstacle
avoidance while navigating.

If the robot spins uncontrollably immediately after the avoider is killed, the
last (turn) command has been left latched. Publish a single zero command to stop
it, or simply issue a Nav2 goal to take over:
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"
```

---

## 7. Known limitation — avoidance logic in tight spaces

The current avoider samples only a narrow front arc and always turns in a fixed
direction at a fixed rate. In tight concave corners this can oscillate (turn →
see adjacent wall → turn back), trapping the robot. This is why `turtlebot3_world`
is the recommended test world rather than `turtlebot3_house`.

Planned hardening (before hardware integration):
- Widen the sampled front arc.
- Choose turn direction by comparing left- vs right-side clearance and turning
  toward the more open side, rather than always turning the same way.

---

## 8. Troubleshooting

### 8.1 Robot does not appear in Gazebo
Almost always `TURTLEBOT3_MODEL` was unset in the Gazebo terminal.
```bash
echo $TURTLEBOT3_MODEL      # must print: burger
export TURTLEBOT3_MODEL=burger
```
Relaunch. A correct launch prints `urdf_file_name : turtlebot3_burger.urdf` and
`Entity creation successful`. Missing floor-texture errors
(`Could not resolve file [Wood_Floor_Dark.jpg]`) are cosmetic and can be ignored.

### 8.2 `/odom` (and other topics) missing from `ros2 topic list`
The `ros_gz_bridge` topics are **lazy** — they are not created until the source
publishes. `/odom` only appears once the simulation is **playing**. Press **Play**
in Gazebo, then re-check. Confirm the robot is fully up with:
```bash
ros2 topic echo /odom --once
```

### 8.3 Nav2 goal accepted but robot does not move (`ABORTED`)
`/cmd_vel` type mismatch. Confirm:
```bash
ros2 topic info /cmd_vel --verbose
```
If Nav2 publishers show `geometry_msgs/msg/Twist` while `ros_gz_bridge` subscribes
as `geometry_msgs/msg/TwistStamped`, apply the §6.3 params fix and relaunch Nav2.
After the fix, `collision_monitor` must show `TwistStamped`.

### 8.4 Nav2 container dies on startup (`exit code -11`)
Segfault in the combined isolated-container bringup at `route_server`. Use the
separate SLAM and Nav2 launches in §6.2 instead of the combined bringup.

### 8.5 RViz "Navigation 2" panel shows inactive but Nav2 log says active
The panel's status labels are unreliable and lag the true lifecycle state. If the
Nav2 log shows `Managed nodes are active`, the stack is up — issue a goal
regardless of the panel. `Localization: inactive` is always expected under SLAM
(see §6.1).

### 8.6 Verify the transform chain
For navigation to work, `map → odom` (from SLAM) must exist:
```bash
ros2 run tf2_ros tf2_echo map odom
```
A printed transform confirms SLAM is publishing and the robot is localized. A
"cannot find transform" error means SLAM Toolbox is not running.

### 8.7 SLAM map becomes corrupted / inconsistent
Do **not** reposition the robot in Gazebo while the simulation is **paused** —
SLAM does not observe the motion and the map desyncs. To reset the map, restart
the SLAM Toolbox node; the map lives only in that node's memory and the robot's
position at launch becomes the new origin `(0, 0)`.

### 8.8 Gazebo will not relaunch cleanly after Ctrl+C
Kill lingering processes before relaunching:
```bash
pkill -9 -f "gz sim"; pkill -9 gzserver; pkill -9 gzclient
```
A full stack reset also clears Nav2/SLAM/RViz:
```bash
pkill -9 -f nav2; pkill -9 -f component_container
pkill -9 -f slam_toolbox; pkill -9 -f rviz2
```
Confirm nothing remains with `ros2 node list`.

---

## 9. Sim-to-hardware porting notes

The sim runs **ROS 2 Jazzy**; the physical robot runs **ROS 2 Humble** on a
Raspberry Pi 5. Key considerations when porting:

- **Avoidance node:** ports unchanged. It depends only on `LaserScan` and
  `/cmd_vel`, whose APIs are stable across Jazzy and Humble.
- **`/cmd_vel` message type:** the `TwistStamped` requirement (§5.2) and the Nav2
  `enable_stamped_cmd_vel` fix (§6.3) are specific to the Gazebo bridge on Jazzy.
  The real robot's driver may expect plain `Twist`. **Re-verify the expected
  `/cmd_vel` type on the Pi** with `ros2 topic info /cmd_vel --verbose` before
  assuming the sim config transfers.
- **Sensor message shape:** the sim lidar publishes a multi-point 360° `LaserScan`.
  The initial hardware sensor (HC-SR04 ultrasonic) produces a **single distance
  reading** — likely a `sensor_msgs/msg/Range`, or a `LaserScan` with one element.
  The avoider's front-arc slicing assumes a multi-point scan and must be adapted
  to handle a single-value message before running on ultrasonic hardware.
- **Velocity ceiling:** the ~0.22 m/s burger cap (§5.3) is a sim-model artifact
  and does not constrain the real robot.

---

## 10. Quick reference

| Task | Command |
|------|---------|
| Build after editing node | `cd ~/RMA-autonomous-robot/ros2_ws && colcon build --packages-select obstacle_avoidance && source install/setup.bash` |
| Launch world | `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py` |
| Run avoider | `ros2 run obstacle_avoidance avoider` |
| Launch SLAM | `ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true` |
| Launch Nav2 | `ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True autostart:=True params_file:=<abs_path>/nav2_params.yaml` |
| Open RViz | `rviz2` |
| Check cmd_vel types | `ros2 topic info /cmd_vel --verbose` |
| Check transform | `ros2 run tf2_ros tf2_echo map odom` |
| Send test goal | `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.5, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}}"` |
| Kill Gazebo | `pkill -9 -f "gz sim"; pkill -9 gzserver; pkill -9 gzclient` |
