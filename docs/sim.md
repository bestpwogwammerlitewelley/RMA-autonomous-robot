# Simulation Environment — Setup & Reproduction Guide

Development guide for the sim environment of the RMA autonomous robot. This
document lets anyone stand up the TurtleBot3 simulation from scratch, run the
obstacle-avoidance node, and run autonomous point-to-point navigation with SLAM +
Nav2 in Gazebo (without hardware).

The simulation exists to prove the autonomy logic of the code (obstacle
avoidance, mapping, navigation), intended to be done before and independently of
the physical robot, as thought out in the project's sim-first workflow.

---

## 1. Scope & design contract

The simulated robot is a stand-in for the real 2WD differential-drive robot. It is
**not** a URDF-accurate model of our hardware, and that is intentional for this
phase — proving the control logic matters more than a matching chassis. A custom
URDF is a planned later upgrade.

The one hard design rule the sim enforces:

> The obstacle-avoidance node depends **only** on the scan topic
> (`sensor_msgs/msg/LaserScan`) and the velocity topic (`/cmd_vel`). It contains
> **no** TurtleBot3-specific or sensor-specific code.

This is what allows the same node to run unchanged on the real robot as its
rangefinder evolves (HC-SR04 ultrasonic array → TFmini-S sweep → lidar), because
each publishes the same `LaserScan` message type. The only cross-environment
difference — the `/cmd_vel` message type — is handled by a runtime parameter
(see §5.2), not by forking the code.

---

## 2. Prerequisites

| Component | Version / value | Notes |
|-----------|-----------------|-------|
| OS | Ubuntu 24.04 | |
| ROS 2 (sim) | **Jazzy** | Native install. The real robot runs Humble — see §9. |
| Simulator | Gazebo (new "Gz Sim", **not** Gazebo Classic) | Installed with `ros_gz` |
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

> **Note:** `~/.bashrc` changes only affect terminals opened *after* the edit.
> If `TURTLEBOT3_MODEL` is unset in the terminal that launches Gazebo, **no robot
> spawns** (see §8). When in doubt, `export TURTLEBOT3_MODEL=burger` explicitly
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
executes the *installed* copy, not the source file, so an un-built edit has no effect.

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

**Terminal 2 — Avoider node (sim requires the stamped flag — see §5.2)**
```bash
ros2 run obstacle_avoidance avoider --ros-args -p use_stamped_cmd_vel:=true
```

Expected: the log prints `Obstacle avoider started (cmd_vel type: TwistStamped)`
and the robot begins driving forward, stopping and turning when an obstacle enters
the front arc.

### 5.1 Node behaviour

`avoider.py` subscribes to `/scan` (`sensor_msgs/LaserScan`) and publishes to
`/cmd_vel`. Logic:

- Sectors the scan **by angle** (not fixed array indices), so it works for any
  scan resolution — the sim lidar's hundreds of beams or the hardware's 5-beam
  ultrasonic fan.
- Front cone = `front_arc` degrees either side of straight ahead; if the minimum
  valid distance there `< stop_distance`, an obstacle is ahead.
- On obstacle: compares left-sector vs right-sector clearance and turns toward the
  more open side (clearance-based turning, avoids corner oscillation).
- Otherwise: drives forward.

Tunable parameters at the top of the node: `forward_speed`, `turn_speed`,
`stop_distance`, `front_arc`, `side_arc`.

### 5.2 `/cmd_vel` message type — the `use_stamped_cmd_vel` parameter (critical)

Whether `/cmd_vel` carries `geometry_msgs/Twist` or `geometry_msgs/TwistStamped`
is a property of whatever **consumes** `/cmd_vel`, **not** of the ROS distro. The
node selects the type at launch via the `use_stamped_cmd_vel` parameter:

| Environment | Consumer of `/cmd_vel` | Required type | Parameter |
|-------------|------------------------|---------------|-----------|
| **Sim** (Jazzy) | Gazebo / TurtleBot3 bridge | `TwistStamped` | `use_stamped_cmd_vel:=true` |
| **Hardware** (Humble) | our ESP32 motor bridge | `Twist` | `use_stamped_cmd_vel:=false` *(default)* |

**Team rule: sim = TwistStamped, hardware = Twist. Any node that must work in both
takes the flag.** The default is `false` (plain `Twist`) for the real robot; in
sim you must pass `use_stamped_cmd_vel:=true`.

> If the robot does not move in sim and the log shows `cmd_vel type: Twist`, you
> forgot the flag — the Gazebo bridge is waiting for `TwistStamped` and silently
> dropping plain `Twist`. Re-run with `--ros-args -p use_stamped_cmd_vel:=true`.

### 5.3 Velocity ceiling

The TurtleBot3 burger is capped at **~0.22 m/s** in its model configuration. The
Gazebo diff-drive controller clamps any higher `linear.x` back to this limit, so
raising `forward_speed` above ~0.22 has no visible effect in sim. This cap is a
burger-model artifact and does not apply to the real robot.

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
Wait for the log line `Managed nodes are active`. Use an **absolute path** for
`params_file:=` — a leading `~` is not expanded inside launch arguments.

**Terminal 4 — RViz**
```bash
rviz2
```
Add displays via **Add → By topic**: `/map` (Map), `/scan` (LaserScan), and
`/global_costmap/costmap` (Map). Set **Fixed Frame** to `map`.

**Terminal 5 (optional) — Avoider, to build the map autonomously**
```bash
ros2 run obstacle_avoidance avoider --ros-args -p use_stamped_cmd_vel:=true
```
Let the robot roam to populate the map, then **stop the avoider** (`Ctrl+C`)
before issuing Nav2 goals — see §6.5.

### 6.3 Nav2 parameters — the `enable_stamped_cmd_vel` fix (critical)

This is the Nav2-side counterpart of §5.2. By default, Nav2's velocity publishers
(`collision_monitor` is the final publisher in the chain, plus `controller_server`,
`velocity_smoother`, `docking_server`) emit plain **`geometry_msgs/msg/Twist`**.
The Gazebo `ros_gz_bridge` subscribes as **`TwistStamped`**. The types do not
match, so the bridge silently drops every command: Nav2 plans a path, believes it
is driving, but the robot never moves and the goal terminates in **`ABORTED`**
(with `error_code: 0`).

**Fix:** a params override sets `enable_stamped_cmd_vel: true` on the relevant
Nav2 nodes so Nav2 publishes `TwistStamped`.

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
>
> Note: this Nav2 fix is **sim-only**. On the Humble robot, Nav2 (if used) drives
> the motor bridge, which expects plain `Twist`, so this override is not applied
> there. It exists purely to match the Jazzy Gazebo bridge.

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
last (turn) command has been left latched. The reliable handover is: **pause the
Gazebo sim → `Ctrl+C` the avoider → unpause → issue the Nav2 goal**. Pausing
freezes the robot in place before it can drift into a wall. (Do **not** *move* the
robot while paused — see §8.7.)

---

## 7. Known limitation — avoidance logic in tight spaces

The avoider samples a front cone and turns toward the more open side, which
handles most obstacles and simple corners. In very tight concave corners it can
still oscillate. This is why `turtlebot3_world` is the recommended test world
rather than `turtlebot3_house`.

Planned hardening (future):
- Widen `front_arc` if corner-clipping is observed.
- Add a recovery behaviour for the "boxed in on all sides" case (e.g. reverse).

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

### 8.3 Robot does not move (sim) — `/cmd_vel` type mismatch
Two independent causes, both the same root issue (see §5.2 and §6.3):

* **Avoider:** launched without `use_stamped_cmd_vel:=true`. The log shows
  `cmd_vel type: Twist`; re-run with the flag.
* **Nav2:** goal is accepted but the robot never moves and the goal ends
  `ABORTED` — the `enable_stamped_cmd_vel` params fix is not active. Confirm with:
  ```bash
  ros2 topic info /cmd_vel --verbose
  ```
  All publishers and the `ros_gz_bridge` subscriber must show
  `geometry_msgs/msg/TwistStamped`. If a publisher shows `Twist`, apply §6.3 (or
  the avoider flag) and relaunch.

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
SLAM does not observe the motion and the map desyncs. Pausing to *stop* the robot
is fine; *moving* it while paused is what breaks the map. To reset the map,
restart the SLAM Toolbox node; the map lives only in that node's memory and the
robot's position at launch becomes the new origin `(0, 0)`.

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
Raspberry Pi 5, with an ESP32 handling low-level motor control and sensor timing.

**What ports unchanged:**
- **The avoider node.** Its `rclpy` API, and the `LaserScan`/`Twist`/`TwistStamped`
  message types, are identical across Jazzy and Humble. The same `avoider.py`
  runs in both environments.

**What differs, and how it is handled:**

- **`/cmd_vel` message type.** Sim (Jazzy Gazebo bridge) needs `TwistStamped`;
  the robot (Humble, our ESP32 motor bridge) uses plain **`Twist`**. This is a
  design decision, not a distro fact — Humble's convention is plain `Twist`
  (`teleop_twist_keyboard`, Nav2, most drivers). Handled by the
  `use_stamped_cmd_vel` parameter (§5.2): default `false` for hardware, set
  `true` in sim. No code fork.

- **Sensor → `/scan`.** On the robot, the front sensor array feeds `/scan` as a
  `LaserScan`, so the avoider is a **drop-in** with no change — this is the whole
  point of the design contract (§1). The hardware sensor path:
  - **Sensors:** five HC-SR04 ultrasonics mounted across the front fan at
    **−90°, −45°, 0°, +45°, +90°**.
  - **Serial protocol (ESP32 → Pi):** one line per sweep, all five readings,
    atomic: `U <d0> <d1> <d2> <d3> <d4>\n` (centimetres, `-1` = no echo/timeout).
    One sweep = one line, to avoid bandwidth cost and timestamp re-sync.
  - **ROS side:** the Pi-side serial bridge publishes a **single
    `sensor_msgs/LaserScan` with 5 beams** (not five `Range` topics — that would
    break the design contract and force a rewrite of the avoider). Fields:
    `angle_min = −90°`, `angle_max = +90°`, `angle_increment = fan/4`, `ranges` =
    the five values in metres, `range_min = 0.02`, `range_max = 4.0`, invalid
    (`-1`) → `inf`, `frame_id` e.g. `laser_frame`. The bridge may *also* publish
    five `Range` topics for debug visibility, but `LaserScan` is the one the
    avoider consumes.
  - **Update rate:** sensors fire **sequentially, never simultaneously** (parallel
    pings cross-talk — one sensor hears another's echo). ~60 ms/sensor worst case
    → full sweep ~**5 Hz**. Adequate for avoidance.

- **Resolution.** The sim lidar gives hundreds of beams; the hardware scan gives
  five. The avoider sectors by **angle**, not fixed indices, so both work without
  change. Turn-direction quality is coarser with five beams but the logic is
  identical.

- **Velocity ceiling.** The ~0.22 m/s burger cap (§5.3) is a sim-model artifact
  and does not constrain the real robot.

**Hardware electrical constraints (for the firmware/wiring author, recorded here
for completeness):** HC-SR04 echo pins output 5 V; the ESP32 is 3.3 V-tolerant
only, so each echo line needs a divider (1 kΩ/2 kΩ) or level shifter —
non-optional. Five sensors = 10 GPIOs (trig + echo each); keep clear of the pins
already used by the TB6612 motor driver (25/26/27/12/13/14) and of
strapping/input-only pins.

---

## 10. Quick reference

| Task | Command |
|------|---------|
| Build after editing node | `cd ~/RMA-autonomous-robot/ros2_ws && colcon build --packages-select obstacle_avoidance && source install/setup.bash` |
| Launch world | `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py` |
| Run avoider (**sim**) | `ros2 run obstacle_avoidance avoider --ros-args -p use_stamped_cmd_vel:=true` |
| Run avoider (**hardware**) | `ros2 run obstacle_avoidance avoider` |
| Launch SLAM | `ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true` |
| Launch Nav2 | `ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True autostart:=True params_file:=<abs_path>/nav2_params.yaml` |
| Open RViz | `rviz2` |
| Check cmd_vel types | `ros2 topic info /cmd_vel --verbose` |
| Check transform | `ros2 run tf2_ros tf2_echo map odom` |
| Send test goal | `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.5, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}}"` |
| Kill Gazebo | `pkill -9 -f "gz sim"; pkill -9 gzserver; pkill -9 gzclient` |
