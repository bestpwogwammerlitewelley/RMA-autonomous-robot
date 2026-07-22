# Raspberry Pi Deployment Guide

How to get the ROS 2 nodes from the repo onto the robot's Raspberry Pi 5 and
running under ROS 2 Humble in Docker.

This is distinct from `docs/setup.md`, which covers the **Windows/WSL developer
machine** (Docker Desktop, Arduino IDE, ESP32 toolchain). This document covers the
**robot itself**.

---

## 1. Deployment model

**Code is not copied to the Pi — the Pi pulls it from GitHub.**

```
laptop  --git push-->  GitHub  --git pull-->  Raspberry Pi  --volume mount-->  Docker container
```

The repo is cloned to the Pi's filesystem and **mounted into** the ROS 2 container
as a volume. This means a `git pull` on the Pi is immediately visible inside the
container — no rebuilding the image, no copying files by hand.

Never `scp` or manually copy source files to the Pi. That causes the Pi and the
repo to drift out of sync, and the running code stops matching what is committed.

`build/`, `install/`, and `log/` are **never** transferred. They are gitignored
and machine/architecture-specific — the laptop is x86, the Pi is ARM. The
workspace is always built **on the Pi**.

---

## 2. Prerequisites

| Component | Value | Notes |
|-----------|-------|-------|
| Hardware | Raspberry Pi 5 (8 GB) | |
| ROS 2 | **Humble**, inside Docker | Not installed natively — `which ros2` on the Pi host returns nothing |
| Docker | 29.x on the Pi | Already installed |
| Image | `rma-robot` | Built from the repo's `Dockerfile` (currently just `FROM ros:humble`) |
| Repo location on Pi | `~/RMA-autonomous-robot` | |

---

## 3. Connecting to the Pi

### 3.1 Finding the Pi's IP

> **The Pi 5's USB-C port will not work for this.** Unlike the Pi 4 / Pi Zero, it
> does not support USB ethernet gadget mode — connecting USB-C to a laptop
> provides power, not a network interface. Use one of the methods below.

Options, in order of convenience:

* **Hostname (mDNS)** — if the Pi is on the same network, skip the IP entirely:
  `ssh <user>@raspberrypi.local` (substitute the actual hostname if changed).
* **On the Pi directly** — with a monitor (micro-HDMI) and keyboard attached:
  ```bash
  hostname -I
  ```
* **Scan the network** from a machine on the same LAN:
  ```bash
  nmap -sn 192.168.1.0/24        # adjust subnet to match your network
  ```
* **Router admin page** — list of connected devices, find by hostname.

If none of these find it, the Pi likely has **no network configuration at all**
(fresh image with no WiFi credentials / SSH not enabled). In that case, re-image
or edit the boot partition config with Raspberry Pi Imager to preset WiFi and
enable SSH.

### 3.2 SSH in

```bash
ssh <user>@<pi-ip-address>
```

On first connection, accept the host fingerprint (`yes`). The password prompt
shows no characters as you type — this is normal.

Common failures:

| Error | Meaning |
|-------|---------|
| `Connection refused` | SSH not enabled on the Pi — enable via `sudo raspi-config` → Interface Options → SSH, or during imaging |
| `Connection timed out` / `No route to host` | Not reachable at that IP — check `ping <ip>`, different subnet, or DHCP reassigned the address |
| `Permission denied` | Wrong username or password |

---

## 4. Getting the code onto the Pi

**First time only** — clone:
```bash
cd ~
git clone https://github.com/bestpwogwammerlitewelley/RMA-autonomous-robot.git
```

**Every time after** — pull:
```bash
cd ~/RMA-autonomous-robot
git pull origin main
git log --oneline -3        # confirm the expected commit arrived
```

Pushing from GitHub requires a **personal access token** as the password (GitHub
disabled password auth).

---

## 5. Starting the ROS 2 container

### 5.1 First run — create the container

```bash
docker run -it --name rma \
  --net=host \
  -v ~/RMA-autonomous-robot:/workspace \
  rma-robot bash
```

You will land at a `root@rccar:/#` prompt — you are now **inside** the container.

Why each flag matters:

* `--net=host` — ROS 2 uses DDS for node discovery. Without host networking,
  nodes inside the container cannot discover nodes outside it (or on other
  machines). **Required.**
* `-v ~/RMA-autonomous-robot:/workspace` — mounts the repo live at `/workspace`
  inside the container. This is what makes `git pull` on the Pi instantly visible
  to the container.
* `--name rma` — names the container so it can be restarted later.

**When the ESP32 is connected**, the serial device must also be passed through so
the bridge node can reach it:
```bash
  --device=/dev/ttyUSB0
```
(confirm the actual device path with `ls /dev/tty*` on the Pi with the ESP32
plugged in — it may be `/dev/ttyUSB0` or `/dev/ttyACM0`).

### 5.2 Subsequent runs — reuse the existing container

Do **not** `docker run` again (it will fail on the duplicate name, or create a
second container).

```bash
docker start -ai rma              # restart and attach
docker exec -it rma bash          # open an ADDITIONAL shell in a running container
```

Use `docker exec` when you need a second/third terminal inside the same container
(e.g. one running a node, another running `ros2 topic echo`).

---

## 6. Building and running

Inside the container, **every new shell** must source ROS 2 first:

```bash
source /opt/ros/humble/setup.bash
cd /workspace/ros2_ws
colcon build --packages-select obstacle_avoidance
source install/setup.bash
```

Run the avoider:
```bash
ros2 run obstacle_avoidance avoider
```

**Expected output:**
```
[INFO] [...] [obstacle_avoider]: Obstacle avoider started (cmd_vel type: Twist)
```

`cmd_vel type: Twist` is correct for hardware — no parameter needed, because the
node's default is plain `Twist` for the Humble motor bridge. (`TwistStamped` is
sim-only; see `docs/sim.md` §5.2.)

> **If the log reads only `Obstacle avoider started` with no `(cmd_vel type: ...)`,
> the Pi is running a STALE version of the node.** Exit, `git pull` on the Pi,
> re-run `colcon build`, and try again. `ros2 run` executes the *installed* copy,
> so a pull without a rebuild has no effect.

---

## 7. Verification

With the node running, open a second shell (`docker exec -it rma bash`, then
`source /opt/ros/humble/setup.bash`) and check:

```bash
ros2 node list                    # expect /obstacle_avoider
ros2 topic info /scan             # confirms the node is subscribed
ros2 topic info /cmd_vel --verbose
```

**The `/cmd_vel` check is the critical one at integration time.** The avoider must
publish `geometry_msgs/msg/Twist` and the motor bridge must subscribe as
`geometry_msgs/msg/Twist`. A type mismatch causes commands to be silently dropped
— the node appears healthy, but the robot does not move. This exact failure mode
has occurred twice in simulation; check it first, not last.

Once the serial bridge exists, confirm sensor data is arriving:
```bash
ros2 topic echo /scan
```
Expect a 5-element `LaserScan` whose values change when obstacles are placed in
front of individual sensors.

**Test with the wheels off the ground** (robot on blocks) before letting it drive
on the floor.

---

## 8. Current integration status

The avoidance node is deployed and verified to build and run on the Pi (ARM /
Humble / Docker). It is **not yet functional end-to-end**, pending two components
owned elsewhere:

| Dependency | Status | Effect if missing |
|------------|--------|-------------------|
| Serial bridge publishing `/scan` | Not yet written | Avoider runs but idles — nothing publishes the topic it subscribes to |
| ESP32 firmware consuming `/cmd_vel` → TB6612 | In progress | Avoider publishes commands into the void |

Integration order: motors respond to `/cmd_vel` → bridge publishes `/scan` → the
avoider ties them together and the robot roams autonomously.

---

## 9. Known gaps / maintenance notes

* **The `Dockerfile` is bare (`FROM ros:humble`).** Any package installed inside
  the container with `apt install` is lost if the container is deleted. Anything
  that turns out to be required should be added to the `Dockerfile` and the image
  rebuilt, so the environment is reproducible for the team.
* **Prompt awareness.** `adamfoster@adamfoster-HP-...` = the laptop.
  `<user>@<pi-hostname>` = the Pi host shell. `root@rccar` = inside the container.
  Running container commands (`cd /workspace/...`) on the laptop fails with
  "No such file or directory", and running `colcon build` from the wrong directory
  scatters `build/`/`install/`/`log/` folders outside the gitignored paths.
* **`.gitignore` only covers `ros2_ws/build|install|log`.** A stray `colcon build`
  from the repo root creates untracked build folders at the top level. Remove them
  with `rm -rf build install log` from the repo root.

---

## 10. Quick reference

| Task | Command | Where |
|------|---------|-------|
| SSH to Pi | `ssh <user>@<pi-ip>` | Laptop |
| Update code | `cd ~/RMA-autonomous-robot && git pull origin main` | Pi host |
| Create container (first time) | `docker run -it --name rma --net=host -v ~/RMA-autonomous-robot:/workspace rma-robot bash` | Pi host |
| Restart container | `docker start -ai rma` | Pi host |
| Extra shell in container | `docker exec -it rma bash` | Pi host |
| Source ROS 2 (every new shell) | `source /opt/ros/humble/setup.bash` | Container |
| Build | `cd /workspace/ros2_ws && colcon build --packages-select obstacle_avoidance && source install/setup.bash` | Container |
| Run avoider (hardware) | `ros2 run obstacle_avoidance avoider` | Container |
| Check cmd_vel types | `ros2 topic info /cmd_vel --verbose` | Container |
| Check sensor data | `ros2 topic echo /scan` | Container |
