# RMA-autonomous-robot

A two-wheel robot that drives around a room on its own and turns away from
obstacles in front of it. Built from parts over a ~3-week engineering
internship at the Royal Military Academy, Brussels.

High-level logic runs on a Raspberry Pi 5 (ROS 2 Humble, in Docker). A separate
ESP32 handles real-time motor control and talks to the Pi over USB serial.
Obstacle detection uses a TFmini-S time-of-flight sensor on the front.

## Team
- Beau — hardware build and integration, ESP32 firmware and toolchain,
  Docker/Git setup, parts and coordination
- Adam — simulation (Gazebo), the avoidance node, SLAM/Nav2
- Olaf — ROS 2 and firmware, Pi-side integration

Most of it was done together; the split above is rough.

## What it does
- Drives forward and, when the TFmini detects something ahead, turns away and
  carries on (reactive obstacle avoidance on the real robot).
- The same avoidance logic was built and tested in simulation first (Gazebo),
  where it also does SLAM mapping and Nav2 navigation to a goal.
- The Pi sends velocity commands; the ESP32 turns them into motor PWM.

## How it's put together
    TFmini-S (front distance sensor)
            |
    Raspberry Pi 5  — ROS 2 Humble in Docker
       - avoidance node
       - serial bridge (/cmd_vel -> serial)
            |  USB serial
    ESP32 — motor PWM + encoder reading
            |
    TB6612 motor driver -> two TT gearmotors

For the demo the robot ran tethered to a laptop. The motors ran from a separate
AA pack through the driver, sharing a common ground with the ESP32. We tried
running the Pi standalone from a USB powerbank first, but the powerbank couldn't
reliably supply the Pi 5 (undervoltage), so fully untethered operation is left
for the next phase.

## Status
Working:
- Drives (forward, reverse, spin) under ROS 2 commands through the ESP32.
- Reactive obstacle avoidance on the real robot using the TFmini.
- Full simulation: avoidance, plus SLAM and Nav2 navigation (Gazebo, ROS 2 Jazzy).
- Wheel encoders counting.

Future work (August):
- Running untethered (sort out Pi power).
- Wider sensing than a single forward sensor.
- SLAM and mapping on the real hardware (currently sim only).
- Odometry and floor-speed calibration.

## Things that cost us time
- Sim runs on ROS 2 Jazzy (laptop), the robot on Humble (Pi). The avoidance node
  is the same on both; only the /cmd_vel message type differs, set by a
  parameter (TwistStamped in sim, plain Twist on hardware).
- Motors dead but the serial says OK -> almost always a cold solder joint or the
  TB6612 STBY pin not tied high. We lost a day to cold joints on the ESP32
  header: a thick board wicks heat away, so joints that looked finished hadn't
  bonded. Reflow hotter and longer.
- Code reaches the Pi by git pull, not file copy. Rebuild inside the Docker
  container after pulling.
- On Windows you need WSL for the ROS 2 side; the Arduino/ESP32 work runs
  natively.

## Repo layout
- firmware/   ESP32 code (motor control + serial, encoder test)
- docs/       setup, simulation, Pi deployment, hardware pin map
- Dockerfile  ROS 2 Humble base
