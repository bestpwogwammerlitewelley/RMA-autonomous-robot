# Project Handoff — RMA Autonomous Robot

Status snapshot for whoever picks this up next. Covers what works, what does
not, what is blocked, and the exact next steps.

**Goal:** a 2WD differential-drive robot that roams a room and avoids obstacles
autonomously. Raspberry Pi 5 running ROS 2 Humble in Docker for the high-level
logic; an ESP32 doing low-level motor control and sensor timing.

---

## 1. Architecture

```
  [Pi 5 / ROS 2 Humble in Docker]
        avoider          subscribes /scan  -> publishes /cmd_vel
        motor_serial_bridge   subscribes /cmd_vel -> serial "V <lin> <ang>"
        scan_serial_bridge    serial "U <d0..d4>" -> publishes /scan   [NOT BUILT]
                    |
                 USB serial
                    |
  [ESP32 DevKitC]
        parses "V ..."  -> differential kinematics -> TB6612FNG -> motors
        fires 5x HC-SR04 sequentially -> emits "U ..."           [NOT BUILT]
        reads 2x LM393 wheel encoders
```

**The core design rule:** `avoider.py` depends *only* on `/scan`
(`sensor_msgs/LaserScan`) and `/cmd_vel`. No sensor-specific or robot-specific
code. This is why the identical node runs against the Gazebo lidar in
simulation and will run against the ultrasonic array on hardware. When the
sensor is upgraded (TFmini, lidar), only the bridge that produces `/scan`
changes — the avoider is untouched.

---

## 2. What is working

| Component | Status | Where |
|-----------|--------|-------|
| Gazebo sim: roaming + obstacle avoidance | Working | `docs/sim.md` |
| Gazebo sim: SLAM mapping + Nav2 navigation | Working | `docs/sim.md` |
| `avoider.py` — sensor-agnostic avoidance node | Working, deployed | `ros2_ws/src/obstacle_avoidance/` |
| Pi deployment: clone → Docker → build → run | Working | `docs/pi-deployment.md` |
| ESP32 motor firmware (`V <lin> <ang>` → motors) | Working | `firmware/cmd_vel_bridge/` |
| `motor_serial_bridge.py` (`/cmd_vel` → serial) | Working | `ros2_ws/src/obstacle_avoidance/` |
| Full actuation chain: ROS 2 → motors | **Verified on hardware** | — |
| LM393 wheel encoders | Verified counting under motor load | `firmware/encoder_test/` |

The actuation half of the robot is done. Publishing a `Twist` to `/cmd_vel` on
the Pi turns the wheels: forward, reverse, and spin all confirmed.

---

## 3. What is NOT working / not built

**The robot cannot sense anything.** Nothing publishes `/scan`, so `avoider`
runs but idles with no data to react to. This is the only thing standing
between the current state and the project goal.

Two pieces are unwritten:

1. **ESP32 firmware** to fire the five HC-SR04 ultrasonics and emit readings.
2. **A Pi-side ROS 2 node** to parse those readings and publish `/scan`.

**First thing to confirm:** have the HC-SR04 sensors physically arrived? The
parts list had them as "TBD depending on TFmini cancellation." The level
shifter needed for them IS in hand.

---

## 4. Hardware reference

### Pin map (ESP32 DevKitC) — currently assigned

| Function | GPIO |
|----------|------|
| TB6612 PWMA / AIN1 / AIN2 | 25 / 26 / 27 |
| TB6612 PWMB / BIN1 / BIN2 | 14 / 12 / 13 |
| TB6612 STBY | 33 |
| Encoder left DO / right DO | 32 / 4 |

### Chassis measurements

| Value | Measurement |
|-------|-------------|
| Wheel base (centre to centre) | 0.13 m |
| Wheel diameter | 0.045 m |
| No-load speed | ~108 RPM at PWM 150 |
| `MAX_WHEEL_SPEED` (derived, PWM 255) | 0.43 m/s — **unverified, optimistic** |
| `MIN_PWM` (stiction floor, tuned) | 90 |
| Encoder edges per revolution | **TODO — not yet recorded** |

### Hard-won electrical lessons

* **Encoder modules run from 3V3, not 5V.** Their DO output sits at the supply
  voltage; 5V would exceed the ESP32's 3.3V pin tolerance.
* **Encoders must be on pins with internal pull-ups.** They were originally on
  GPIO 34/35 and counted nothing, despite the module LEDs toggling correctly:
  GPIO 34-39 are input-only AND have no internal pull-ups, which an
  open-collector DO output requires. Moved to 32 and 4 with `INPUT_PULLUP`.
* **`MIN_PWM` had to be raised from 60 to 90.** One motor would not overcome
  static friction at 60. Cheap gearmotors differ; closed-loop control using the
  encoders is the proper fix later.
* **HC-SR04 ECHO outputs 5V** and must go through the level shifter before
  reaching any ESP32 pin. Non-optional — it will damage the pin.

---

## 5. Serial protocol (ESP32 <-> Pi)

One line-based protocol in both directions, one message per line.

**Pi → ESP32 (motor commands, implemented):**
```
V <linear_m_s> <angular_rad_s>\n
```
The ESP32 runs a **500 ms command watchdog**: if commands stop arriving the
motors stop. This is why `motor_serial_bridge` resends at 20 Hz rather than
only on each `/cmd_vel` message — it also means the robot halts by itself if
the Pi crashes or the cable is pulled.

**ESP32 → Pi (sensor readings, NOT yet implemented):**
```
U <d0> <d1> <d2> <d3> <d4>\n     (centimetres, -1 = no echo / timeout)
```
One sweep = one line, atomic. Five separate messages would cost bandwidth and
force timestamp re-syncing.

---

## 6. Next steps, in order

### Step 1 — Settle the ultrasonic pin map (on paper, before wiring)

Five sensors need **10 GPIOs** (5 TRIG outputs, 5 ECHO inputs).

* Already used: 25, 26, 27, 14, 12, 13, 33, 32, 4
* GPIO 1 / 3 are the USB serial line — off-limits
* GPIO 34-39 are input-only: usable for ECHO, never for TRIG
* GPIO 21 / 22 are the default I2C pins for the MPU-6050 IMU (planned)

This leaves roughly **one pin short**. Cleanest release valve: remap the IMU's
I2C off 21/22, which the ESP32 permits on almost any pair.

### Step 2 — Wire ONE sensor through the level shifter, test, then replicate

Per sensor: VCC → 5V rail, GND → common ground, TRIG → ESP32 GPIO (direct, safe),
ECHO → level shifter high side, shifter low side → ESP32 GPIO. The shifter
needs both references: HV → 5V, LV → 3.3V, grounds common.

**Verify with a multimeter that the shifter low side reads ≤3.3V before
connecting any ESP32 pin.** Getting one sensor fully working proves power,
ground, trigger, and shifting before the same mistake is repeated five times.

### Step 3 — ESP32 firmware: sequential firing

Sensors must fire **one at a time, never simultaneously** — parallel pings
cross-talk, where one sensor hears another's echo. Roughly 60 ms per sensor
worst case, so a full sweep runs at about **5 Hz**. Emit one `U ...` line per
sweep. This sits alongside the existing `V ...` parsing in
`firmware/cmd_vel_bridge/`.

### Step 4 — Pi-side node publishing `/scan`

Parse the `U ...` line and publish a **single `sensor_msgs/LaserScan` with 5
beams** — *not* five `Range` topics. Five topics would break the design rule in
§1 and force a rewrite of the avoider.

Fields: `angle_min` = −90°, `angle_max` = +90°, `angle_increment` = fan/4,
`ranges` = the five values **in metres**, `range_min` = 0.02, `range_max` = 4.0,
invalid (`-1`) → `inf`, `frame_id` = `laser_frame`.

Verify with `ros2 topic echo /scan` that blocking each sensor changes the
correct element, in the correct left-to-right order.

### Step 5 — Integration

Run `avoider`, `motor_serial_bridge`, and the new scan bridge together.

**Check the message types first:**
```bash
ros2 topic info /cmd_vel --verbose
```
Publisher and subscriber must **both** show `geometry_msgs/msg/Twist`. A
mismatch causes commands to be silently dropped — the nodes look healthy and
the robot simply does not move. This has caused confusing failures twice
already.

Then: wheels-off-ground test → floor test in open space → tune
`stop_distance`, `forward_speed`, `turn_speed` on the real chassis.

---

## 7. Two loose ends worth closing

**Floor calibration has never been done.** `MAX_WHEEL_SPEED = 0.43` is derived
from a *no-load* RPM measurement and is certainly optimistic; loaded speed is
likely 0.30–0.35 m/s. Test: stream `V 0.2 0.0` for 5 s — it should cover ~1.0 m.
If short, lower `MAX_WHEEL_SPEED` and re-flash. Similarly, `angular 0.5` for
~12.6 s should be exactly one full turn; over- or under-rotating means the
0.13 m wheel base needs re-measuring.

**Encoder edges-per-revolution was never recorded.** Turn one wheel exactly one
revolution by hand and note the count increase (expect ~40 for a 20-slot disc
with `CHANGE` interrupts). That figure plus the 0.045 m wheel diameter converts
pulses to metres. Odometry work cannot start without it.

---

## 8. Expectations to set

**The 5 Hz sensor rate is much slower than the simulation's lidar.** The robot
travels meaningfully between readings, so it will react more sluggishly than
the sim suggests. Expect to reduce `forward_speed` and increase
`stop_distance` relative to the tuned sim values.

**Ultrasonics are noisy.** Soft or angled surfaces absorb or deflect pings, so
spurious `inf` readings are normal. Simple filtering may be needed if
behaviour proves erratic.

**Single-channel encoders give speed only, not direction** — the count rises
whether the wheel turns forwards or backwards. Adequate for speed feedback,
but true odometry needs direction: either infer it from the commanded motor
direction, or move to quadrature encoders. Worth deciding before August.

---

## 9. Key documents

| File | Contents |
|------|----------|
| `docs/sim.md` | Full simulation setup and reproduction guide |
| `docs/pi-deployment.md` | Getting code onto the Pi, Docker, serial device access |
| `docs/hardware.md` | Pin map, chassis measurements, serial protocol |
| `docs/setup.md` | Windows/WSL developer machine setup (not the Pi) |
| `firmware/cmd_vel_bridge/` | ESP32 motor control firmware (working) |
| `firmware/encoder_test/` | ESP32 encoder verification sketch |
