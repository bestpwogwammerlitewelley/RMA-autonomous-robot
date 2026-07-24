# Project Pause & Resume — RMA Autonomous Robot

Written at the point work paused. Read this first when picking the project back
up; it records exactly where things stopped, what physically exists, and the
precise next action.

**Goal:** a 2WD differential-drive robot that roams a room and avoids obstacles
autonomously. Raspberry Pi 5 running ROS 2 Humble in Docker for high-level
logic; an ESP32 handling motor control and sensor timing.

---

## 1. One-paragraph status

The robot **moves but cannot sense**. The entire actuation chain works and is
verified on hardware: publishing a velocity command on the Pi turns the wheels.
The obstacle-avoidance logic is written, proven in simulation, and deployed to
the Pi. Wheel encoders work. The remaining gap is the ultrasonic sensor array —
the sensors have arrived and wiring had just begun when work paused. Once
something publishes `/scan`, the existing avoidance node takes over and the
robot roams autonomously. **Nothing else is blocking the goal.**

---

## 2. Exactly where work stopped

Wiring of the **first HC-SR04 sensor (sensor 0)** onto the breadboard, via the
level shifter. The pin map was agreed (§6) but no sensor was fully wired or
tested.

**The literal next action:** wire sensor 0 (VCC, GND, TRIG, ECHO-through-shifter),
verify with a multimeter that the shifter's A1 output reads ≤3.3 V, then test
that single sensor before replicating to the other four.

Nothing was left half-connected in a dangerous state — but **check the physical
breadboard against §6 and §7 before powering anything**, since partial wiring
may be present.

---

## 3. What works (verified on hardware)

| Component | Status |
|-----------|--------|
| Gazebo sim: roaming + obstacle avoidance | Working |
| Gazebo sim: SLAM mapping + Nav2 navigation | Working |
| `avoider.py` — sensor-agnostic avoidance node | Working, deployed to Pi |
| Pi deployment: clone → Docker → build → run | Working |
| ESP32 motor firmware | Working, flashed |
| `motor_serial_bridge.py` (`/cmd_vel` → serial) | Working |
| **Full chain: ROS 2 → serial → ESP32 → motors** | **Verified** |
| LM393 wheel encoders | Verified counting under motor load |

Forward, reverse, and spin all confirmed on the real robot.

---

## 4. What does not exist yet

Two pieces of software, both unwritten:

1. **ESP32 firmware** to fire the five HC-SR04s sequentially and emit readings
   over serial.
2. **A Pi-side ROS 2 node** to parse those readings and publish `/scan`.

Plus the physical wiring of the five sensors (§7).

---

## 5. The one design rule that must not be broken

`avoider.py` depends **only** on `/scan` (`sensor_msgs/LaserScan`) and
`/cmd_vel`. It contains no sensor-specific or robot-specific code.

This is why the identical node runs against the Gazebo lidar in simulation and
will run unchanged against the ultrasonic array. When the sensor is later
upgraded (TFmini, lidar), **only the bridge producing `/scan` changes** — the
avoider is untouched.

Practical consequence: the sensor bridge must publish **one `LaserScan` with 5
beams**, *not* five separate `Range` topics. Five topics would break this rule
and force a rewrite of the avoider.

---

## 6. Agreed pin map (decided, not yet wired)

### Already in use — do not reassign

| Function | GPIO |
|----------|------|
| TB6612 PWMA / AIN1 / AIN2 | 25 / 26 / 27 |
| TB6612 PWMB / BIN1 / BIN2 | 14 / 12 / 13 |
| TB6612 STBY | 33 |
| Encoder left DO / right DO | 32 / 4 |

GPIO 1 and 3 are the USB serial line — never use.

### Ultrasonic array (agreed, pending wiring)

| Sensor | Position | TRIG | ECHO (via shifter) |
|--------|----------|------|--------------------|
| 0 | far left (−90°) | 16 | 34 |
| 1 | left (−45°) | 17 | 35 |
| 2 | centre (0°) | 18 | 36 |
| 3 | right (+45°) | 19 | 39 |
| 4 | far right (+90°) | 23 | 5 |

Reasoning: GPIO 34/35/36/39 are **input-only**, which suits ECHO perfectly and
frees general-purpose pins for TRIG. This map leaves GPIO 21/22 free, so the
MPU-6050 IMU keeps its default I2C pins later — that resolves the pin crunch
that was previously a concern.

**Caveat:** GPIO 5 is a strapping pin (sampled at boot). Fine as an ECHO input
in practice, but if the ESP32 ever fails to boot with sensors connected, that is
the first thing to suspect.

---

## 7. Wiring plan (not yet executed)

### Level shifter — HW-221 (8-channel, TXS0108E)

Three boards are on hand; one is enough for all five ECHO lines.

**B side = 5 V (sensors). A side = 3.3 V (ESP32).** B1 pairs with A1, and so on.

| Shifter pin | Connects to |
|-------------|-------------|
| VB | 5 V rail |
| VA | ESP32 3V3 |
| GND (both) | common ground rail |
| **OE** | ESP32 3V3 |

**The OE pin is easy to miss — the shifter does nothing if it is not tied
high.** If a freshly wired sensor produces no readings at all, check OE first.

### Breadboard rails

There is only one 3V3 pin on the ESP32, so run it to a breadboard power rail
and branch from there. The long strips down the board edges are internally
connected along their length.

| Rail | Fed from | Feeds |
|------|----------|-------|
| 3.3 V | ESP32 3V3 | shifter VA, shifter OE |
| 5 V | ESP32 VIN/5V pin (USB-powered) | shifter VB, all 5 sensor VCC |
| GND | ESP32 GND | shifter both GNDs, all 5 sensor GND, battery − |

### Per sensor

Each HC-SR04 has four pins: VCC, TRIG, ECHO, GND.

* VCC → 5 V rail
* GND → common ground rail
* TRIG → ESP32 GPIO **directly** (safe: 3.3 V out is read as logic-high)
* ECHO → shifter **B** channel; matching **A** channel → ESP32 GPIO

**ECHO outputs 5 V. It must never reach an ESP32 pin without passing through
the shifter — it will damage the pin.** Verify with the multimeter that the A
side reads ≤3.3 V before connecting to the ESP32.

### Method

Wire **one sensor completely and test it** before replicating. Getting sensor 0
working proves power, ground, trigger, and level shifting all at once — far
better than repeating the same mistake five times.

---

## 8. Remaining work, in order

**Step 1 — Wire and test sensor 0.** Per §7. Multimeter-verify the shifter
output before connecting the ESP32 pin.

**Step 2 — Replicate to sensors 1–4.** Test each individually as it is added,
so a fault is traceable to the sensor just wired.

**Step 3 — ESP32 firmware: sequential firing.** Sensors must fire **one at a
time, never simultaneously** — parallel pings cross-talk, where one sensor
hears another's echo. Roughly 60 ms per sensor worst case, so a full sweep runs
at about **5 Hz**. Emit one atomic line per sweep:
```
U <d0> <d1> <d2> <d3> <d4>\n      (centimetres, -1 = no echo / timeout)
```
This sits alongside the existing `V <linear> <angular>` command parsing in
`firmware/cmd_vel_bridge/`.

**Step 4 — Pi-side node publishing `/scan`.** Parse the `U ...` line, publish a
single 5-beam `sensor_msgs/LaserScan`:

| Field | Value |
|-------|-------|
| `angle_min` | −90° (in radians) |
| `angle_max` | +90° (in radians) |
| `angle_increment` | fan / 4 (i.e. 45°) |
| `ranges` | the five values **in metres** |
| `range_min` | 0.02 |
| `range_max` | 4.0 |
| invalid (`-1`) | `inf` |
| `frame_id` | `laser_frame` |

Verify with `ros2 topic echo /scan` that blocking each sensor changes the
correct element, in correct left-to-right order.

**Step 5 — Integration.** Run `avoider`, `motor_serial_bridge`, and the new
scan bridge together. **Check message types first:**
```bash
ros2 topic info /cmd_vel --verbose
```
Publisher and subscriber must **both** show `geometry_msgs/msg/Twist`. A
mismatch causes commands to be silently dropped — nodes look healthy, robot
does not move. This has caused confusing failures twice already.

Then wheels-off-ground test → floor test in open space → tune `stop_distance`,
`forward_speed`, `turn_speed` on the real chassis.

---

## 9. Two loose ends never closed

**Floor calibration was never done.** `MAX_WHEEL_SPEED = 0.43` m/s is derived
from a *no-load* RPM measurement and is certainly optimistic; real loaded speed
is likely 0.30–0.35. Test: stream `V 0.2 0.0` for 5 s — should cover ~1.0 m. If
short, lower the constant and re-flash. Similarly `angular 0.5` for ~12.6 s
should be exactly one full turn; over- or under-rotation means the 0.13 m wheel
base needs re-measuring.

**Encoder edges-per-revolution was never recorded.** Turn one wheel exactly one
revolution by hand and note the count increase (expect ~40 for a 20-slot disc
with `CHANGE` interrupts). That figure plus the 0.045 m wheel diameter converts
pulses to metres. **Odometry work cannot start without it.**

Both take ~20 minutes total and need no new parts. Good first tasks on
returning — they re-familiarise you with the toolchain before the harder
sensor work.

---

## 10. Hardware reference

| Value | Measurement |
|-------|-------------|
| Wheel base (centre to centre) | 0.13 m |
| Wheel diameter | 0.045 m |
| No-load speed | ~108 RPM at PWM 150 |
| `MAX_WHEEL_SPEED` (derived) | 0.43 m/s — **unverified** |
| `MIN_PWM` (stiction floor, tuned) | 90 |
| Encoder edges per revolution | **not recorded** |

### Hard-won electrical lessons — do not relearn these

* **Encoder modules run from 3V3, not 5 V.** Their DO output sits at the supply
  voltage; 5 V exceeds the ESP32's pin tolerance.
* **Encoders need pins with internal pull-ups.** Originally on GPIO 34/35 they
  counted nothing, despite the module LEDs toggling correctly: GPIO 34–39 are
  input-only *and* have no internal pull-ups, which an open-collector DO output
  requires. Moved to 32 and 4 with `INPUT_PULLUP`.
* **`MIN_PWM` was raised from 60 to 90.** One motor would not overcome static
  friction at 60. Cheap gearmotors differ; closed-loop control using the
  encoders is the proper fix later.
* **HC-SR04 ECHO outputs 5 V** and must pass through the level shifter.

---

## 11. Environment notes

**Simulation** runs on ROS 2 **Jazzy** (laptop). **The robot** runs ROS 2
**Humble** (Pi, in Docker). The avoidance node is portable between them; the
only difference is the `/cmd_vel` message type, handled by a runtime parameter:

* Sim (Jazzy/Gazebo): `use_stamped_cmd_vel:=true` → `TwistStamped`
* Hardware (Humble): default `false` → plain `Twist`

**Code reaches the Pi via git, never by copying files.** Push from the laptop,
`git pull` on the Pi host (not inside the container — the repo is mounted into
it). Then rebuild inside the container, because `ros2 run` executes the
*installed* copy, not the source.

**A Windows machine needs WSL** for the ROS 2 side — plain Command Prompt will
not work (`ls: command not found` is the symptom). The Arduino IDE and all
hardware work run natively on Windows without it. `wsl --install` from an
Administrator PowerShell, then reboot.

---

## 12. Key documents

| File | Contents |
|------|----------|
| `docs/handoff.md` | Fuller architectural handoff |
| `docs/sim.md` | Simulation setup and reproduction guide |
| `docs/pi-deployment.md` | Getting code onto the Pi, Docker, serial access |
| `docs/hardware.md` | Pin map, chassis measurements, serial protocol |
| `docs/setup.md` | Windows/WSL developer machine setup |
| `firmware/cmd_vel_bridge/` | ESP32 motor firmware (working, flashed) |
| `firmware/encoder_test/` | ESP32 encoder verification sketch |

---

## 13. First 30 minutes on returning

1. Read §2 and §8 of this document.
2. Physically inspect the breadboard against §6 and §7 — check what is already
   wired before powering anything.
3. Confirm the robot still drives: run the motor bridge on the Pi and publish a
   `Twist` (see `docs/pi-deployment.md`). This proves the toolchain, the Pi, and
   the ESP32 all still work before adding anything new.
4. Do the two loose ends in §9 — quick, useful, and they rebuild familiarity.
5. Then start Step 1 in §8: wire sensor 0.
