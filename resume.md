# Project pause-and-resume — RMA autonomous robot

Written at the point work paused for the summer. Read this first when picking the
project back up: it records where things got to, what physically exists, and the
sensible next steps.

Goal: a two-wheel differential-drive robot that roams a room and avoids obstacles on
its own. A Raspberry Pi 5 runs the high-level logic (ROS 2 Humble, in Docker); a
separate ESP32 handles motor control and serial timing.

## 1. Where we got to

The robot drives and avoids obstacles. The whole actuation chain works on hardware:
a velocity command on the Pi turns the wheels, and forward, reverse and spin are all
confirmed. A TFmini-S time-of-flight sensor on the front gives reactive avoidance: it
drives forward, and when something is close ahead it turns away and carries on. This
was shown with the robot tethered to a laptop; the Pi was not yet running off a
battery (see section 4 on power).

The avoidance logic was built and proven in simulation first, where it also does SLAM
mapping and Nav2 navigation to a goal. The wheel encoders are wired and counting.

Left for August: running untethered, richer sensing than a single forward beam,
SLAM/mapping on the real hardware (simulation only so far), and odometry/speed
calibration.

One gap to fill in when you are back at the bench: write down exactly how the TFmini
is wired and how its reading reaches the avoidance logic. It is not recorded anywhere
yet.

## 2. What works, verified on hardware

| Component | Status |
|-----------|--------|
| Gazebo sim: roaming + obstacle avoidance | Working |
| Gazebo sim: SLAM mapping + Nav2 navigation | Working |
| Avoidance node (sensor-agnostic) | Working, deployed to the Pi |
| Pi deployment: clone, Docker, build, run | Working |
| ESP32 motor firmware | Working, flashed |
| cmd_vel to serial bridge on the Pi | Working |
| Full chain: ROS 2 to serial to ESP32 to motors | Verified |
| TFmini reactive avoidance on the real robot | Working (tethered) |
| LM393 wheel encoders | Counting under motor load |

## 3. The one design rule that matters

The avoidance node depends only on a LaserScan input and a velocity output. It has no
sensor-specific or robot-specific code. That is why the same node runs against the
Gazebo lidar in simulation. Any sensor change (ultrasonic, TFmini, lidar) should only
change the small bridge that produces the scan input; the avoider stays untouched.

Practical consequence: a multi-sensor bridge should publish one LaserScan with several
beams, not one range topic per sensor. Separate topics would break this and force a
rewrite of the avoider.

## 4. Power

For the demo the Pi ran tethered (USB/wall power) and the motors ran from a separate
AA pack through the TB6612 driver, sharing a common ground with the ESP32. We tried a
USB powerbank for the Pi first, but it could not hold the Pi 5 up under load
(undervoltage, red light).

The proper untethered option is the 2S LiPo with a buck converter for the Pi, which
was not finished. Before connecting it: set the buck output to about 5 V and check it
with a multimeter, with the Pi disconnected. An unverified buck can put far more than
5 V onto the Pi and kill it. The motors can sit on the raw battery rail safely because
motor voltage is limited in firmware by the PWM cap, not in hardware.

## 5. Pin map (as wired)

Already in use, do not reassign:

| Function | GPIO |
|----------|------|
| TB6612 PWMA / AIN1 / AIN2 | 25 / 26 / 27 |
| TB6612 PWMB / BIN1 / BIN2 | 14 / 12 / 13 |
| TB6612 STBY | 33 |
| Encoder left DO / right DO | 32 / 4 |

GPIO 1 and 3 are the USB serial line, never use them.

## 6. Hardware reference

| Value | Measurement |
|-------|-------------|
| Wheel base (centre to centre) | 0.13 m |
| Wheel diameter | 0.045 m |
| No-load speed | about 108 RPM at PWM 150 |
| MAX_WHEEL_SPEED (derived) | 0.43 m/s, unverified |
| MIN_PWM (stiction floor, tuned) | 90 |
| Encoder edges per revolution | not recorded |

## 7. Electrical lessons worth not relearning

- Encoder modules run from 3V3, not 5 V. Their DO output sits at the supply voltage,
  and 5 V is over the ESP32's pin limit.
- Encoders need pins with internal pull-ups. On GPIO 34/35 they counted nothing even
  though the module LEDs toggled, because 34 to 39 are input-only and have no internal
  pull-ups, which an open-collector DO output needs. Moved to 32 and 4 with INPUT_PULLUP.
- MIN_PWM was raised from 60 to 90. One motor would not overcome static friction at 60.
  Closed-loop control using the encoders is the proper fix later.
- HC-SR04 ECHO outputs 5 V and needs a level shifter. Only relevant if the ultrasonic
  array in the appendix gets built.

## 8. Environment notes

- Simulation runs on ROS 2 Jazzy (laptop); the robot runs ROS 2 Humble (Pi, in Docker).
  The avoidance node is portable between them. The only difference is the cmd_vel
  message type, set by a runtime parameter: TwistStamped in simulation, plain Twist on
  the hardware.
- Code reaches the Pi by git, not by copying files. Push from the laptop, git pull on
  the Pi host (not inside the container, which has the repo mounted into it), then
  rebuild inside the container because ros2 run uses the installed copy.
- A Windows machine needs WSL for the ROS 2 side; plain Command Prompt will not work.
  The Arduino IDE and all hardware work run on Windows natively. Run wsl --install from
  an Administrator PowerShell, then reboot.

## 9. Calibration loose ends (never done, quick, no new parts)

- Floor-speed calibration. MAX_WHEEL_SPEED of 0.43 m/s comes from a no-load RPM
  measurement and is optimistic; loaded speed is probably 0.30 to 0.35. Test: stream a
  linear command of 0.2 m/s for 5 s, which should cover about 1.0 m. If it falls short,
  lower the constant and reflash.
- Encoder edges-per-revolution was never recorded. Turn one wheel exactly one
  revolution by hand and note the count increase (expect about 40 for a 20-slot disc
  with CHANGE interrupts). That figure plus the 0.045 m wheel diameter converts pulses
  to metres. Odometry cannot start without it.

These two take about 20 minutes together and are good first tasks on returning: they
rebuild familiarity with the toolchain before the harder work.

## 10. Ideas for August

- Untethered power: finish the LiPo and buck setup for the Pi (section 4).
- Richer sensing: a multi-sensor ultrasonic array (plan in the appendix), the TFmini
  swept on a servo, or a small lidar. All feed the same scan interface, so the avoider
  does not change.
- SLAM and mapping on the real hardware (it already works in simulation).
- The two calibration jobs in section 9.

## 11. Key files

| File | Contents |
|------|----------|
| docs/handoff.md | Fuller architectural handoff |
| docs/sim.md | Simulation setup and how to reproduce it |
| docs/pi-deployment.md | Getting code onto the Pi, Docker, serial access |
| docs/hardware.md | Pin map, chassis measurements, serial protocol |
| docs/setup.md | Windows and WSL developer setup |
| firmware/cmd_vel_bridge/ | ESP32 motor firmware (working, flashed) |
| firmware/encoder_test/ | ESP32 encoder verification sketch |

## Appendix — ultrasonic array (planned, not built)

This was the original plan for wider sensing before the single TFmini was used. It is
kept here as reference in case the array gets built later. None of it was wired.

Planned pin map (ECHO on input-only pins, TRIG on general-purpose pins):

| Sensor | Position | TRIG | ECHO (via shifter) |
|--------|----------|------|--------------------|
| 0 | far left | 16 | 34 |
| 1 | left | 17 | 35 |
| 2 | centre | 18 | 36 |
| 3 | right | 19 | 39 |
| 4 | far right | 23 | 5 |

GPIO 5 is a strapping pin (sampled at boot). Fine as an ECHO input in practice, but if
the board ever fails to boot with sensors attached, suspect it first.

Level shifter (HW-221 / TXS0108E, 8-channel, three on hand). B side is 5 V (sensors), A
side is 3.3 V (ESP32). Wire VB to 5 V, VA to 3V3, both GNDs to the common ground, and OE
to 3V3. The OE pin is easy to miss, and the shifter does nothing if it is left floating.

Per sensor: VCC to 5 V, GND to common ground, TRIG to an ESP32 GPIO directly (3.3 V out
reads as logic-high), ECHO through a shifter B channel with the matching A channel to the
ESP32. ECHO outputs 5 V and must never reach an ESP32 pin without the shifter; check with
a multimeter that the A side reads 3.3 V before connecting it. Wire and test one sensor
fully before copying to the rest.

If built, fire the sensors one at a time (parallel pings cross-talk), about 60 ms each,
so a five-sensor sweep runs near 5 Hz. Emit one line per sweep, for example
U d0 d1 d2 d3 d4 in centimetres with -1 for no echo, and publish it on the Pi as a single
five-beam LaserScan (angle_min -90 degrees, angle_max +90 degrees, five ranges in metres,
invalid as inf, frame_id laser_frame). Before integrating, check that the cmd_vel
publisher and subscriber show the same message type with ros2 topic info /cmd_vel
--verbose; a mismatch silently drops commands, so the robot looks healthy but does not
move. That caught us twice.
