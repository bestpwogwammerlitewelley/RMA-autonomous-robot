# RMA-autonomous-robot
## Goal:
A self-built differential-drive robot that perceives its surroundings with a custom
servo-swept rangefinder and autonomously avoids obstacles while roaming an indoor
space — developed as modular ROS 2 nodes, validated in simulation first,
containerized with Docker, and version-controlled on GitHub.

## Team
- **T1 (Beau)** — order material, GitHub Repo setup, rest TBD
- **T2 (Adam)**— TBD
- **T3 (Olaf)** — TBD

## Plan — Tiered goals (10-days, + possible August follow-on week) 
- (all from CLAUDE)

**Tier 0 — Guaranteed (Day 7)**
- Chassis assembled, motors + caster + power wired
- ESP32 drives motors via TB6612 (PWM capped for 6V motors)
- Pi `/cmd_vel` → ESP32 serial → wheels move (teleop working)
- Obstacle avoidance proven in Gazebo sim against a simulated rangefinder
- Everything running in Docker, full history in Git

**Tier 1 — Target (Day 9)**
- Encoder odometry (LM393 sensors) publishing
- HC-SR04 ultrasonic on the real robot → reactive obstacle avoidance, live

**Tier 2 — Stretch (Day 10 / August)**
- TFmini-S on servo publishing `LaserScan` (swept rangefinder)
- Smoother/more capable avoidance
- Real 360° lidar upgrade → SLAM mapping (drop-in sensor swap, same `LaserScan` interface)
- Android app supervision layer
- mmWave radar module (redundant safety layer)

## Architecture
```
TFmini-S / HC-SR04 + wheel encoders
        │
        ▼
ESP32 (real-time motor control, serial link)
        │
        ▼
Raspberry Pi 5 (ROS 2 Humble, Dockerized)
   - teleop / cmd_vel
   - obstacle avoidance node
   - (later) SLAM / Nav2
```

## Hardware status
| Item | Status | Notes |
|---|---|---|
| Chassis + motors + caster + encoder discs | Ordered (Amazon.de) | |
| Raspberry Pi 5 8GB | Ordered |  |
| Active Cooler + 27W PSU | Ordered | |
| TB6612FNG motor driver | Ordered | |
| LM393 encoder sensors | Ordered | |
| ESP32 DevKitC | Ordered | |
| MPU-6050 IMU | Ordered | For August/heading estimation |
| 2S LiPo battery + HTRC charger | Ordered | |
| XT60s, wiring, standoffs, tape, breadboard, perfboard, SD cards ×2 | Ordered | |
| TFmini-S ToF rangefinder | **Requested cancellation, next step TBD** | Original ETA 30–31 July |
| HC-SR04 ultrasonic (bridge sensor) | TBD depending on TFmini cancellation | Fast delivery, unblocks Tier 1 without waiting on TFmini |
| Soldering iron, solder, wire strippers, multimeter | In basket / asking PM re: lab loan | Build location: teammate's house |

## Notes
- Build takes place at T3's house — soldering setup TBD (lab loan pending PM approval).
- Budget: €506 currnetly (kit + Pi + tools separate), against a €600 ceiling.
- Sim-first workflow: obstacle avoidance is validated against a simulated sensor in Gazebo before any hardware sensor is required; why the TFmini delay doesn't block the 10 day plan.
```
