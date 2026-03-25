
## Personal Competition Robot — ROS2 Humble

Personal workspace for a 4-wheel skid-steer competition robot running on Raspberry Pi 4 Model B.

## Hardware

| Component | Details |
|---|---|
| SBC | Raspberry Pi 4 Model B |
| LiDAR | 2D LiDAR (e.g. RPLidar A1/A2) on `/scan` |
| Drive motors | 4× DC motor — skid-steer layout |
| Motor drivers | 2× TB6612FNG (front pair + rear pair) |
| Effector servos | 3× DSServo — 2 paddle, 1 crank (3D printed) |
| Odometry | IMU + EKF localization |

## Package Layout

```
src/
├── robot_bringup/     # Master launch files
├── robot_description/ # URDF/Xacro model
├── robot_navigation/  # Nav2 + LiDAR config
├── robot_drivers/     # TB6612FNG motor driver + odometry
├── robot_effectors/   # DSServo paddle + crank controller
└── robot_simulation/  # Gazebo sim world
```

## Quick Start

```bash
# Install deps
rosdep install --from-paths src --ignore-src -r -y
pip install RPi.GPIO pigpio

# Build
colcon build --symlink-install
source install/setup.bash

# Run on hardware
ros2 launch robot_bringup robot.launch.py

# Run in simulation
ros2 launch robot_bringup sim.launch.py
```

## GPIO Pin Map

### TB6612FNG — Front Driver (motors FL + FR)
| Signal | RPi GPIO (BCM) |
|---|---|
| PWMA (FL speed) | 32 |
| AIN1 (FL dir)   | 29 |
| AIN2 (FL dir)   | 31 |
| PWMB (FR speed) | 33 |
| BIN1 (FR dir)   | 35 |
| BIN2 (FR dir)   | 37 |
| STBY            | 40 |
| VCC             | 17 |
| VIN to 12V rail |
| GND             | 39 |

### TB6612FNG — Rear Driver (motors RL + RR)
| Signal | RPi GPIO (BCM) |
|---|---|
| PWMA (RL speed) | 12 |
| AIN1 (RL dir)   | 11  |
| AIN2 (RL dir)   | 13 |
| PWMB (RR speed) | 22 |
| BIN1 (RR dir)   | 15 |
| BIN2 (RR dir)   | 18 |
| STBY            | 38 |
| VCC             | 17 |
| VIN to 12V rail |
| GND             | 39 |

## IMU - Odometry 
| VCC             | 1 |
| GND             | 6 |
| SCL             | 5 |
| SDA             | 3 |
| EDA X               |
| ECL X               |
| ADO             | 6 |
| (optional) INT  | 11 |
| NCS             | 1 |
| FSYNC X             |

### DSServo — Effectors
| Servo | RPi GPIO (BCM) | Default angle |
|---|---|---|
| Paddle Left  | - | 90° (neutral) |
| Paddle Right | - | 90° (neutral) |
| Crank        | - | 0°  (retracted) |

## Servo Auto-Trigger Logic

- **Paddle servos** — fire when LiDAR detects an object within `paddle_trigger_distance`
  (default 0.5 m) in the forward arc. Both paddles sweep to 45° then return.
- **Crank servo** — fires when Nav2 signals goal reached. Sweeps 0° → 180° → 0°.

Tune thresholds in `robot_effectors/config/effector_config.yaml`.
=======
# pi-ros2-robo
>>>>>>> ce1b0d54f489b8c5cbd2d94fbccd266a7acb1241
