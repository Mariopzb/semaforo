# Puzzlebot — Autonomous Navigation & Traffic Light Detection

> **ROS2-based autonomous robot system** featuring closed-loop control, odometry estimation, path following, and real-time traffic light detection using computer vision.

---

##  Repository Structure

```
📁 puzzlebot/
├── turtle_odometry.py       # Wheel encoder odometry
├── turtle_closeloop.py      # Closed-loop PID controller
├── path_generator.py        # Waypoint path planner
└── semaforo.py              # Traffic light supervisor
```

---

##  Node Overview

```
/VelocityEncR ──┐
                ├──► turtle_odometry ──► /odom ──┐
/VelocityEncL ──┘                                │
                                                 ▼
                              path_generator ──► /next_point ──► turtle_closeloop ──► /cmd_vel_raw
                                    ▲                                                        │
                                    │                                                        ▼
                                  /odom                                              semaforo.py
                                                                                            │
                                                                              /video_source/raw
                                                                                            │
                                                                                            ▼
                                                                                       /cmd_vel
```

---

##  Nodes

---

###  `turtle_odometry.py` — Odometry Node

Estimates the robot's pose `(x, y, θ)` from wheel encoder velocities using differential drive kinematics.

**Subscriptions**
| Topic | Type | Description |
|---|---|---|
| `/VelocityEncR` | `std_msgs/Float32` | Right wheel angular velocity |
| `/VelocityEncL` | `std_msgs/Float32` | Left wheel angular velocity |

**Publications**
| Topic | Type | Description |
|---|---|---|
| `/odom` | `turtlesim/Pose` | Estimated pose `(x, y, θ)` |

**Key Parameters**
```python
r = 0.052   # Wheel radius (m)
L = 0.18    # Wheelbase (m)
rate = 100  # Update rate (Hz)
```

**Kinematics**
```
v = r * (ωR + ωL) / 2
ω = r * (ωR - ωL) / L

x     += dt * v * cos(θ)
y     += dt * v * sin(θ)
θ     += dt * ω
```

---

### `turtle_closeloop.py` — Closed-Loop Controller

State machine controller that navigates the robot to waypoints using proportional control for both angle and distance.

**Subscriptions**
| Topic | Type | Description |
|---|---|---|
| `/odom` | `turtlesim/Pose` | Current robot pose |
| `/next_point` | `turtlesim/Pose` | Target waypoint |

**Publications**
| Topic | Type | Description |
|---|---|---|
| `/cmd_vel_raw` | `geometry_msgs/Twist` | Raw velocity command |

**State Machine**
```
        [new target]           [angle OK]           [distance OK]
  stop ──────────────► state1 ──────────► state2 ───────────────► stop
                      (rotate)            (move)
```

**Tuning Parameters**
```python
Kv = 0.5     # Linear velocity gain
Kw = 1.2     # Angular velocity gain
v_limit = 0.4                 # Max linear speed (m/s)
tolerance_distance = 0.02     # Distance threshold (m)
tolerance_angle    = 0.03     # Angle threshold (rad)
```

---

###  `path_generator.py` — Path Generator

Publishes sequential waypoints to guide the robot through a predefined path. Advances to the next waypoint automatically once the robot is within the tolerance threshold.

**Subscriptions**
| Topic | Type | Description |
|---|---|---|
| `/odom` | `turtlesim/Pose` | Current robot pose |

**Publications**
| Topic | Type | Description |
|---|---|---|
| `/next_point` | `turtlesim/Pose` | Current target waypoint |

**Supported Paths**

| Shape | Waypoints |
|---|---|
|  Square | `(2,0) → (2,2) → (0,2) → (0,0)` |
|  Triangle | `(2,0) → (1, 1.732) → (0,0)` |
|  Trapezoid | `(2,0) → (1.5,1) → (0.5,1) → (0,0)` |

To switch shapes, uncomment the desired `point_list` in `__init__`.

---

###  `semaforo.py` — Traffic Light Supervisor

Intercepts velocity commands and modulates them based on real-time traffic light detection via HSV color segmentation. Implements a **sequential state machine** that enforces the correct Green → Yellow → Red → Green cycle.

**Subscriptions**
| Topic | Type | Description |
|---|---|---|
| `/video_source/raw` | `sensor_msgs/Image` | Camera feed |
| `/cmd_vel_raw` | `geometry_msgs/Twist` | Raw velocity from controller |

**Publications**
| Topic | Type | Description |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Modulated velocity output |

**State Machine**

```
             [yellow detected]      [red detected]       [green detected]
  🟢 GREEN ──────────────────► 🟡 YELLOW ──────────► 🔴 RED ──────────────► 🟢 GREEN
    ×1.0     (not green at same time)  (seen_yellow=True)   ×0.0  (seen_red=True)   ×1.0
                    ×0.5
```

**Speed Multipliers**
| State | Multiplier | Behavior |
|---|---|---|
|  GREEN | `1.0` | Full speed |
|  YELLOW | `0.5` | Slow down |
|  RED | `0.0` | Full stop |

**Transition Guards**
- `GREEN → YELLOW` requires yellow **without** green (prevents HSV overlap false positives)
- `YELLOW → RED` requires `seen_yellow = True` (ensures legitimate sequence)
- `RED → GREEN` requires `seen_red = True` (ensures robot was actually stopped first)

**HSV Color Ranges**
```python
Green:   H[40–80]   S[50–255]   V[50–255]
Yellow:  H[20–35]   S[100–255]  V[100–255]
Red:     H[0–10]  + H[170–180]  S[150–255]  V[120–255]
```

---

##  Running the System

Launch all nodes in separate terminals:

```bash
# 1. Odometry
ros2 run <package> turtle_odometry.py

# 2. Closed-loop controller
ros2 run <package> turtle_closeloop.py

# 3. Path generator
ros2 run <package> path_generator.py

# 4. Traffic light supervisor
ros2 run <package> semaforo.py
```

---

##  Dependencies

- ROS2 (Humble or later)
- `rclpy`
- `opencv-python`
- `numpy`
- `cv_bridge`
- `turtlesim` *(for simulation)*

```bash
pip install opencv-python numpy
sudo apt install ros-humble-cv-bridge
```

---

##  Notes

- The odometry node uses `time.time()` for `dt` — make sure system clock is stable.
- Color thresholds may need tuning depending on lighting conditions and camera model.
- `threshold` in the path generator (`0.1 m`) and controller (`0.02 m`) can be adjusted for precision vs. stability.

---

<div align="center">
  <sub>Built with ROS2 · OpenCV · Python</sub>
</div>
