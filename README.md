
---

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://github.com/ManchesterRoboticsLtd/Puzzlebot/blob/main/Misc/Logos/Puzzle_Bot_Logo_W.png">
  <source media="(prefers-color-scheme: light)" srcset="https://github.com/ManchesterRoboticsLtd/Puzzlebot/blob/main/Misc/Logos/Puzzle_Bot_Logo_B.png">
  <img alt="Shows MCR2 logo in black or white." width="250" align="right">
</picture>


 <div id="user-content-toc">
  <ul align="center" style="list-style: none;">
    <summary>
      <h1>Puzzlebot Python Simulator</h1>
    </summary>
  </ul>
</div>

---

## 🎓 Introduction

The **Puzzlebot Python Simulator** provides a 2D GUI-based simulation environment for differential-drive mobile robots. Designed by [Manchester Robotics](https://manchester-robotics.com), it is ideal for learning and teaching robotics fundamentals such as control algorithms, localization, and mapping using Python.

This simulator supports both standalone virtual robots and real Puzzlebot hardware via standardized topic communication.

It also includes simulation of different dynamiclal systems to learn control concepts such as Open and closed loop control, frequency anaalysis, system identification, etc. 

---

## 🚀 Features

- 🛞 Differential-drive robot simulation
- 🧠 Integrated control via velocity commands
- 📏 Simulated sensors:
  - Laser distance sensor (single beam)
  - 2D LiDAR scan
  - Marker-based range and bearing sensor (Aruco marker)
  - Servo motor for sensor rotation
- 🧩 Customizable robot definitions via JSON files
- 🔁 Compatible with real Puzzlebot via topic interfaces
- 🧪 Perfect for teaching control, dead reckoning, and SLAM

---

## 🧪 Examples Included

Found in the `my_examples/` folder:

| Example | Description |
|--------|-------------|
| `drive_straight.py` | Moves the robot forward for a fixed time |
| `drive_square.py` | Drives the robot in a square path using odometry |
| `dead_reckoning.py` | Estimates robot pose using encoder data |
| `driveToGoal.py` | Navigates to multiple targets using dead reckoning |
| `BinaryMapping.py` | Builds a binary map using LiDAR data |
| `driveToGoalMapping.py` | Combines navigation with mapping |
| `kalman.py` | Implements Kalman filter correction step |
| `driveToGoalKalman.py` | Navigation using Kalman-based localization |
| `driveToGoalObst.py` | Basic obstacle avoidance while navigating |
| `follow_wall.py` | Wall-following behavior using laser range sensor |

---

## 🤖 Robot Configurations

JSON robot definitions located in the `hardware/` folder:

- `puzzlebot01.json`: Includes laser, LiDAR, servo, and marker sensors
- `puzzlebot_laser.json`: Laser distance + servo
- `puzzlebot_cam01.json`: Marker sensor for vision-based localization

---


## 🗂️ Communication Topics

Defined in each `.json` config file:

| Topic | Description | Type |
|-------|-------------|------|
| `VelocitySetR`, `VelocitySetL` | Wheel control commands | `puzz_msgs.Float32` |
| `VelocityEncR`, `VelocityEncL` | Encoder velocities | `puzz_msgs.Float32` |
| `Pose` | Estimated pose (must be set by user) | `puzz_msgs.Pose` |
| `LaserDistance` | Single-beam range measurement | `puzz_msgs.Float32` |
| `LidarScan` | Full LiDAR scan | `puzz_msgs.LaserScan` |
| `ServoAngle` | Servo motor angle in degrees | `puzz_msgs.Float32` |
| `markers` | Detected marker bearings/ranges | `puzz_msgs.MarkerBearingArray` |

Message definitions are found in `lib/puzz_msgs.py`. You don’t need to manually set `type`, `topic`, or `status` fields — they’re auto-handled.

---

## 🧰 Installation

### Prerequisites

- Python 3.8+  
- Packages: `numpy`, `matplotlib`, `json`.

### Install Dependencies

```bash
pip install -r requirements.txt
```
## Run the GUI Simulator

```bash
python puzzlebot_gui.py
```

