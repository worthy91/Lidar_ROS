📹 Demo Video: https://photos.app.goo.gl/yoX34gfQT7HfFGDB7


Installation & Usage

Prerequisites
Ubuntu 22.04 LTS
ROS 2 Humble
Nav2 Stack
ros_gz_bridge

Build

Bash

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
Run
The system uses a unified launch file that handles the simulation, navigation stack, and mission control nodes in a timed sequence to prevent race conditions.

Bash

ros2 launch my_cpp_pkg my_world_nav.launch.py
Note: The GUI dashboard will appear after approx. 15 seconds once the navigation stack is fully active.

# Autonomous Security Patrol Robot 🤖


## 📖 Project Overview

This project represents a complete autonomous security patrol implementation developed using **ROS 2 Humble** and the **Nav2 stack**. Designed for a specific office environment, the robot autonomously patrols predefined waypoints, avoids obstacles in real-time, and manages its own charging cycles based on battery levels.

This system demonstrates a practical application of autonomous mission planning, sensor fusion (EKF), and custom mapping in a constrained environment.


---

## 🎯 Key Features

* **Autonomous Patrol:** Navigates sequentially through 4 strategic waypoints in a loop.
* **Intelligent Battery Management:** Monitors battery levels; automatically aborts mission and navigates to the charging dock when battery drops below 25%.
* **Smart Docking:** Recharges to 100% before automatically resuming the patrol mission.
* **Real-time Obstacle Avoidance:** Uses a rolling window costmap (5cm resolution) to navigate dynamic environments.
* **Custom Monitoring GUI:** A Tkinter-based dashboard for real-time status and battery tracking.

---

## 🛠️ Technical Architecture

### Software Stack
* **Framework:** ROS 2 Humble
* **Navigation:** Nav2 (Regulated Pure Pursuit Controller)
* **Simulation:** Gazebo (Custom world modeling actual office floor plan)
* **Localization:** AMCL (Adaptive Monte Carlo Localization) + EKF (Robot Localization)


### Navigation Strategy
I selected the **Regulated Pure Pursuit Controller** over MPPI for this differential drive robot.
* **Reasoning:** It handles U-turns and reversals more gracefully in the confined office corridors compared to MPPI.
* **Configuration:**
    * Max Linear Speed: 0.4 m/s (Safety constrained)
    * Lookahead Distance: Adaptive (0.3m - 0.9m)
    * Rotate-to-heading: Enabled (Critical for narrow doorways)

### Battery & Mission Logic
The robot operates on a Finite State Machine managed by a custom Python node (`patrol_manager.py`).

| Parameter | Value | Description |
| :--- | :--- | :--- |
| **Critical Threshold** | 25% | Triggers "Return to Dock" behavior |
| **Discharge Rate** | 1% / sec | Simulated discharge during operation |
| **Charge Rate** | 10% / sec | fast charging at dock |
| **Dock Location** | `[-4.38, 1.28]` | Fixed charging station coordinates |

---

## 📂 File Structure

```text
src/my_cpp_pkg/
├── config/              # Nav2 parameters and EKF configuration
├── launch/              # Orchestrated launch file (Sim + Nav2 + Nodes)
├── maps/                # Custom generated office PGM/YAML maps
├── nodes/               # Python source code
│   ├── battery_node.py  # Simulates battery discharge/charge logic
│   └── patrol_manager.py# Mission control & State Machine
├── robot/               # URDF Robot description
└── worlds/              # Gazebo simulation worlds
