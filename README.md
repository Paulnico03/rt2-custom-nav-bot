# RT2 Assignment 1 - Custom State-Machine Navigation Stack

## Overview
This package provides a custom robot navigation system built for ROS 2. It implements a fully functional action server and client using **ROS 2 Components (Plugins)** running inside a unified **Manual Composition Container**, fulfilling the core assignment requirements.

Instead of a basic proportional controller, this system utilizes a **3-Phase State Machine** (Aim → Drive → Align) for precise movement. It also features a custom terminal UI that provides live, real-time feedback of the robot's distance to the target.

---

## Key Features
*   **Component-Based Architecture:** Both the UI Client and Navigation Server run as shared library plugins in the same process footprint.
*   **TF2 Integration:** Robot localization is strictly handled by querying `tf2_ros::Buffer` for `odom` → `base_link` transforms, ensuring robust spatial tracking.
*   **State-Machine Control:** 
    *   *Phase 1:* Rotate in place to face the target coordinates.
    *   *Phase 2:* Drive linearly while making minor angular corrections.
    *   *Phase 3:* Perform a final rotation to match the target heading ($\theta$).
*   **Live Dashboard:** The UI uses carriage returns (`\r`) to provide a sleek, updating countdown of the distance remaining without flooding the terminal.

---

## Package Structure
```text
custom_nav_bot/
├── action/
│   └── GoToPoint.action
├── include/custom_nav_bot/
│   ├── nav_server.hpp
│   └── ui_client.hpp
├── src/
│   ├── nav_server.cpp
│   ├── ui_client.cpp
│   └── main_container.cpp
├── CMakeLists.txt
└── package.xml
```
---
## How to Build and Run

###1. Build the Workspace
cd ~/ros2_ws
colcon build --packages-select custom_nav_bot
source /opt/ros/jazzy/setup.bash
source install/setup.bash

###2. Launch the Simulation Environment
ros2 launch bme_gazebo_sensors spawn_robot.launch.py

###3. Run the Navigation Container
In a new terminal, launch the combined Action Server and UI Client process:
source install/setup.bash
ros2 run custom_nav_bot run_robot_system
---

###4. Usage Instructions
Once the UI starts, type your desired target coordinates (X, Y, Theta) separated by spaces:
[Target Input] > 4.0 4.0 1.57

To preemptively stop the robot and cancel the goal at any time, simply type:
[Target Input] > cancel

## Author: Paolo Nicolini 