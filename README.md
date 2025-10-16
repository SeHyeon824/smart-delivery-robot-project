Face Recognition-based Smart Delivery System

This project is an autonomous delivery robot system that performs real-time SLAM and secure delivery through a dual-authentication process. It was developed as a final project for a comprehensive engineering design course.

1. Project Overview

The primary goal of this project is to develop an intelligent delivery robot that can autonomously navigate indoor environments. The robot creates a map in real-time using SLAM, avoids obstacles to reach its destination, and securely delivers items through a two-factor authentication process: 1) Face Recognition and 2) Remote Admin Approval via Telegram upon failure.

This system is designed to be a cost-effective and flexible solution for various indoor delivery scenarios where security and adaptability are crucial.

2. Key Features

Real-time SLAM: Generates and expands a 2D map of unknown environments using Google's Cartographer.

Autonomous Navigation: Navigates to a specified goal while avoiding dynamic and static obstacles using the ROS move_base navigation stack.

Intelligent Recovery Behavior: Autonomously executes a backup maneuver (e.g., moving backward) to escape deadlock situations when stuck.

Dual-Authentication System:

Primary (Face Recognition): Uses the face_recognition library to verify the recipient.

Secondary (Remote Approval): If face recognition fails, it sends a photo of the scene to an administrator via the Telegram Bot API for remote approval or rejection.

Distributed System Communication: Utilizes Socket programming to orchestrate tasks between the main control PC (Xycar) and the authentication unit (Raspberry Pi).

3. System Architecture

The system operates as a distributed network composed of a Xycar PC (for navigation), a Raspberry Pi (for authentication), and an Administrator's Smartphone (for remote control).

Data Flow:

Goal Setting: A user sets a destination goal in RViz.

Autonomous Driving: The Xycar PC's move_base plans a path, avoids obstacles, and sends /cmd_vel commands.

Arrival Signal: Upon reaching the goal, destination_watcher.py detects the success and sends a "GOAL_REACHED" signal to the Raspberry Pi via a socket.

Authentication: The Raspberry Pi's smart_safe_main.py activates the camera and attempts face recognition.

Remote Approval Request: On failure, telegram_manager.py sends a photo and buttons to the administrator's smartphone.

Safe Unlock: Upon successful face recognition or remote approval, gpiozero is used to control a servo motor and open the safe.

4. Used Technologies (Hardware & Software)

Main Platform: Xycar Robot, Asus mini-PC (Ubuntu 20.04, ROS Noetic)

Authentication Unit: Raspberry Pi 5, Camera Module 3, Servo Motor

Key Sensors: 2D LiDAR (YDLIDAR G4), 9-DoF IMU

Core Frameworks & Libraries:

| Field | Technology / Library | Role |
| Autonomous Driving | Cartographer, move_base, DWA Planner | Real-time SLAM, Path Planning, Obstacle Avoidance |
| Artificial Intelligence | OpenCV, face_recognition | Real-time Image Processing, Face Feature Extraction |
| Communication | Socket (TCP/IP), Telegram Bot API | PC-Pi Communication, Remote Admin Communication |
| Hardware Control | gpiozero, picamera2 | Servo Motor Drive, Camera Control |

5. Project Structure

The project is primarily managed within the sehyen_project ROS package.

launch/: Contains ROS launch files for running different parts of the system.

bringup.launch: Starts all basic hardware drivers and SLAM.

navigation.launch: Starts the move_base navigation brain and the mission controller.

config/: Contains parameter files (.yaml) that tune the robot's behavior.

costmap_... .yaml: Defines the robot's size (footprint) and safety margins (inflation_radius).

base_local_planner_params.yaml: Configures the robot's "driving style" (max speed, acceleration, etc.).

recovery_params.yaml: Defines "Plan B" actions for when the robot gets stuck.

scripts/: Contains the main Python executable scripts.

destination_watcher.py: The high-level mission controller running on the Xycar PC.

rviz/, urdf/: Stores RViz configurations and the robot's 3D model.

6. Key Technical Challenges & Solutions

move_base 'Robot off map' Error:

Problem: move_base consistently failed to plan paths, even when TF data appeared correct.

Solution: Traced the root cause to a 'double namespace' issue where ROS parameters were being loaded into incorrect addresses (e.g., /move_base/global_costmap/global_costmap/). Resolved by refactoring all .yaml files to remove redundant top-level keys and correcting the ns attribute in the navigation.launch file. This demonstrated a deep understanding of the ROS parameter server.

Deadlock During Navigation:

Problem: The robot would freeze in front of obstacles because the DWA planner failed to find a valid path but did not declare a failure, thus never triggering recovery behaviors.

Solution: Activated the oscillation detection feature in move_base (oscillation_timeout, oscillation_distance) to forcibly detect a stuck state. Additionally, the planner's "personality" was tuned by lowering path_distance_bias and adjusting recovery_params.yaml to encourage more flexible and aggressive escape maneuvers.

Raspberry Pi Environment Setup:

Problem: Encountered externally-managed-environment errors with pip and ModuleNotFoundError: No module named 'libcamera' due to recent OS policy changes and library dependencies.

Solution: Established a robust development environment by creating a venv virtual environment with the --system-site-packages option. This allowed the virtual environment to access system-level libraries (like python3-libcamera) installed via apt, while still managing project-specific packages independently. Also installed necessary build tools like cmake and libcap-dev.y
