# Simulation Setup Guide

This guide explains how to set up the ROS2 and Gazebo environment to simulate the autonomous food warehouse robot.

## Prerequisites
- **Operating System**: Ubuntu 22.04 (Jammy) or compatible.
- **Software**:
  - ROS2 Humble Hawksbill
  - Gazebo (version 11 or later)
  - Python 3.10+
  - OpenCV (`pip install opencv-python`)
- **Hardware**: PC with 8GB RAM, 4-core CPU, and GPU (recommended for Gazebo).

## Installation Steps
1. **Install ROS2 Humble**:
   ```bash
   sudo apt update
   sudo apt install locales
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt install ros-humble-desktop
   source /opt/ros/humble/setup.bash
   ```
   1. **Install Gazebo**:
       ```bash
       sudo apt install ros-humble-gazebo-ros-pkgs
        ```
   2. **Install Dependencies:**
      ```bash
      sudo apt install python3-pip
      pip3 install opencv-python
      sudo apt install ros-humble-robot-state-publisher ros-humble-nav2-bringup
      ```
   3. **Clone Repository:**
       ```bash
       git clone https://github.com/FaisalSWE/automated-warehouse-robot.git
       cd automated-warehouse-robot
       ```
   4. **Build ROS Workspace:**
      ```bash
         cd code/ros_workspace
         colcon build
         source install/setup.bash
      ```
## Running the Simulation
1. **Launch Gazebo with Robot:**
    ```bash
         ros2 launch warehouse_robot simulate_robot.launch.py
**This loads:**
- Gazebo world (code/simulation/warehouse.world)
- Robot URDF (code/simulation/warehouse_robot.urdf)
- Navigation node (code/ros_workspace/src/warehouse_robot/nav_node.py)

3. **Run QR Scanner Node (in a new terminal):**
    ```bash
         source code/ros_workspace/install/setup.bash
         ros2 run warehouse_robot qr_scanner.py

## Expected Output
- Gazebo opens with a 50m x 30m warehouse, shelves, and the robot.
- Navigation node logs map receipt and path planning.
- QR scanner node publishes detected QR codes to /qr_data.
## Troubleshooting
-Gazebo Crashes: Ensure GPU drivers are updated; reduce world complexity.
-ROS Node Errors: Verify colcon build completed successfully.
 QR Detection Fails: Check camera simulation settings in Gazebo.
