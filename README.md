# Morphing-Drone-sim

A simulation project for a 12-actuator drone featuring:
- 4 main motors
- 4 tilt servos
- 4 flare servos

This project uses **ROS 2 Humble** and **Gazebo Classic** for simulation and visualization of the morphing drone dynamics.

## Dependencies
All required dependencies for ROS 2 and Gazebo Classic can be installed using the provided script.


## ROS 2 & Gazebo Setup Script
This script installs essential dependencies for using **ROS 2 (Humble)** with **Gazebo Classic**, and sets up a recommended development environment for working with URDF/XACRO models.

### What it installs

- **ROS 2 packages**:
  - `joint-state-publisher-gui`
  - `robot-state-publisher`
  - `xacro`
  - `gazebo-ros-pkgs`
  - `ros2_control` and controllers
- **Gazebo simulator** and plugins
- **Python** dependencies:
  - `catkin_pkg`
- (Optional) **VSCode extensions**:
  - XML syntax highlighting
  - URDF model tools

### How to Use

1. Clone this repository and navigate into it:
   ```bash
   git clone https://github.com/your-username/Morphing-Drone-sim.git
   cd Morphing-Drone-sim
   ```
2. Give execution permission to the setup script:
   ```bash
   chmod +x setup_dependencies.sh
   ```
3. Run the script to install all required packages:
   ```bash
   ./setup_dependencies.sh
   ```
4. Source your ROS2 environment:

   ```bash
   source /opt/ros/humble/setup.bash
   ```
5. Build the workspace:
   ```bash
   cd ROS2_ws
   colcon build
   source install/setup.bash
   ```
6. Launch the simulation (example):
   ```bash
   ros2 launch morphing_description morphing_launch.py
   ```
