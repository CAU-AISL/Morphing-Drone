#!/bin/bash

echo "Installing required ROS2 and Gazebo packages..."

# ROS2 Humble core dependencies
sudo apt update
sudo apt install -y \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control \
  libgazebo-dev \

# Gazebo tools
sudo apt install -y \
  gazebo \
  ros-humble-gazebo-ros

# Python dependency (for ament/catkin compatibility)
echo "Installing Python dependencies..."
pip3 install catkin_pkg

echo "Installation complete."
