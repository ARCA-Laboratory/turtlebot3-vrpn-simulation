# Simulating OptiTrack VRPN with TurtleBot3 in ROS Melodic on WSL (Windows Subsystem for Linux)

## Prerequisites
- **ROS Melodic** and **Gazebo** installed on your WSL machine.
- **X server** set up on Windows (e.g., VcXsrv or Xming) to run graphical applications (like RViz and Gazebo).

## Steps to Configure and Simulate VRPN with TurtleBot3 and ROS

### 1. Install TurtleBot3 and Gazebo

1. **Install TurtleBot3 Packages**:
   Open a terminal in WSL and run:
   ```bash
   sudo apt-get install ros-melodic-turtlebot3 ros-melodic-turtlebot3-simulations
