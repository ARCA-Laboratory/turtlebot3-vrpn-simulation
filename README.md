# Simulating OptiTrack VRPN with TurtleBot3 in ROS Melodic on WSL (Windows Subsystem for Linux)

## Prerequisites
- **ROS Melodic** and **Gazebo** installed on your WSL machine.
- **X server** set up on Windows (e.g., VcXsrv or Xming) to run graphical applications (like RViz and Gazebo).

## Steps to Configure and Simulate VRPN with TurtleBot3 and ROS

### Step 1: Install TurtleBot3 Packages

Open a terminal in WSL and run:

```
sudo apt-get install ros-melodic-turtlebot3 ros-melodic-turtlebot3-simulations
```
---

### Step 2: Set the TurtleBot3 Model Environment Variable

Add the following lines to your `.bashrc` file:

```
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc  
source ~/.bashrc
```
---

### Step 3: Verify the Installation

Launch Gazebo with the TurtleBot3 world to ensure everything is installed correctly:
```
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
If the TurtleBot3 robot and world load successfully in Gazebo, the installation is correct.

---

### Step 4: Modify and Run the Launch File

Create a launch file called `turtlebot_sim.launch` and modify it as follows:
```xml
<launch>
  <!-- Launch Gazebo with the TurtleBot3 world -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find turtlebot3_gazebo)/worlds/turtlebot3_world.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
  </include>

  <!-- Spawn TurtleBot3 model in Gazebo -->
  <param name="robot_description" command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_burger.urdf.xacro" />
  <node pkg="gazebo_ros" type="spawn_model" name="spawn_urdf" 
        args="-urdf -model turtlebot3_burger -x -2.0 -y -0.5 -z 0.0 -param robot_description" />
</launch>
```
---

### Step 5: Run the TurtleBot3 Simulation

Start the simulation in Gazebo by running:
```

roslaunch turtlebot_sim turtlebot_sim.launch
```
---

### Step 6: Prepare and Run the VRPN Simulation Script

Place the `optitrack_sim.py` script in your ROS workspace:
```

mkdir -p ~/catkin_ws/src/optitrack_sim/src
```
# Copy optitrack_sim.py here

Then, run the Python simulation script:
```
python optitrack_sim.py
```
---

### Step 7: Test and Verify the Setup

Ensure the `/vrpn/Body1/odom` topic is being published by running:
```
rostopic list
```
Inspect the pose data:
```
rostopic echo /vrpn/Body1/odom
```
---

### Step 8: Visualize in RViz

Open RViz and add the appropriate TF frames and pose topics:
```
rosrun rviz rviz
```
---

## Troubleshooting

1. **X Server Issues**:  
   Ensure your X server on Windows (e.g., VcXsrv) is running and properly configured.

2. **Networking**:  
   If using WSL, use your Windows machine’s IP (`10.0.x.x`) instead of `127.0.0.1` when necessary.
