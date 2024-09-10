# Simulating OptiTrack VRPN with TurtleBot3 in ROS Melodic on WSL (Windows Subsystem for Linux)

This guide provides instructions to simulate an OptiTrack VRPN system with a TurtleBot3 in a ROS Melodic environment running on WSL. It outlines the steps to set up the required software components, modify launch files, and simulate motion data. The guide enables the visualization of TurtleBot3 in Gazebo and RViz using simulated data from an OptiTrack system.

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

Modify the launch file called `turtlebot3_world.launch` as follows:

```xml
<launch>
  <arg name="use_simulation" default="true" doc="Set to true to use the simulated OptiTrack node, false for real OptiTrack" />

  <!-- Launch Gazebo with an empty world -->
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

  <!-- Simulated VRPN Node -->
  <group if="$(arg use_simulation)">
    <node pkg="optitrack_sim" type="optitrack_sim.py" name="vrpn_sim_node" output="screen"/>
  </group>

  <!-- Static Transform from map to optitrack (for global positioning) -->
  <node pkg="tf" type="static_transform_publisher" name="map_to_optitrack" args="0 0 0 0 0 0 map optitrack 100" />
</launch>
```
---

### Step 5: Run the TurtleBot3 Simulation

Start the simulation in Gazebo by running:
```

roslaunch turtlebot3_gazebo turtlebot3_world.launch

```
---

### Step 6: Prepare and Run the VRPN Simulation Script

Place the `optitrack_sim.py` script in your ROS workspace:
```

mkdir -p ~/catkin_ws/src/optitrack_sim/src
```
# Copy vrpn-sim.py here

Then, run the Python simulation script:
```
python vrpn-sim.py
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
#### Add the TurtleBot3 Robot Model in RViz:

1. In **RViz**, click **"Add"** on the left-hand panel.
2. Choose **"RobotModel"** from the list of display types and click **OK**.
3. Set the **"Fixed Frame"** to `optitrack` to match the frame where the robot is being tracked.
4. You should now be able to visualize the TurtleBot3 and its movement based on the *simulated* VRPN data.

---

## Troubleshooting

1. **X Server Issues**:  
   Ensure your X server on Windows (e.g., VcXsrv) is running and properly configured.

2. **Networking**:  
   If using WSL, use your Windows machine’s IP (`10.0.x.x`) instead of `127.0.0.1` when necessary.
