# Simulating OptiTrack VRPN with TurtleBot3 in ROS Melodic on WSL (Windows Subsystem for Linux)

This guide provides instructions to simulate an OptiTrack VRPN system with a TurtleBot3 in a ROS Melodic environment running on WSL. It outlines the steps to set up the required software components, modify launch files, and simulate motion data. The guide enables the visualization of TurtleBot3 in Gazebo and RViz using simulated data from an OptiTrack system.

## Prerequisites
- **ROS Melodic** and **Gazebo** installed on your WSL machine.
- **X server** set up on Windows (e.g., VcXsrv or Xming) to run graphical applications (like RViz and Gazebo).

## Steps to Configure and Simulate VRPN with TurtleBot3 and ROS

### Step 1: Set Up Your Catkin Workspace and Clone the TurtleBot3 Repositories

Create a Catkin Workspace (if you haven't already):
Open a terminal in WSL and run:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
Clone the TurtleBot3 Repositories:
Navigate to the src folder of your workspace and run the following commands:
```
cd ~/catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```
Build the Workspace:
Run the following commands:
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
---

### Step 2: Verify the Installation

Launch Gazebo with the TurtleBot3 world to ensure everything is installed correctly:
```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
If the TurtleBot3 robot and world load successfully in Gazebo, the installation is correct.

---

### Step 3: Modify and Run the Launch File

Modify the launch file called `turtlebot3_world.launch` as follows:

```xml
<launch>
  <!-- Original TurtleBot3 Parameters -->
  <arg name="model" default="burger" doc="model type [burger, waffle, waffle_pi]"/>  
  <arg name="x_pos" default="-2.0"/>
  <arg name="y_pos" default="-0.5"/>
  <arg name="z_pos" default="0.0"/>

  <!-- Launch Gazebo with Empty World -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find turtlebot3_gazebo)/worlds/turtlebot3_world.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>

  <!-- Spawn TurtleBot3 in Gazebo -->
  <param name="robot_description" command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro" />
  <node pkg="gazebo_ros" type="spawn_model" name="spawn_urdf" args="-urdf -model turtlebot3_$(arg model) -x $(arg x_pos) -y $(arg y_pos) -z $(arg z_pos) -param robot_description" />

  <!-- Start VRPN (OptiTrack) Simulator -->
  <node pkg="your_package_name" type="optitrack_sim.py" name="vrpn_sim_node" output="screen"/>

  <!-- Static Transform from map to optitrack (assuming map is static) -->
  <node pkg="tf" type="static_transform_publisher" name="map_to_optitrack" args="0 0 0 0 0 0 map optitrack 100" />

  <!-- Disable odom transform publication by TurtleBot -->
  <!-- The base_link will now be handled by the optitrack system -->
  <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" output="screen">
    <param name="publish_frequency" value="50.0" />
    <param name="use_sim_time" value="true" />
    <remap from="/odom" to="/fake_odom" />
  </node>

  <!-- Optional: EKF Localization Node (if you decide to use it for sensor fusion) -->
  <!-- Uncomment if EKF is needed for fusing with other sensors -->
  <!--
  <node pkg="robot_localization" type="ekf_localization_node" name="ekf_localization_node" output="screen">
    <param name="use_sim_time" value="true"/>
    <rosparam command="load" file="/home/tom/Dev/turtlebot_ws/src/turtlebot3/turtlebot3/config/turtlebot3_ekf.yaml"/>
  </node>
  -->
</launch>
```
---

### Step 4: Run the TurtleBot3 Simulation

Start the simulation in Gazebo by running:
```

roslaunch turtlebot3_gazebo turtlebot3_world.launch

```
---

### Step 5: Prepare and Run the VRPN Simulation Script

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

### Step 6: Test and Verify the Setup

Ensure the `/vrpn/Body1/odom` topic is being published by running:
```
rostopic list
```
Inspect the pose data:
```
rostopic echo /vrpn/Body1/odom
```
---

### Step 7: Visualize in RViz

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
