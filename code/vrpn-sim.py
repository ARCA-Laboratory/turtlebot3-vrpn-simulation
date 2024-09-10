#!/usr/bin/env python

"""
OptiTrack VRPN Simulator for Simulating Rigid Body Movements and Publishing Velocities

This script simulates the movement of multiple rigid bodies in a predefined 2D space. 
It publishes their odometry and velocity commands (in the /cmd_vel topic) that can be used to control robots 
in Gazebo or other robotic systems.

This script can also be easily modified to integrate real-world VRPN data from the OptiTrack system.

Main features:
- Simulates rigid bodies with customizable movement bounds and velocity.
- Calculates and publishes velocity commands based on changes in position (to /cmd_vel).
- Publishes the transform from OptiTrack to the robot's `base_footprint` frame.
- Can be easily adapted to integrate with real-world robots by adjusting VRPN data input.

"""

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
import tf
import math

# Parameters for movement bounds and speed (can be modified based on robot's environment)
x_min, x_max = 0.0, 5.0  # X-axis bounds for simulation area
y_min, y_max = 0.0, 5.0  # Y-axis bounds for simulation area
speed = 0.05  # Speed of movement per update (adjust based on your requirements)

# To track previous positions for velocity calculation (used for velocity command generation)
previous_positions = {}

def initialize_bodies(num_bodies):
    """
    Initialize the positions and velocities for a given number of rigid bodies.
    Positions are spaced evenly within the defined bounds, and velocity alternates direction.

    Args:
        num_bodies (int): Number of rigid bodies to simulate.

    Returns:
        dict: Dictionary of body positions and velocities.
    """
    bodies = {}
    spacing_x = (x_max - x_min) / num_bodies  # Evenly space bodies along the x-axis
    spacing_y = (y_max - y_min) / num_bodies  # Evenly space bodies along the y-axis

    for i in range(1, num_bodies + 1):
        x_start = x_min + spacing_x * i  # Start position on x-axis
        y_start = y_min + spacing_y * i  # Start position on y-axis
        vx = speed if i % 2 == 0 else -speed  # Alternate velocities for different bodies
        vy = speed if i % 2 == 1 else -speed
        bodies['Body{}'.format(i)] = {'x': x_start, 'y': y_start, 'vx': vx, 'vy': vy}
        previous_positions['Body{}'.format(i)] = (x_start, y_start)  # Store initial positions
    return bodies

def calculate_velocity(body_name, x, y, dt):
    """
    Calculate the velocity based on the change in position over time.

    Args:
        body_name (str): The name of the rigid body.
        x (float): Current x position.
        y (float): Current y position.
        dt (float): Time step between position updates.

    Returns:
        float, float: Calculated velocities along the x and y axes.
    """
    previous_x, previous_y = previous_positions[body_name]
    velocity_x = (x - previous_x) / dt if dt > 0 else 0.0
    velocity_y = (y - previous_y) / dt if dt > 0 else 0.0
    previous_positions[body_name] = (x, y)  # Update previous position for next calculation
    return velocity_x, velocity_y

def publish_cmd_vel(pub, vel_x, vel_y):
    """
    Publish velocity commands (Twist) to control the robot in Gazebo or real-world.

    Args:
        pub (rospy.Publisher): ROS publisher for the /cmd_vel topic.
        vel_x (float): Linear velocity in x-direction.
        vel_y (float): Linear velocity in y-direction (if supported by the robot).
    """
    twist_msg = Twist()
    twist_msg.linear.x = vel_x
    twist_msg.linear.y = vel_y
    twist_msg.angular.z = 0.0  # Modify if angular velocity is required
    pub.publish(twist_msg)

def simulate_vrpn_data(num_bodies):
    """
    Main loop to simulate VRPN data and publish pose and velocity commands.

    Args:
        num_bodies (int): Number of rigid bodies to simulate.
    """
    bodies = initialize_bodies(num_bodies)

    # Publishers for each body's odometry and a general publisher for /cmd_vel
    pubs = {'Body{}'.format(i): rospy.Publisher('/vrpn/Body{}/odom'.format(i), Odometry, queue_size=10) for i in range(1, num_bodies + 1)}
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.init_node('simulate_vrpn_data', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    # Fixed orientation (can be modified for 3D simulation)
    orientation = tf.transformations.quaternion_from_euler(0, 0, 0)

    # Initialize TF broadcaster to publish transforms
    broadcaster = tf.TransformBroadcaster()

    while not rospy.is_shutdown():
        dt = rate.sleep_dur.to_sec()  # Time step between updates

        for body_name, body_data in bodies.items():
            # Update position based on velocity
            body_data['x'] += body_data['vx']
            body_data['y'] += body_data['vy']

            # Reverse direction if out of bounds
            if body_data['x'] > x_max or body_data['x'] < x_min:
                body_data['vx'] = -body_data['vx']
            if body_data['y'] > y_max or body_data['y'] < y_min:
                body_data['vy'] = -body_data['vy']

            # Calculate and publish velocity to the robot (in Gazebo or real-world)
            vel_x, vel_y = calculate_velocity(body_name, body_data['x'], body_data['y'], dt)
            publish_cmd_vel(cmd_vel_pub, vel_x, vel_y)

            # Publish transform from optitrack to base_footprint (used in RViz and robot localization)
            broadcaster.sendTransform(
                (body_data['x'], body_data['y'], 0.0),
                tf.transformations.quaternion_from_euler(0, 0, 0),
                rospy.Time.now(),
                "base_footprint",  # Change to the robot's frame
                "optitrack"        # Parent frame (global)
            )

        rate.sleep()

if __name__ == '__main__':
    try:
        # Number of rigid bodies can be specified at runtime
        num_bodies = int(raw_input("Enter the number of rigid bodies to simulate: "))
        simulate_vrpn_data(num_bodies)
    except rospy.ROSInterruptException:
        pass
