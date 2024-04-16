#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
import time

def move_robot(linear_x: float, linear_y: float, angular_z: float):
    # Create a Twist message and set x, y, and z linear velocities
    move_cmd = Twist()
    move_cmd.linear.x = linear_x  # Move forward at 0.5 m/s
    move_cmd.linear.y = linear_y
    move_cmd.linear.z = 0.0

    # Set angular velocities to zero (no rotation)
    move_cmd.angular.x = 0.0
    move_cmd.angular.y = 0.0
    move_cmd.angular.z = angular_z
    return move_cmd


if __name__ == '__main__':
    # Initialize a new ROS node named 'robot_mover'
    rospy.init_node('robot_mover', anonymous=True)

    # Create a Publisher object, that will publish on the /cmd_vel topic
    # messages of type Twist
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # move the robot forward
    vel = move_robot(0.5, 0, 0)
    pub.publish(vel)
    time.sleep(1)

    # move the robot left
    vel = move_robot(0, 0.5, 0)
    pub.publish(vel)
    time.sleep(1)

    # turn the robot anti-clockwise
    vel = move_robot(0, 0, 0.5)
    pub.publish(vel)
    time.sleep(1)
