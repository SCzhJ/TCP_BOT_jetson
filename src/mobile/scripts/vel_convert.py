#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

vel = Twist()
def cmd_vel_callback(data):
    global vel
    if data.linear.x < 0 and data.angular.z !=0:
        vel.linear.y = -data.angular.z/abs(data.angular.z) * abs(data.linear.x)
        vel.linear.x = 0
        vel.angular.z = 0
    else:
        vel = data
    # angular.x is halt message, if not zero, then not halt when vel change
    # else halt when vel change
    vel.angular.x = data.angular.x

if __name__ == '__main__':
    rospy.init_node('cmd_vel_listener', anonymous=True)
    rospy.Subscriber("vel", Twist, cmd_vel_callback, queue_size=10)
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(vel)
        rate.sleep()
