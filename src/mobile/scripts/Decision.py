#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from PID import PID
import time
import copy
import numpy as np

import rospy

class RobotControl:
    def __init__(self):
        self.pid = PID(P=0.22, I=0.20, D=0.0, MAX_OUPUT=1.5, MIN_OUTPUT=-1.5)
        self.pid.setWindup(1.0)
        self.pid.SetPoint = 0.0  # Desired yaw angle
        self.pid.setSampleTime(0.02)

        self.vel = Twist()
        self.vel.angular.x = 1 
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.yaw_sub = rospy.Subscriber('theta', Float32, self.yaw_callback)

        self.ultraL_sub = rospy.Subscriber("distL", Float32, self.ultraL_callback)
        self.distL = float("inf")

        self.rate = rospy.Rate(20)

        self.yaw = None
        self.yaw_angle = None
        self.yaw_pub = 0
        self.min_dist = float("inf")

        self.to_left = None
    def ultraL_callback(self,msg):
        self.distL = msg.data
        if self.distL < self.min_dist:
            self.min_dist = self.distL
            self.yaw_angle = self.yaw

    def yaw_callback(self, msg):
        # Convert yaw angle from degrees to radians
        yaw = msg.data / 3.14159265359 * 180
        self.yaw = yaw

        error = self.pid.SetPoint - yaw
        # Handle angle wrap-around
        while error > 180:
            error -= 360
        while error < -180:
            error += 360

        if self.yaw_pub == 1:
            # Update PID controller
            self.pid.update(error)

            self.yaw_rate = self.pid.output
            self.vel.angular.z = self.yaw_rate
            # use angular.x as halt message, if not zero,
            # Then robot do not halt
            self.vel_pub.publish(self.vel)
    
    def setNewGoal(self, setPoint):
        while setPoint > 360:
            setPoint -= 360
        while setPoint < 0:
            setPoint += 360
        self.pid.clear()
        self.pid.setNewGoal(setPoint)
    
    def Align(self):
        self.yaw_pub = 1
        rospy.loginfo("align")
        turn_angle = 50
        min_dist = float("inf")
        yaw_angle = 0

        self.setNewGoal(turn_angle)
        rospy.loginfo("set turn angle")
        while self.yaw < self.wrapAngle(turn_angle-10) or self.yaw > self.wrapAngle(turn_angle+10) :
            if self.distL < min_dist:
                min_dist = self.distL
                yaw_angle = self.yaw
            self.rate.sleep()
        print("min yaw: "+str(yaw_angle))
        print("min dist: "+str(min_dist))

        time.sleep(1)
        print("end")

        self.setNewGoal(1)
        print("begin")
        while self.yaw > self.wrapAngle(355) or self.yaw < self.wrapAngle(5) :
            if self.distL < min_dist:
                min_dist = self.distL
                yaw_angle = self.yaw
            self.rate.sleep()
            rospy.loginfo(self.yaw)
        print("min yaw: "+str(yaw_angle))
        print("min dist: "+str(min_dist))

        self.setNewGoal(-turn_angle)

        while self.yaw < self.wrapAngle(-turn_angle-5) or self.yaw > self.wrapAngle(-turn_angle + 5) :
            if self.distL < min_dist:
                min_dist = self.distL
                yaw_angle = self.yaw
            self.rate.sleep()
        print("min yaw: "+str(yaw_angle))
        print("min dist: "+str(min_dist))
        time.sleep(1)

        self.setNewGoal(yaw_angle)
        self.yaw_angle = yaw_angle
        if yaw_angle > 0:
            self.to_left = False
        else:
            self.to_left = True
        time.sleep(5)
    
    def Approach(self):
        self.vel.linear.x = 0.4
        while self.distL > 35:
            self.rate.sleep()
        self.yaw_pub = 0
        self.vel.linear.x = 0
        self.vel.linear.y = 0
        self.vel.angular.z = 0
        for i in range(10):
            self.rate.sleep()
            self.vel_pub.publish(self.vel)
    
    def Rotate(self):
        self.vel.angular.z = 0.8
        for i in range(10):
            self.rate.sleep()
            self.vel_pub.publish(self.vel)
        while self.yaw < self.wrapAngle(300 + self.yaw_angle)  or self.yaw > self.wrapAngle(340+self.yaw_angle):
            self.rate.sleep()
        self.yaw_pub = 1
        self.setNewGoal(self.yaw_angle)
        while self.yaw < self.wrapAngle(self.yaw_angle-5)  or self.yaw > self.wrapAngle(self.yaw_angle+5):
            self.rate.sleep()
        self.yaw_pub = 0
        self.vel.linear.x = 0
        self.vel.linear.y = 0
        self.vel.angular.z = 0
        for i in range(10):
            self.rate.sleep()
            self.vel_pub.publish(self.vel)
        time.sleep(1.5)

        self.vel.angular.z = -0.8
        for i in range(10):
            self.rate.sleep()
            self.vel_pub.publish(self.vel)
        while self.yaw > self.wrapAngle(60 + self.yaw_angle)  or self.yaw < self.wrapAngle(30+self.yaw_angle):
            self.rate.sleep()
        self.yaw_pub = 1
        self.setNewGoal(self.yaw_angle)
        time.sleep(5)
        self.yaw_pub = 0
        self.vel.linear.x = 0
        self.vel.linear.y = 0
        self.vel.angular.z = 0
        for i in range(10):
            self.rate.sleep()
            self.vel_pub.publish(self.vel)
        time.sleep(1.5)
    
    def Shift(self):
        self.yaw_pub = 1
        self.setNewGoal(self.yaw_angle)
        if self.to_left == True:
            self.vel.linear.x = 0
            self.vel.linear.y = 0.6
        else:
            self.vel.linear.x = 0
            self.vel.linear.y = -0.6
        time.sleep(4)
        self.yaw_pub = 0
        self.vel.angular.z = 0
        self.vel.linear.y  =0
        for i in range(10):
            self.rate.sleep()
            self.vel_pub.publish(self.vel)
    
    def DecisionMaking(self):
        self.Align()
        self.Approach()
        self.Rotate()
        self.Shift()
    

    def wrapAngle(self, angle):
        while angle > 360:
            angle -= 360
        while angle < 0:
            angle += 360
        return angle


if __name__=="__main__":
    rospy.init_node("RobotControl")
    rospy.loginfo("begin")
    robot_control = RobotControl();
    rospy.loginfo("Decision")
    robot_control.DecisionMaking();

    rospy.spin()

