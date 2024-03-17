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
        self.pid = PID(P=0.17, I=0.020, D=0.0, MAX_OUPUT=1.0, MIN_OUTPUT=-1.0)
        self.pid.setWindup(1.0)
        self.pid.SetPoint = 0.0  # Desired yaw angle
        self.pid.setSampleTime(0.02)

        self.yaw_pub = 1

        # self.distR = 0
        self.distL = 0
        self.rate = rospy.Rate(40)

        self.vel = Twist()
        self.vel.angular.x = 1 
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.yaw_sub = rospy.Subscriber('theta', Float32, self.yaw_callback)
        # self.ultraR_sub = rospy.Subscriber("distR", Float32, self.ultraR_callback)
        self.ultraL_sub = rospy.Subscriber("distL", Float32, self.ultraL_callback)
        self.yaw_rate = 0.0

        self.error_tolerance = 4
        self.yaw = 0
        self.yaw_angle = 0
    
    def DecisionMaking(self):
        self.Align()
        # self.Approach()
    
    def Approach(self):
        self.yaw_pub = 1
        self.setNewAngle(self.yaw_angle)
        min_dist = float("inf")
        while dist > 25:
            self.set_vel(0.3, 0, 1)
            self.rate.sleep()
            dist = (self.distR + self.distL)/2
        
        self.stop()
        rospy.loginfo("done")
        for i in range(10):
            self.rate.sleep()
            self.vel_pub.publish(self.vel)
    
    def Align(self):
        turn_angle = 50
        self.setNewAngle(turn_angle)

        min_dist = float("inf")
        yaw_angle = 0
        while self.yaw < self.wrapAngle(turn_angle-5) and self.yaw > self.wrapAngle(turn_angle+5) :
            if self.distL < min_dist:
                min_dist = self.distL
                yaw_angle = self.yaw
            self.rate.sleep()
        print("min yaw: "+str(yaw_angle))
        self.stop()
        time.sleep(1)
        
        # self.start()
        # self.setNewAngle(0)
        # while self.yaw > self.wrapAngle(10) or self.yaw < self.wrapAngle(350):
        #     if self.distL < min_dist:
        #         min_dist = self.distL
        #         yaw_angle = self.yaw
        #     self.rate.sleep()
        # print("min yaw: "+str(yaw_angle))
        # self.stop()
        # time.sleep(1)

        # self.start()
        # self.setNewAngle(-turn_angle)
        # while self.yaw > self.wrapAngle(-turn_angle+10) or self.yaw < self.wrapAngle(-turn_angle-10):
        #     if self.distL < min_dist:
        #         min_dist = self.distL
        #         yaw_angle = self.yaw
        #     self.rate.sleep()
        # print("min yaw: "+str(yaw_angle))
        # self.stop()
        # time.sleep(1)

        # self.start()
        # self.setNewAngle(yaw_angle)
        # self.yaw_angle = yaw_angle
        # while self.yaw > self.wrapAngle(yaw_angle+5) or self.yaw < self.wrapAngle(yaw_angle-5):
        #     if self.distL < min_dist:
        #         min_dist = self.distL
        #         yaw_angle = self.yaw
        #     self.rate.sleep()
        # print("MIN YAW: "+str(yaw_angle))
        # self.stop()
        # time.sleep(1)

        # rospy.loginfo("done")
        # for i in range(10):
        #     self.rate.sleep()
        #     self.vel_pub.publish(self.vel)
    
    def start(self):
        self.set_vel(0,0,1)
        self.pid.clear()
        self.yaw_pub = 1
    
    def stop(self):
        self.set_vel(0,0,0)
        self.pid.clear()
        self.yaw_pub = 0
        self.vel.angular.z = 0
        self.vel_pub.publish(self.vel)
    
    def set_vel(self, lin_x, lin_y, halt):
        self.vel.angular.x = halt
        self.vel.linear.x = lin_x
        self.vel.linear.y = lin_y

    def yaw_callback(self, msg):
        # Convert yaw angle from degrees to radians
        self.yaw = msg.data / 3.14159265359 * 180
        # Update PID controller
        if self.yaw_pub == 1:
            error = self.pid.SetPoint - self.yaw
            while error > 180:
                error -= 360
            while error < -180:
                error += 360
            self.pid.update(error)
            self.yaw_rate = self.pid.output
            self.vel.angular.z = self.yaw_rate
            self.vel_pub.publish(self.vel)

    def ultraR_callback(self,msg):
        self.distR = msg.data

    def ultraL_callback(self,msg):
        self.distL = msg.data
    
    def setNewAngle(self, setPoint):
        self.pid.clear()
        if self.yaw_pub == 1:
            self.pid.setNewGoal(self.wrapAngle(setPoint))
    
    def wrapAngle(self, angle):
        while angle > 360:
            angle -= 360
        while angle < 0:
            angle += 360
        return angle


if __name__=="__main__":
    rospy.init_node("RobotControl")
    robot_control = RobotControl();
    robot_control.DecisionMaking();

    rospy.spin()

