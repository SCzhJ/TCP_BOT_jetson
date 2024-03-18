#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from PID import PID
import time
import copy
import numpy as np

import rospy

class MovingAverage:
    def __init__(self, windowSize):
        if windowSize < 3:
            raise ValueError("windowSize must be at least 3")
        self.windowSize =windowSize
        self.data = []
        for i in range(windowSize):
            self.data.append(0)
    def add(self, value):
        self.data.pop(0)
        self.data.append(value)
        return (sum(self.data)-max(self.data)-min(self.data))/(self.windowSize-2)

class RobotControl:
    def __init__(self):
        self.pid = PID(P=0.15, I=0.07, D=0.0, MAX_OUPUT=1.3, MIN_OUTPUT=-1.3)
        self.pid.setWindup(1.3)
        self.pid.SetPoint = 0.0  # Desired yaw angle
        self.pid.setSampleTime(0.02)

        self.vel = Twist()
        self.vel.angular.x = 1 
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.yaw_sub = rospy.Subscriber('theta', Float32, self.yaw_callback)

        self.ultraL_sub = rospy.Subscriber("distL", Float32, self.ultraL_callback)
        self.ultraL_mv = MovingAverage(4)
        self.distL = float("inf")
        self.ultraR_sub = rospy.Subscriber("distR", Float32, self.ultraR_callback)
        self.ultraR_mv = MovingAverage(4)
        self.distR = float("inf")

        self.dt = 0.05
        self.rate = rospy.Rate(1/self.dt)

        self.yaw = None
        self.yaw_angle = 0
        self.yaw_pub = 0
        self.min_dist = float("inf")

        self.to_left = None

        self.set_dist = 10
    def ultraL_callback(self,msg):
        if msg.data > 130:
            pass
        elif msg.data > 100:
            self.distL = self.ultraL_mv.add(100)
        else:
            self.distL = self.ultraL_mv.add(msg.data)
    def ultraR_callback(self,msg):
        if msg.data > 130:
            pass
        elif msg.data > 100:
            self.distR = self.ultraR_mv.add(100)
        else:
            self.distR = self.ultraR_mv.add(msg.data)

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
            dist = (self.distL + self.distR)/2
            if dist < min_dist:
                min_dist = dist
                yaw_angle = self.yaw
            self.rate.sleep()
        print("min yaw: "+str(yaw_angle))
        print("min dist: "+str(min_dist))

        time.sleep(1)
        print("end")

        self.setNewGoal(1)
        print("begin")
        while self.yaw > self.wrapAngle(355) or self.yaw < self.wrapAngle(5) :
            dist = (self.distL + self.distR)/2
            if dist < min_dist:
                min_dist = dist
                yaw_angle = self.yaw
            self.rate.sleep()
            rospy.loginfo(self.yaw)
        print("min yaw: "+str(yaw_angle))
        print("min dist: "+str(min_dist))

        self.setNewGoal(-turn_angle)

        while self.yaw < self.wrapAngle(-turn_angle-5) or self.yaw > self.wrapAngle(-turn_angle + 5) :
            dist = (self.distL + self.distR)/2
            if dist < min_dist:
                min_dist = dist
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
        self.yaw_pub = 0
        ultra_PID = PID(P=0.2, I=0.001, D=0.0, MAX_OUPUT=1.1, MIN_OUTPUT=-1.1)
        ultra_PID.clear()
        ultra_PID.setWindup(1.1)
        ultra_PID.setSampleTime(0.05)
        time.sleep(1)

        self.vel.angular.x = 1
        self.vel.linear.y = 0
        self.vel.linear.x = 0.25
        while self.distL > 35 and (not rospy.is_shutdown()):
            print(self.distL)
            self.vel.angular.x = 1
            yaw_rate = self.updateUltraPIDturn(ultra_PID)
            self.vel.angular.z = yaw_rate
            if yaw_rate > 0.1:
                self.vel.angular.z += 0.05
            if yaw_rate < -0.1:
                self.vel.angular.z -= 0.05
            self.vel_pub.publish(self.vel)
            self.rate.sleep()
            self.vel_pub.publish(self.vel)
            self.rate.sleep()
        rospy.loginfo("End")

        self.yaw_pub = 0
        self.vel.linear.x = 0
        self.vel.linear.y = 0
        self.vel.angular.x = 0
        self.vel.angular.z = 0
        for i in range(10):
            self.rate.sleep()
            self.vel_pub.publish(self.vel)
    
    def Rotate(self):
        time.sleep(1)
        self.yaw_pub = 1

        self.setNewGoal(90)
        rospy.loginfo("turn to 90")
        while not (self.yaw < 95 and self.yaw > 85):
            self.rate.sleep()

        time.sleep(2)
        self.setNewGoal(0)
        rospy.loginfo("turn to 0")
        while not ((self.yaw < 5 and self.yaw >=0) or (self.yaw <= 360 and self.yaw > 355)):
            self.rate.sleep()

        self.setNewGoal(-90)
        rospy.loginfo("turn to -90")
        while not (self.yaw < 275 and self.yaw >= 265):
            self.rate.sleep()

        self.setNewGoal(180)
        rospy.loginfo("turn to 180")
        while not (self.yaw < 185 and self.yaw >= 175):
            self.rate.sleep()

        time.sleep(2)

        self.setNewGoal(90)
        rospy.loginfo("turn to 90")
        while not (self.yaw < 95 and self.yaw >= 85):
            self.rate.sleep()

        self.setNewGoal(0)
        rospy.loginfo("turn to 0")
        while not ((self.yaw < 5 and self.yaw >=0) or (self.yaw <= 360 and self.yaw > 355)):
            self.rate.sleep()
        
        time.sleep(2)
    
    def Shift(self):
        self.yaw_pub = 0
        ultra_PID = PID(P=0.4, I=0.002, D=0.0, MAX_OUPUT=1.4, MIN_OUTPUT=-1.4)
        ultra_PID.clear()
        ultra_PID.setWindup(1.4)
        ultra_PID.setSampleTime(0.05)

        ultra_PID_dist = PID(P=0.03, I=0.003, D=0.0, MAX_OUPUT=0.6, MIN_OUTPUT=-0.6)
        ultra_PID_dist.clear()
        ultra_PID_dist.setWindup(0.50)
        ultra_PID_dist.setSampleTime(0.05)
        self.vel.angular.x = 1
        self.vel.linear.x = 0
        self.vel.linear.y = 0.3
        time = 0
        while not rospy.is_shutdown():
            time += self.dt
            yaw_rate = self.updateUltraPIDturn(ultra_PID)
            self.vel.angular.x = 1
            self.vel.angular.z = yaw_rate
            if yaw_rate > 0.1:
                self.vel.angular.z += 0.05
            if yaw_rate < -0.1:
                self.vel.angular.z -= 0.05

            lin_x = self.updateUltraPIDdist(ultra_PID_dist)
            self.vel.linear.x = lin_x

            # rospy.loginfo(yaw_rate)
            rospy.loginfo(lin_x)
            self.vel_pub.publish(self.vel)
            self.rate.sleep()
            self.vel_pub.publish(self.vel)

            self.rate.sleep()
            if time > 2:
                break

        self.yaw_pub = 0
        self.vel.linear.x = 0
        self.vel.linear.y = 0
        self.vel.angular.x = 0
        self.vel.angular.z = 0
        for i in range(10):
            self.rate.sleep()
            self.vel_pub.publish(self.vel)

        # ultra_PID_dist = PID(P=0.1, I=0.02, D=0.0, MAX_OUPUT=1.5, MIN_OUTPUT=-1.5)
    
    def updateUltraPIDturn(self, ultra_PID):
        error = self.distR - self.distL
        ultra_PID.update(error)
        return ultra_PID.output

    def updateUltraPIDdist(self, ultra_PID):
        error = (self.distL + self.distR)/2 - self.set_dist
        ultra_PID.update(error)
        return ultra_PID.output
    
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
    # robot_control.Approach()
    # robot_control.Rotate()

    rospy.spin()

