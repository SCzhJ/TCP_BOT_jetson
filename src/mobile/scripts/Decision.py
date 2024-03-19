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
        self.pid = PID(P=0.15, I=0.03, D=0.0, MAX_OUPUT=1.3, MIN_OUTPUT=-1.3)
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

        self.LDRR_sub = rospy.Subscriber("LDRR", Float32, self.LDRR_cb)
        self.LDRR = 100
        self.LDRL_sub = rospy.Subscriber("LDRL", Float32, self.LDRL_cb)
        self.LDRL = 100

        self.dt = 0.05
        self.rate = rospy.Rate(1/self.dt)

        self.yaw = None
        self.yaw_angle = 0
        self.yaw_pub = 0
        self.min_dist = float("inf")

        self.to_left = None

        self.set_dist = 9

    def dist_metric_toRight(self):
        return self.distL + self.distR * 1.15 + (abs(self.distL-self.distR)*100) ** 3
        # return 2*abs(self.distL-self.distR)
    def dist_metric_toLeft(self):
        return self.distL * 1.15 + self.distR + (abs(self.distL-self.distR)*100) ** 3
        # return 2*abs(self.distL-self.distR)
    
    def LDRR_cb(self, msg):
        self.LDRR = msg.data
    def LDRL_cb(self, msg):
        self.LDRL = msg.data

    def ultraL_callback(self,msg):
        if msg.data > 150:
            pass
        elif msg.data > 120:
            self.distL = self.ultraL_mv.add(120)
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
        time.sleep(1)
        self.yaw_pub = 1
        rospy.loginfo("align")
        turn_angle = 60
        min_dist = float("inf")
        yaw_angle = 0

        self.setNewGoal(turn_angle)
        rospy.loginfo("set turn angle")
        while self.yaw < self.wrapAngle(turn_angle-10) or self.yaw > self.wrapAngle(turn_angle+10) :
            dist = self.dist_metric_toLeft()
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
            dist = self.dist_metric_toRight()
            if dist < min_dist:
                min_dist = dist
                yaw_angle = self.yaw
            self.rate.sleep()
            rospy.loginfo(self.yaw)
        print("min yaw: "+str(yaw_angle))
        print("min dist: "+str(min_dist))

        self.setNewGoal(-turn_angle)

        while self.yaw < self.wrapAngle(-turn_angle-5) or self.yaw > self.wrapAngle(-turn_angle + 5) :
            dist = self.dist_metric_toLeft()
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
        print("to left:")
        print(self.to_left)
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
        while self.distL > 31 and (not rospy.is_shutdown()):
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
        self.yaw_angle = self.yaw
    
    def Rotate(self):
        time.sleep(1)
        self.yaw_pub = 1

        self.setNewGoal(self.yaw_angle - 90)
        rospy.loginfo("turn to -90")
        while self.withInAngle(self.yaw_angle - 90, 5, self.yaw) == False:
            self.rate.sleep()

        time.sleep(5)
        self.setNewGoal(self.yaw_angle + 0)
        rospy.loginfo("turn to 0")
        while self.withInAngle(self.yaw_angle + 0, 5, self.yaw) == False:
            self.rate.sleep()

        self.setNewGoal(self.yaw_angle+90)
        rospy.loginfo("turn to 90")
        while self.withInAngle(self.yaw_angle + 90, 5, self.yaw) == False:
            self.rate.sleep()

        self.setNewGoal(self.yaw_angle+180)
        rospy.loginfo("turn to 180")
        while self.withInAngle(self.yaw_angle+180, 5, self.yaw) == False:
            self.rate.sleep()

        time.sleep(5)

        self.setNewGoal(self.yaw_angle+90)
        rospy.loginfo("turn to 90")
        while self.withInAngle(self.yaw_angle+90, 5, self.yaw) == False:
            self.rate.sleep()

        self.setNewGoal(self.yaw_angle + 0)
        rospy.loginfo("turn to 0")
        while self.withInAngle(self.yaw_angle + 0, 5, self.yaw) == False:
            self.rate.sleep()
        
        time.sleep(5)
    
    def Shift(self):
        self.yaw_pub = 0
        ultra_PID = PID(P=0.2, I=0.001, D=0.0, MAX_OUPUT=1.1, MIN_OUTPUT=-1.1)
        ultra_PID.clear()
        ultra_PID.setWindup(1.1)
        ultra_PID.setSampleTime(0.05)
        time.sleep(1)

        self.vel.angular.x = 1
        self.vel.linear.y = 0
        self.vel.linear.x = 0.10
        while self.distL > 15 and (not rospy.is_shutdown()):
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
        ultra_PID = PID(P=0.4, I=0.002, D=0.0, MAX_OUPUT=1.4, MIN_OUTPUT=-1.4)
        ultra_PID.clear()
        ultra_PID.setWindup(1.4)
        ultra_PID.setSampleTime(0.05)

        ultra_PID_dist = PID(P=0.025, I=0.004, D=0.0, MAX_OUPUT=0.7, MIN_OUTPUT=-0.7)
        ultra_PID_dist.clear()
        ultra_PID_dist.setWindup(0.60)
        ultra_PID_dist.setSampleTime(0.05)
        self.vel.angular.x = 1
        self.vel.linear.x = 0
        # print("to left:")
        # print(self.to_left)
        # if self.to_left == True:
        #     self.vel.linear.y = 0.18
        # else:
        #     self.vel.linear.y = -0.18


        acc_time = 0
        right_time = 6
        left_time = 14
        end = False
        self.set_dist = 12

        while not rospy.is_shutdown() and (not end):
            self.vel.linear.y = -0.2
            while not rospy.is_shutdown() and (not end):
                acc_time += self.dt * 2
                yaw_rate = self.updateUltraPIDturn(ultra_PID)
                self.vel.angular.x = 1
                self.vel.angular.z = yaw_rate
                if yaw_rate > 0.1:
                    self.vel.angular.z += 0.05
                if yaw_rate < -0.1:
                    self.vel.angular.z -= 0.05
                lin_x = self.updateUltraPIDdist(ultra_PID_dist)
                self.vel.linear.x = lin_x
                self.vel_pub.publish(self.vel)
                self.rate.sleep()
                self.vel_pub.publish(self.vel)
                self.rate.sleep()

                end = self.EndCondition()
                if end == True:
                    break
                if acc_time > right_time:
                    acc_time = 0
                    right_time -= 0.2
                    break
            if end:
                print("success")
                break
            if right_time <= 0:
                print("timeout")
                break

            self.vel.linear.y = 0.2
            while not rospy.is_shutdown() and (not end):
                acc_time += self.dt * 2
                yaw_rate = self.updateUltraPIDturn(ultra_PID)
                self.vel.angular.x = 1
                self.vel.angular.z = yaw_rate
                if yaw_rate > 0.1:
                    self.vel.angular.z += 0.05
                if yaw_rate < -0.1:
                    self.vel.angular.z -= 0.05
                lin_x = self.updateUltraPIDdist(ultra_PID_dist)
                self.vel.linear.x = lin_x
                self.vel_pub.publish(self.vel)
                self.rate.sleep()
                self.vel_pub.publish(self.vel)
                self.rate.sleep()

                end = self.EndCondition()
                if acc_time > left_time:
                    acc_time = 0
                    left_time -= 0.5
                    break
            if end:
                print("success")
                break
            if left_time <= 0:
                print("timeout")
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
    def Adjust(self):
        self.yaw_pub = 0
        ultra_PID = PID(P=0.33, I=0.0025, D=0.0, MAX_OUPUT=1.4, MIN_OUTPUT=-1.4)
        ultra_PID.clear()
        ultra_PID.setWindup(1.4)
        ultra_PID.setSampleTime(0.05)

        ultra_PID_dist = PID(P=0.025, I=0.004, D=0.0, MAX_OUPUT=0.7, MIN_OUTPUT=-0.7)
        ultra_PID_dist.clear()
        ultra_PID_dist.setWindup(0.60)
        ultra_PID_dist.setSampleTime(0.05)
        self.vel.angular.x = 1
        self.vel.linear.x = 0

        acc_time = 0
        set_time = 7
        self.set_dist = 7.5

        while not rospy.is_shutdown():
            acc_time += self.dt * 2
            yaw_rate = self.updateUltraPIDturn(ultra_PID)
            self.vel.angular.x = 1
            self.vel.angular.z = yaw_rate
            if yaw_rate > 0.1:
                self.vel.angular.z += 0.05
            if yaw_rate < -0.1:
                self.vel.angular.z -= 0.05
            lin_x = self.updateUltraPIDdist(ultra_PID_dist)
            self.vel.linear.x = lin_x
            self.vel_pub.publish(self.vel)
            self.rate.sleep()
            self.vel_pub.publish(self.vel)
            self.rate.sleep()

            if acc_time > set_time:
                acc_time = 0
                break

        self.yaw_pub = 0
        self.vel.linear.x = 0
        self.vel.linear.y = 0
        self.vel.angular.x = 0
        self.vel.angular.z = 0
        for i in range(10):
            self.rate.sleep()
            self.vel_pub.publish(self.vel)
    
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
    
    def EndCondition(self):
        print("left: " + str(self.LDRL))
        print("rigt: " + str(self.LDRR))
        # print(self.LDRR+self.LDRL)
        if self.LDRR + self.LDRL > 485:
            return True
        else:
            return False

    def wrapAngle(self, angle):
        while angle > 360:
            angle -= 360
        while angle < 0:
            angle += 360
        return angle
    
    def withInAngle(self, target_angle, error_tolerance, current_angle):
        # Ensure the target angle is between 0 and 360
        target_angle = target_angle % 360

        # Calculate the lower and upper bounds
        lower_bound = (current_angle - error_tolerance) % 360
        upper_bound = (current_angle + error_tolerance) % 360

        # Check if the target angle is within the error tolerance
        if lower_bound <= upper_bound:
            return lower_bound <= target_angle <= upper_bound
        else:  # The range crosses the 0/360 boundary
            return target_angle >= lower_bound or target_angle <= upper_bound


if __name__=="__main__":
    rospy.init_node("RobotControl")
    rospy.loginfo("begin")
    robot_control = RobotControl();
    rospy.loginfo("Decision")
    robot_control.Align()
    robot_control.Approach()
    robot_control.Rotate()
    robot_control.Shift()
    robot_control.Adjust()
    # while not rospy.is_shutdown():
    #     robot_control.EndCondition()
    #     time.sleep(0.2)

    rospy.spin()
