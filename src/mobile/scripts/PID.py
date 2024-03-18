#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

import rospy

class PID:
    def __init__(self, P=0.2, I=0.0, D=0.0, MAX_OUPUT=100, MIN_OUTPUT=-100):
        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.max_out = MAX_OUPUT
        self.min_out = MIN_OUTPUT

        self.sample_time = 0.00
        self.current_time = rospy.get_time()
        self.last_time = self.current_time

        self.clear()
    
    def setNewGoal(self, setPoint):
        self.SetPoint = setPoint

    def clear(self):
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        # Windup Guard
        self.output = 0.0

    def update(self, error):
        self.current_time = rospy.get_time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            # if delta_time > 0:
            #     self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

            if self.output > self.max_out:
                self.output = self.max_out
            elif self.output < self.min_out:
                self.output = self.min_out

    def setWindup(self, windup):
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        self.sample_time = sample_time


class YawController:
    def __init__(self):
        self.pid = PID(P=0.17, I=0.02, D=0.0, MAX_OUPUT=1.5, MIN_OUTPUT=-1.5)
        self.pid.setWindup(1.0)
        self.pid.SetPoint = 0.0  # Desired yaw angle
        self.pid.setSampleTime(0.02)

        self.vel = Twist()
        self.vel.angular.x = 1 
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.yaw_sub = rospy.Subscriber('theta', Float32, self.yaw_callback)

        self.yaw_rate = 0.0

    def yaw_callback(self, msg):
        # Convert yaw angle from degrees to radians
        yaw = msg.data / 3.14159265359 * 180

        error = self.pid.SetPoint - yaw
        # Handle angle wrap-around
        while error > 180:
            error -= 360
        while error < -180:
            error += 360

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


if __name__ == '__main__':
    rospy.init_node('yaw_controller', anonymous=True)
    controller = YawController()
    vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 5)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        angle = float(input("input an angle: "))
        controller.setNewGoal(angle)

        rate.sleep()


