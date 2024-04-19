#!/usr/bin/env python3
import rospy
import actionlib
from nav.msg import NavActionAction, NavActionGoal, NavActionFeedback, NavActionResult, NavActionFeedback
import sys
import os
from std_msgs.msg import Int32, Bool

script_path = os.path.abspath(os.path.join(os.path.dirname(__file__),"DWA"))
if script_path not in sys.path:
    sys.path.append(script_path)
rospy.loginfo("Added path: %s", script_path)
from test_util_2 import *

q = [0 for i in range(2)]
q_len = len(q)
q_i = 0
confidence = 0
continue_patrol = False

def OD_cb(msg):
    global confidence, q, q_len, q_i
    q[q_i] = msg.data
    q_i = (q_i + 1) % q_len
    confidence = int(sum(q) / q_len)

def PD_cb(msg):
    global continue_patrol
    continue_patrol = msg.data


def feedback_cb(feedback):
    print('[Feedback] Progress: {0}'.format(feedback.progress_bar))

done = False
def done_cb(state, result):
    global done
    done = True

if __name__ == '__main__':
    rospy.init_node('Decision_Node')

    subOD = rospy.Subscriber("obj_det", Int32, OD_cb, queue_size=10)
    pubPick = rospy.Publisher("pick", Bool, queue_size=10)
    subPickDone = rospy.Subscriber("pickDone", Bool, PD_cb, queue_size=10)

    action_started = False
    # client = actionlib.SimpleActionClient('nav_ctrl', NavActionAction)

    rate = rospy.Rate(20)
    rospy.loginfo("begin")
    while not rospy.is_shutdown():
        if confidence > 50:
            # if action_started == True:
            #     action_started = False
            #     client.cancel_goal()
            rospy.loginfo("Object Found")
            continue_patrol = False
            pubPick.publish(Bool(True))
            pubPick.publish(Bool(True))
            pubPick.publish(Bool(True))
            while continue_patrol == False and not rospy.is_shutdown():
                rate.sleep()
            rospy.loginfo("Object Focused or Lost")
            # action_started = True
        
        # if done == True:
        #     done == False
        #     action_started = True
        #     client.wait_for_server()
        #     goal = NavActionGoal(1,1)
        #     client.send_goal(goal, done_cb=done_cb, feedback_cb=feedback_cb)
        
        rate.sleep()

    # if action_started == True:
    #     client.cancel_goal()


