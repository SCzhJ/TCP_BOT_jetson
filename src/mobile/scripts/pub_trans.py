#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import tf
import tf2_ros
import geometry_msgs.msg

class RobotTransformPublisher:
    def __init__(self):
        rospy.init_node("robot_transform_publisher")

        self.x = None
        self.y = None
        self.theta = None

        rospy.Subscriber("x", Float32, self.update_x)
        rospy.Subscriber("y", Float32, self.update_y)
        rospy.Subscriber("theta", Float32, self.update_theta)

        self.br = tf2_ros.TransformBroadcaster()

    def update_x(self, data):
        self.x = data.data

    def update_y(self, data):
        self.y = data.data

    def update_theta(self, data):
        self.theta = data.data

    def publish_transform(self):
        if self.x is not None and self.y is not None and self.theta is not None:
            t = geometry_msgs.msg.TransformStamped()

            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "map"
            t.child_frame_id = "base_link"
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.theta)
            t.transform.rotation.x = quaternion[0]
            t.transform.rotation.y = quaternion[1]
            t.transform.rotation.z = quaternion[2]
            t.transform.rotation.w = quaternion[3]

            self.br.sendTransform(t)

if __name__ == '__main__':
    rtp = RobotTransformPublisher()
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        rtp.publish_transform()
        rate.sleep()