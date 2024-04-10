#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

double x = 0.0;
double y = 0.0;
double th = 0.0;

void xCallback(const std_msgs::Float32::ConstPtr& msg) {
    x = msg->data;
}

void yCallback(const std_msgs::Float32::ConstPtr& msg) {
    y = msg->data;
}

void thetaCallback(const std_msgs::Float32::ConstPtr& msg) {
    th = msg->data;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry_publisher");

    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    ros::Subscriber x_sub = n.subscribe("x", 100, xCallback);
    ros::Subscriber y_sub = n.subscribe("y", 100, yCallback);
    ros::Subscriber theta_sub = n.subscribe("theta", 100, thetaCallback);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    nav_msgs::Odometry odom;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    ros::Rate r(20.0);
    while(n.ok()){
        ros::spinOnce();              

        // first, we'll publish the transform over tf
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);

        // send the transform
        odom_broadcaster.sendTransform(odom_trans);

        // next, we'll publish the odometry message over ROS
        odom.header.stamp = ros::Time::now();

        // set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);

        // set the velocity
        odom.twist.twist.linear.x = 0;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = 0;

        // publish the message
        odom_pub.publish(odom);

        r.sleep();
    }
}