#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
const float PI = 3.14159265358979323846;

int main(int argc, char** argv){
    std::string node_name = "real_robot_transform";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    tf2_ros::TransformBroadcaster broadcaster;

    geometry_msgs::TransformStamped transformStamped1;
    transformStamped1.header.frame_id = "base_link";
    transformStamped1.child_frame_id = "laser";
    transformStamped1.transform.translation.x = 0.07;
    transformStamped1.transform.translation.y = 0;
    transformStamped1.transform.translation.z = 0;
    transformStamped1.transform.rotation.x = 0;
    transformStamped1.transform.rotation.y = 0;
    transformStamped1.transform.rotation.z = 0;
    transformStamped1.transform.rotation.w = 1;

    ros::Rate rate(20.0);
    while (nh.ok()){
        transformStamped1.header.stamp = ros::Time::now();
        broadcaster.sendTransform(transformStamped1);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
