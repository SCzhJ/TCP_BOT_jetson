#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

// Wheel radius and robot radius (distance from center of robot to wheels)
double wheel_radius = 0.035;
double robot_len_wid = 0.105 + 0.08;

ros::Publisher w1_pub, w2_pub, w3_pub, w4_pub;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    double v_x = msg->linear.x;
    double v_y = msg->linear.y;
    double v_theta = msg->angular.z;

    // Calculate wheel speeds based on mecanum wheel kinematics
    double w1 = (v_x - v_y - v_theta * robot_len_wid) / wheel_radius;
    double w2 = (v_x + v_y + v_theta * robot_len_wid) / wheel_radius;
    double w3 = (v_x + v_y - v_theta * robot_len_wid) / wheel_radius;
    double w4 = (v_x - v_y + v_theta * robot_len_wid) / wheel_radius;

    // Publish wheel speeds
    std_msgs::Float32 msg_out;
    msg_out.data = w1;
    w1_pub.publish(msg_out);
    msg_out.data = w2;
    w2_pub.publish(msg_out);
    msg_out.data = w3;
    w3_pub.publish(msg_out);
    msg_out.data = w4;
    w4_pub.publish(msg_out);
}

int main(int argc, char **argv)
{
    std::string node_name = "mecanum_kinematics"; 
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    // if (!nh.getParam("/"+node_name+"/wheel_radius", wheel_radius))
    // {
    //     ROS_ERROR("Failed to retrieve parameter 'wheel_radius'");
    //     return -1;
    // }
    // if (!nh.getParam("/"+node_name+"/robot_len_wid", robot_len_wid))
    // {
    //     ROS_ERROR("Failed to retrieve parameter 'robot_len_wid'");
    //     return -1;
    // }
    

    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 10, cmdVelCallback);
    w1_pub = nh.advertise<std_msgs::Float32>("w1", 10);
    w2_pub = nh.advertise<std_msgs::Float32>("w2", 10);
    w3_pub = nh.advertise<std_msgs::Float32>("w3", 10);
    w4_pub = nh.advertise<std_msgs::Float32>("w4", 10);

    ros::spin();

    return 0;
}