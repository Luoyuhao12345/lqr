#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "car_info.h"

ros::Publisher car_info_pub;
ros::Subscriber car_pose_sub;
float car_x, car_y, car_yaw, car_v;

void odom_cb(const nav_msgs::Odometry::ConstPtr& pose)
{
    tf::TransformListener tf_listener;
    nav_msgs::Odometry _odom_pose = *pose;
    geometry_msgs::PoseStamped odom_pose;
    geometry_msgs::PoseStamped target_pose;

    car_v = _odom_pose.twist.twist.linear.x;

    odom_pose.header = _odom_pose.header;
    odom_pose.header.stamp = ros::Time();
    odom_pose.pose.position = _odom_pose.pose.pose.position;
    odom_pose.pose.orientation = _odom_pose.pose.pose.orientation;

    tf_listener.waitForTransform("/map", "/odom", ros::Time(0), ros::Duration(0.5)); 
    try
    {
        tf_listener.transformPose("/map", odom_pose, target_pose);
        car_x = target_pose.pose.position.x;
        car_y = target_pose.pose.position.y;

        tf::Quaternion quat;
        double roll, pitch, yaw;
        tf::quaternionMsgToTF(target_pose.pose.orientation, quat);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        car_yaw = yaw;
    }
    catch( tf::TransformException ex)
    {
        ROS_WARN("transfrom exception : %s",ex.what());
        return;
    }

    my_mpc::car_info car_info;
    car_info.car_x = car_x;
    car_info.car_y = car_y;
    car_info.car_yaw = car_yaw;
    car_info.car_v = car_v;
    car_info_pub.publish(car_info);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "car_info_pub_node");
    ros::NodeHandle nh;
    car_pose_sub = nh.subscribe("/odom", 5, &odom_cb);
    car_info_pub = nh.advertise<my_mpc::car_info>("/car_info", 1);
    car_x = car_y = car_yaw = car_v = 0;
    ros::spin();
    return 0;
}