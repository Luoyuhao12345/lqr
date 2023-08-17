#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "my_marker.h"


visualization_msgs::Marker marker_rear, marker_front;
ros::Publisher marker_pub;
ros::Subscriber marker_sub;

void marker_cb(const my_mpc::my_marker::ConstPtr& marker_info)
{
    marker_rear.pose.position.x = marker_info->car_x;
    marker_rear.pose.position.y = (*marker_info).car_y;
    marker_front.pose.position.x = marker_info->ref_x;
    marker_front.pose.position.y = (*marker_info).ref_y;
    marker_pub.publish(marker_rear);
    marker_pub.publish(marker_front);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_rviz");
  ros::NodeHandle n;
  marker_sub  =  n.subscribe("/my_marker", 1, &marker_cb);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);


  // 设置该Publisher发布的消息类型
  marker_rear.header.frame_id = "/map";
  marker_rear.header.stamp = ros::Time::now();
  marker_rear.ns = "points";
  marker_rear.id = 0;
  marker_rear.type = visualization_msgs::Marker::SPHERE;
  marker_rear.action = visualization_msgs::Marker::ADD;
  marker_rear.pose.position.x = 0.0;
  marker_rear.pose.position.y = 0.0;
  marker_rear.pose.position.z = 0.0;
  marker_rear.pose.orientation.x = 0.0;
  marker_rear.pose.orientation.y = 0.0;
  marker_rear.pose.orientation.z = 0.0;
  marker_rear.pose.orientation.w = 1.0;
  marker_rear.scale.x = 0.15;
  marker_rear.scale.y = 0.15;
  marker_rear.scale.z = 0.0;
  marker_rear.color.r = 1.0f;
  marker_rear.color.g = 0.0f;
  marker_rear.color.b = 0.0f;
  marker_rear.color.a = 1.0;

  marker_front.header.frame_id = "/map";
  marker_front.header.stamp = ros::Time::now();
  marker_front.ns = "points";
  marker_front.id = 1;
  marker_front.type = visualization_msgs::Marker::SPHERE;
  marker_front.action = visualization_msgs::Marker::ADD;
  marker_front.pose.position.x = 0.0;
  marker_front.pose.position.y = 0.0;
  marker_front.pose.position.z = 0.0;
  marker_front.pose.orientation.x = 0.0;
  marker_front.pose.orientation.y = 0.0;
  marker_front.pose.orientation.z = 0.0;
  marker_front.pose.orientation.w = 1.0;
  marker_front.scale.x = 0.15;
  marker_front.scale.y = 0.15;
  marker_front.scale.z = 0.0;
  marker_front.color.r = 0.0f;
  marker_front.color.g = 0.0f;
  marker_front.color.b = 1.0f;
  marker_front.color.a = 1.0;


    ros::spin();
    return 0;
}
