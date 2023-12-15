#include <ros/ros.h>
#include <tf/transform_listener.h>
#include"std_msgs/String.h"

#include<stdio.h>
#include <iostream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
 using namespace std;
geometry_msgs::PoseWithCovarianceStamped msg;
 ros::Publisher pub;

 void subCallback(std_msgs::String)
{
    pub.publish(msg);
}
int main(int argc, char** argv){
  ros::init(argc, argv, "transform_position");

  // 创建一个节点句柄
  ros::NodeHandle nh;

  // 创建一个 TF 监听器对象
  tf::TransformListener listener;
 pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1000);
    ros::Subscriber status_sub = nh.subscribe("/pub_initial", 1, subCallback);   
  // 等待 TF 数据的发布
  ros::Duration(0.5).sleep();

  while (ros::ok()){
    // 尝试获取机器人在 /map 坐标系中的位置
    tf::StampedTransform transform;
    try{
      listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    // 输出机器人在 /map 坐标系中的位置
   msg.pose.pose.position.x = transform.getOrigin().x();
    msg.pose.pose.position.y = transform.getOrigin().y();
    msg.pose.pose.position.z =   transform.getOrigin().z();
    msg.pose.pose.orientation.x =  transform.getRotation().x();
    msg.pose.pose.orientation.y =  transform.getRotation().y();
    msg.pose.pose.orientation.z =  transform.getRotation().z();
    msg.pose.pose.orientation.w = transform.getRotation().w();
  
    ROS_INFO("Robot is at (x, y, z) = (%.2f, %.2f, %.2f), (x, y, z, w) = (%.2f, %.2f, %.2f, %.2f)",
             transform.getOrigin().x(),
             transform.getOrigin().y(),
             transform.getOrigin().z(),
             transform.getRotation().x(),
             transform.getRotation().y(),
             transform.getRotation().z(),
             transform.getRotation().w());

    ros::Duration(1.0).sleep();
    ros::spinOnce();
  }

  return 0;
}