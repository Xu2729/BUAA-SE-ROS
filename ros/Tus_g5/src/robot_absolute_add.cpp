#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/PointStamped.h>
#include "std_msgs/Char.h"
#include<stdio.h>
// 机器人当前位姿
geometry_msgs::PointStamped current_pose;

// 地图元数据信息
nav_msgs::MapMetaData map_metadata;

// 机器人在地图中的坐标
geometry_msgs::Point robot_position;

ros::Publisher send_place_pub;
// 机器人当前位姿回调函数
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
    current_pose.header = odom->header;
    current_pose.point = odom->pose.pose.position;
}

void placeCallback(std_msgs::Char msg)
{
    send_place_pub.publish(robot_position);
}
// 地图元数据信息回调函数
void metadataCallback(const nav_msgs::MapMetaData::ConstPtr& metadata)
{
    map_metadata = *metadata;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_position_node");
    ros::NodeHandle nh;
    ros::Subscriber odometry_subscriber = nh.subscribe("/odom", 1, odomCallback);
    ros::Subscriber map_metadata_subscriber = nh.subscribe("/map_metadata", 1, metadataCallback);
    ros::Publisher position_publisher = nh.advertise<geometry_msgs::Point>("robot_absolute_add", 1);
    ros::Subscriber send_place_sub = nh.subscribe("/nav/need_send_place", 1, placeCallback);
    send_place_pub =nh.advertise<geometry_msgs::Point>("/nav/absolute_place", 1);;
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        // 计算机器人在地图中的x,y坐标
        robot_position.x = current_pose.point.x - map_metadata.origin.position.x;
        robot_position.y = current_pose.point.y - map_metadata.origin.position.y;

        // 发送机器人在地图中的坐标
        position_publisher.publish(robot_position);
        printf("x=%f,y=%f\n",current_pose.point.x,current_pose.point.y);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}