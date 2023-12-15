#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseAction.h>
#include"std_msgs/String.h"
#include<Tus_g5/ErrorMsg.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ac;
nav_msgs::Odometry last_position,new_position;
int flag=0;
int loop=0;
int time_stop=0;
int isActive=0;
ros::Publisher pub;
ros::Publisher pub_error;
void status_callback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
  //  printf("in status_callback\n");
    for(auto& status : msg->status_list)
    {

        if(status.status == actionlib_msgs::GoalStatus::ACTIVE)
        {
            isActive=1;
            // 执行活动状态下的操作
        }
        else 
        {
            isActive=0;
        }
    }
}
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double old_x,old_y,old_z,new_x,new_y,new_z;
double sum=0;
    if(flag==0)
    {
        flag=1;
        last_position=*msg;
        return;
    }
    else{
        last_position=new_position;
        new_position=*msg;
    }
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;

    // ROS_INFO_STREAM("Robot position: [" << x << ", " << y << ", " << z << "]");

    old_x=last_position.pose.pose.position.x;
    old_y=last_position.pose.pose.position.y;
    old_z=last_position.pose.pose.position.x;
    new_x=new_position.pose.pose.position.x;
    new_y=new_position.pose.pose.position.y;
    new_z=new_position.pose.pose.position.z;
    sum=(old_x-new_x)*(old_x-new_x)+(old_y-new_y)*(old_y-new_y);
      if (sum<0.0004 )
    {
        if(isActive==0)
            time_stop=0;
        else{
        time_stop+=1;
            if(time_stop>=70){
                ROS_WARN("Robot velocity is 0 for longer than 30 seconds!");
                std_msgs::String   msg;
                printf("has send\n");
                msg.data="a";
                pub.publish(msg);
                Tus_g5::ErrorMsg temp;
                temp.type=0;
                temp.message="导航过程中无法找到正确路径，导航停止";
                pub_error.publish(temp);
            //ac->cancelGoal();
            }
        }
    }
    else{
        time_stop=0;
        //printf("%f:%f %f %f\n",elapsed_time.toSec() , last_cmd_vel.linear.x, last_cmd_vel.linear.y, last_cmd_vel.linear.z);
        printf("%f\n",sum);
    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "nav_error");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/odom", 1, odomCallback);
     pub = nh.advertise<std_msgs::String>("/nav/stop", 1000);
     pub_error = nh.advertise<Tus_g5::ErrorMsg>("/error", 1000);
    ros::Subscriber status_sub = nh.subscribe("/move_base/status", 1, status_callback);     
  ros::Rate rate(10); // 10 Hz

  while (ros::ok())
  {
   
  
    ros::spinOnce();
    rate.sleep();
  }

    return 0;
}