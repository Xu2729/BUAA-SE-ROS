/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THEs
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/* @author Zhang Wanjie                                             */

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>


#include <actionlib_msgs/GoalStatusArray.h>
#include <string>
#include<stdio.h>

#include "std_msgs/String.h"
#include"Tus_g5/NavControl.h"

Tus_g5::NavControl control;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ac;
ros::Subscriber sub;
int hascancel=0;
int run_flag=1;
int status=0;
int loop=0;


void navCallBack(Tus_g5::NavControl msgs)
{
    printf("navcallback\n");
    status=1;
    hascancel=0;
  //  printf("%d\n",status);
    control=msgs;
    loop=control.loop;

}
void status_callback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
   // printf("in status_callback\n");
    for(auto& status : msg->status_list)
    {

        if(status.status == actionlib_msgs::GoalStatus::ACTIVE)
        {
            ROS_INFO("Robot navigation is active.");
            // 执行活动状态下的操作
        }
        else if(status.status == actionlib_msgs::GoalStatus::ABORTED)
        {
            ROS_INFO("Robot navigation has been aborted.");
            run_flag=0;
            if(hascancel==0){
                ac->cancelGoal();
            // 执行中止状态下的操作
            hascancel=1;
            }
        }
    }
}
void resultCb(const move_base_msgs::MoveBaseActionResult::ConstPtr& result)
{
    if (result->status.status == actionlib_msgs::GoalStatus::ABORTED)
    {
       // ac.cancelGoal();
        ROS_INFO("The navigation task has been cancelled due to planning failure.");
    }
    else if (result->status.status == actionlib_msgs::GoalStatus::SUCCEEDED)
    {
        run_flag=0;
        ROS_INFO("The navigation task has been completed successfully.");
    }
    else if (result->status.status == actionlib_msgs::GoalStatus::PREEMPTED)
    {
        ROS_INFO("The navigation task has been preempted.");
    }
}
void subCallback(std_msgs::String msgs)
{
   // ROS_INFO("has in callback\n");
    //ROS_INFO("recieve %s",msgs.name.c_str());
    ac->cancelGoal();
    hascancel=1;
    
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "nav");
    
    ros::NodeHandle nh;
    
    sub = nh.subscribe("/nav/stop", 10,& subCallback);
    ros::Subscriber status_sub = nh.subscribe("/move_base/status", 1, status_callback);
    ros::Subscriber result_sub = nh.subscribe<move_base_msgs::MoveBaseActionResult>("/move_base/result", 1, &resultCb);
     ros::Subscriber sub_nav = nh.subscribe("/nav/nav", 1, &navCallBack);
     ros::Publisher ctrl_pub_nav_arrive = nh.advertise<Tus_g5::NavControl>("/nav/arrive", 10);
    ///////////////////////////////////////////////////////////////////////////////////
  

    ac=new MoveBaseClient("move_base", true);
    
    while(!ac->waitForServer(ros::Duration(5.0)))
    {
        if(!ros::ok())
            break;
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    int nWPIndex = 0;
    int nNumOfWaypoints = 0;
    move_base_msgs::MoveBaseGoal goal;
    
    while(ros::ok())
    {
       ros::spinOnce();
       //printf("status=%d\n",status);
       if(status==0)
            continue;
       std:: vector<geometry_msgs::Pose> poses =control.poses;
       while(loop>0){
           printf("in loop\n");
           for (auto temp : poses)
    {
        
    
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
       
        //control.points[0].pose=srvI.response.pose;
        printf("pose=%f\n",temp.position.x);
        goal.target_pose.pose =  temp;
      /*  goal.target_pose.pose.position.x=-4.17598;
        goal.target_pose.pose.position.y=-0.953346;
        goal.target_pose.pose.position.z=0;
        goal.target_pose.pose.orientation.x=0;
        goal.target_pose.pose.orientation.y=0;
        goal.target_pose.pose.orientation.z=0.0517015;
        goal.target_pose.pose.orientation.w=0.998663;*/
        ac->sendGoal(goal);
        //ac->waitForResult();
        printf("hascancel=%d\n",hascancel);
        while(true)
        {
            //printf("run_flag=1\n");
            if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
               
        break;
            }
                    
            if(hascancel==1)
                break;
             ros::spinOnce();
        }
         
        run_flag=1;
        if(hascancel==1){
            break;
        }
            
        if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Arrived at WayPoint[%d] !",nWPIndex);
            nWPIndex ++;
        }
        else
            ROS_INFO("Failed to get to WayPoint[%d] ...",nWPIndex );
       }
       loop--;
       Tus_g5::NavControl temp2;
        ctrl_pub_nav_arrive.publish(temp2);
        printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!has send\n");
       if(hascancel==1)
       {
           loop=0;
           break;
       }
    }
    
    if(hascancel==1){
            hascancel=0;
            status=0;
            continue;
        }
        status=0;
        
    }

    return 0;
}