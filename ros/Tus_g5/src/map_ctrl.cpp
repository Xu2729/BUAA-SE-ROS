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
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include "std_msgs/Char.h"
#include "Tus_g5/MappingCmd.h"
#include "string.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

static float linear_vel = 0.1;
static float angular_vel = 0.1;
static int k_vel = 3;
int direction;
int flag = 0; // switch
float speed;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Publisher cmd_vel_pub;

void chatterCallBack(const Tus_g5::MappingCmd::ConstPtr& cmd) {
  ROS_INFO("callback function called!");
  if (!cmd->isCmd) {
        ROS_INFO("data is direction!");
        if (flag && (cmd->direction == 0 || cmd->direction == 1|| cmd->direction== 2|| cmd->direction == 3 || cmd->direction== 4|| cmd->direction == 5 || cmd->direction == 6)) {
            geometry_msgs::Twist base_cmd;
            base_cmd.linear.x = 0;
            base_cmd.linear.y = 0;
            base_cmd.angular.z = 0;
            if (cmd->direction == 1) {
                base_cmd.linear.x = cmd->speed;
            } else if (cmd->direction == 2) {
                base_cmd.linear.x = -cmd->speed;
            } else if (cmd->direction == 3) {
                base_cmd.linear.y = cmd->speed;
            } else if (cmd->direction == 4) {
                base_cmd.linear.y = -cmd->speed;
            } else if (cmd->direction == 5) {
                base_cmd.angular.z = cmd->speed;
            } else if (cmd->direction == 6) {
                base_cmd.angular.z = -cmd->speed;
            }
            cmd_vel_pub.publish(base_cmd);
        }
  }
  else {
        ROS_INFO("data is command!");
        // 如果是其他命令
        // cmdId:
        // 1: 开始建图
        // 2: 取消建图
        // 3: 存图
        // 4: 
        printf("cmd->cmdId = %d, flag = %d\n", cmd->cmdId, flag);
        if (cmd->cmdId == 1) {
            flag = 1;
            //system("roslaunch Tus_g5 gmapping_sim.launch&");
            printf("=============================================\n");
             system("roslaunch Tus_g5 gmapping.launch&");
            ROS_INFO("mapping started!");
        }
        else if (flag && cmd->cmdId == 2) {
            ROS_INFO("mapping cancelled!");
            flag = 0;
            system("rosnode kill gmapping");
            system("rosnode kill robot_state_publisher");
            system("killall -9 rviz");
        }
        else if (flag && cmd->cmdId == 3) {
            char a[] = "rosrun map_server map_saver -f ";
            char b[]=" /home/robot/maps/";
            
            strcat(a,b);
            strcat(a, cmd->graphName.c_str());
            ROS_INFO(a);
            system(a);
            ROS_INFO("map saved!");
        }
    }
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_ctrl");
  ros::NodeHandle n;
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  ros::Subscriber ctrl_sub = n.subscribe("/map_ctrl",10,chatterCallBack);
  
  ros::spin();
  return 0;
}
