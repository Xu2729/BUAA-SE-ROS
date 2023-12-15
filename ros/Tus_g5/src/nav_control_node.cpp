#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <termios.h>
#include <iostream>
#include <fstream>
#include "std_msgs/Char.h"
#include<std_msgs/String.h>

#include "std_msgs/Int32.h"
#include"Tus_g5/NavControl.h"
#include <sensor_msgs/JointState.h>

#define KEYBOARD '1'
#define SPEECH '2'
#define MAP '3'
using namespace std;
Tus_g5::NavControl control;
int status=0;
int receive=0;
int isbreak=0;
int cKey;
int op;
int flag_initial=0;
int direction;
double speed;
string command;
int has_arrive=0;
double now_arm_height;
double now_arm_distance;
string name;
void chatterCallback(Tus_g5::NavControl msg)
{
    receive=1;
    control=msg;
    cKey=control.op;
    direction = msg.direction;
    speed = msg.speed;
    command = msg.command;
    name=msg.map_name;
}
void arriveCallback(Tus_g5::NavControl msg)
{
    has_arrive=1;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "NavControl");
  
  
  ros::NodeHandle n;
 
ros::Publisher ctrl_pub_stop = n.advertise<std_msgs::String>("/nav/stop", 10);
ros::Publisher ctrl_pub_nav = n.advertise<Tus_g5::NavControl>("/nav/nav", 10);
ros::Publisher ctrl_pub_start = n.advertise<std_msgs::String>("/nav/start", 10);
ros::Publisher ctrl_pub_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
ros::Publisher ctrl_pub_mani = n.advertise<sensor_msgs::JointState>("/wpb_home/mani_ctrl", 30);
ros::Publisher ctrl_pub_initial = n.advertise<std_msgs::String>("/pub_initial", 30);
ros::Subscriber sub_op = n.subscribe("/nav", 1000, &chatterCallback);
ros::Subscriber sub_arrive = n.subscribe("/nav/arrive", 1000, &arriveCallback); 
  while(ros::ok())
  {
  
  ros::spinOnce();
    if(receive==0){
        has_arrive=0;
        continue;
    }
        
    
    printf("op=%d\n",cKey);
    printf("status=%d\n",status);
    if(cKey==0){
     string move="cp /home/robot/maps/"+name+".pgm /home/robot/catkin_ws/src/wpb_home/wpb_home_tutorials/maps/map.pgm";
    string move2="cp /home/robot/maps/"+name+".yaml  /home/robot/catkin_ws/src/wpb_home/wpb_home_tutorials/maps/map.yaml";
    //    string move="cp /home/rookie/maps/"+name+".pgm /home/rookie/catkin_try/src/wpr_simulation/maps/map.pgm";
    //     string move2="cp /home/rookie/maps/"+name+".yaml  /home/rookie/catkin_try/src/wpr_simulation/maps/map.yaml";
    if(flag_initial==0)
    {
        std_msgs::String msg;
       ctrl_pub_initial.publish(msg);
        flag_initial++;
    }
        printf("move=%s\n",move.c_str());
        system(move.c_str());
        system(move2.c_str());
        std_msgs::String temp;
        status=1;
        ctrl_pub_start.publish(temp);
        system("rosrun Tus_g5 image_detect&");
        system("rosrun Tus_g5 warnning_sound&");
        printf("has send\n");
        }
    else {
       /* if(status==0)
            continue;*/
      if(cKey==1)
      {
          printf("in cket1 %d\n",has_arrive);
          system("rosnode kill image_detect");
    system("rosnode kill voice_node");
        status=0;      
        std_msgs::String msgs;
          ctrl_pub_stop.publish(msgs);
          Tus_g5::NavControl temp;
          temp.op=2;
          geometry_msgs::Pose zero;
          vector<geometry_msgs::Pose> poses;
          zero.position.x=0;
          zero.position.y=0;
          zero.position.z=0;
          zero.orientation.w=1;
          temp.loop=1;
          poses.push_back(zero);
          temp.poses=poses;
          has_arrive=0;
          ctrl_pub_nav.publish(temp);
        while(has_arrive!=1)
        {
            ros::spinOnce();
        }
           system("killall -9 rviz");
      }
      else if(cKey==2)
      {
          //航点巡逻
          printf("has in nav\n");
          
          ctrl_pub_nav.publish(control);
          printf("nav end \n");
      }
      else if(cKey==3)
      {
          std_msgs::String msgs;
          ctrl_pub_stop.publish(msgs);
          //停止
      }
      else if(cKey==4)
      {
          //加载地图
      }
      else if(cKey == 5) {
                std::cout << "into op 5" << std::endl;
                geometry_msgs::Twist vel_cmd;
                vel_cmd.linear.x = 0;
                vel_cmd.linear.y = 0;
                vel_cmd.linear.z = 0;
                vel_cmd.angular.x = 0;
                vel_cmd.angular.y = 0;
                vel_cmd.angular.z = 0;
                switch (direction) {
                case 0:
                    break;
                case 1:
                    std::cout << "into case 1" << std::endl;
                    vel_cmd.linear.x = speed;
                    break;
                case 2:
                    vel_cmd.linear.x = -speed;
                    break;
                case 3:
                    vel_cmd.linear.y = speed;
                    break;
                case 4:
                    vel_cmd.linear.y = -speed;
                    break;
                case 5:
                    vel_cmd.angular.z = speed;
                    break;
                case 6:
                    vel_cmd.angular.z = -speed;
                    break;
                default:
                    break;
                }
                std::cout << "vec pub" << std::endl;
                ctrl_pub_vel.publish(vel_cmd);
            }
            else if(cKey == 6) {
                std::cout << command << std::endl;
                if(command.find("停") != string::npos) {
                    geometry_msgs::Twist vel_cmd;
                    vel_cmd.linear.x = 0;
                    vel_cmd.linear.y = 0;
                    vel_cmd.linear.z = 0;
                    vel_cmd.angular.x = 0;
                    vel_cmd.angular.y = 0;
                    vel_cmd.angular.z = 0;
                    ctrl_pub_vel.publish(vel_cmd);
                } else if(command.find("向前") != string::npos) {
                    geometry_msgs::Twist vel_cmd;
                    vel_cmd.linear.x = 0.1;
                    vel_cmd.linear.y = 0;
                    vel_cmd.linear.z = 0;
                    vel_cmd.angular.x = 0;
                    vel_cmd.angular.y = 0;
                    vel_cmd.angular.z = 0;
                    ctrl_pub_vel.publish(vel_cmd);
                } else if(command.find("向后") != string::npos) {
                    geometry_msgs::Twist vel_cmd;
                    vel_cmd.linear.x = -0.1;
                    vel_cmd.linear.y = 0;
                    vel_cmd.linear.z = 0;
                    vel_cmd.angular.x = 0;
                    vel_cmd.angular.y = 0;
                    vel_cmd.angular.z = 0;
                    ctrl_pub_vel.publish(vel_cmd);
                } else if(command.find("左转") != string::npos) {
                    geometry_msgs::Twist vel_cmd;
                    vel_cmd.linear.x = 0;
                    vel_cmd.linear.y = 0;
                    vel_cmd.linear.z = 0;
                    vel_cmd.angular.x = 0;
                    vel_cmd.angular.y = 0;
                    vel_cmd.angular.z = 0.1;
                    ctrl_pub_vel.publish(vel_cmd);
                } else if(command.find("右转") != string::npos) {
                    geometry_msgs::Twist vel_cmd;
                    vel_cmd.linear.x = 0;
                    vel_cmd.linear.y = 0;
                    vel_cmd.linear.z = 0;
                    vel_cmd.angular.x = 0;
                    vel_cmd.angular.y = 0;
                    vel_cmd.angular.z = -0.1;
                    ctrl_pub_vel.publish(vel_cmd);
                } else if(command.find("向左") != string::npos) {
                    geometry_msgs::Twist vel_cmd;
                    vel_cmd.linear.x = 0;
                    vel_cmd.linear.y = 0.1;
                    vel_cmd.linear.z = 0;
                    vel_cmd.angular.x = 0;
                    vel_cmd.angular.y = 0;
                    vel_cmd.angular.z = 0;
                    ctrl_pub_vel.publish(vel_cmd);
                } else if(command.find("向右") != string::npos) {
                    geometry_msgs::Twist vel_cmd;
                    vel_cmd.linear.x = 0;
                    vel_cmd.linear.y = -0.1;
                    vel_cmd.linear.z = 0;
                    vel_cmd.angular.x = 0;
                    vel_cmd.angular.y = 0;
                    vel_cmd.angular.z = 0;
                    ctrl_pub_vel.publish(vel_cmd);
                } else {
                    vector<string> name_vec;
                    vector<geometry_msgs::Pose> pose_vec;
                    for(int i = 0; i < control.names.size(); ++i) {
                        if(command.find(control.names.at(i)) != string::npos) {
                            name_vec.push_back(control.names.at(i));
                            pose_vec.push_back(control.poses.at(i));
                            control.op = 2;
                            control.names = name_vec;
                            control.poses = pose_vec;
                        }
                    }
                    if(name_vec.size() != 0)
                        ctrl_pub_nav.publish(control);
                }
            } else if(cKey == 7) {
                std::cout << "now op is 7" << std::endl;
                sensor_msgs::JointState ctrl_msg;
                ctrl_msg.name.resize(2);
                ctrl_msg.position.resize(2);
                ctrl_msg.velocity.resize(2);
                ctrl_msg.name[0] = "lift";
                ctrl_msg.name[1] = "gripper";
                ctrl_msg.position[0] = 0;
                ctrl_msg.position[1] = 0;
                switch(control.arm_op) {
                    case 0: // 拉下机械臂
                        ctrl_msg.position[0] = 0.5;     //升降高度(单位:米)
                        ctrl_msg.velocity[0] = 0.5;     //升降速度(单位:米/秒)
                        ctrl_msg.position[1] = 0.1;     //手爪指间距(单位:米)
                        ctrl_msg.velocity[1] = 5;       //手爪开合角速度(单位:度/秒)
                        now_arm_distance = 0.5;
                        break;
                    case 1: // 调整高度
                        ctrl_msg.position[0] = control.arm_height;       //升降高度(单位:米)
                        ctrl_msg.velocity[0] = 0.5;     //升降速度(单位:米/秒)
                        ctrl_msg.position[1] = now_arm_distance;     //手爪指间距(单位:米)
                        ctrl_msg.velocity[1] = 5;       //手爪开合角速度(单位:度/秒)
                        now_arm_height = control.arm_height;
                        break;
                    case 2: // 合
                        ctrl_msg.position[0] = now_arm_height;       //升降高度(单位:米)
                        ctrl_msg.velocity[0] = 0.5;     //升降速度(单位:米/秒)
                        ctrl_msg.position[1] = 0;     //手爪指间距(单位:米)
                        ctrl_msg.velocity[1] = 5;       //手爪开合角速度(单位:度/秒)
                        now_arm_distance = 0;
                        break;
                    case 3: // 开
                        ctrl_msg.position[0] = now_arm_height;       //升降高度(单位:米)
                        ctrl_msg.velocity[0] = 0.5;     //升降速度(单位:米/秒)
                        ctrl_msg.position[1] = 0.1;     //手爪指间距(单位:米)
                        ctrl_msg.velocity[1] = 5;       //手爪开合角速度(单位:度/秒)
                        now_arm_distance = 0.5;
                        break;
                    case 4: // 收起机械臂
                        ctrl_msg.position[0] = 0;       //升降高度(单位:米)
                        ctrl_msg.velocity[0] = 0.5;     //升降速度(单位:米/秒)
                        ctrl_msg.position[1] = 0.1;     //手爪指间距(单位:米)
                        ctrl_msg.velocity[1] = 5;       //手爪开合角速度(单位:度/秒)
                        break;
                }
                ctrl_pub_mani.publish(ctrl_msg);    //发送指令
            } else if(cKey == 8) {
                system("rosrun wpb_home_tutorials wpb_home_grab_client");
            }
   
  }
   receive=0;
   printf("arrive =%d\n",has_arrive);
   has_arrive=0;
   printf("arrive2=%d\n",has_arrive);
 }
    
  
  return 0;
}
