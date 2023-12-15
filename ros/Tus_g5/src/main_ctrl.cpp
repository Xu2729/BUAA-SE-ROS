#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <Tus_g5/MainCtrl.h>
#include <Tus_g5/KeyBoardCtrlMsg.h>
#include <Tus_g5/NavCtrlMsg.h>
#include <Tus_g5/MappingCmd.h>
#include <Tus_g5/NavControl.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
using namespace std;
#define STATE_IDLE          0
#define STATE_NAVIGATION    2
#define STATE_MAPPING       3
#define STATE_ERROR         4

static int state = STATE_IDLE;

#define FORCE_STOP          0
#define ROS_EXIT            1

#define USER_CTRL_KEYBOARD  12
#define USER_CTRL_COMMADN   13

#define MAPPING_START       20
#define MAPPING_END         21
#define MAPPING_MOVE        22
#define MAPPING_SAVE        23

#define NAVIGATION_START    30
#define NAVIGATION_END      31
#define NAVIGATION_PATROL   32
#define NAVIGATION_STOP     34


static ros::Publisher vel_pub;
static ros::Publisher mapping_pub;
static ros::Publisher navigation_pub;
float origin_x,origin_y;
//
void GetOrigin(std::string fileName)
{
    std::ifstream inFile;
    inFile.open(fileName);
    std::string line;
    std::stringstream ss;
    int currentLine = 0;
    string temp = "cat "+fileName;
    // 将所有行读入 stringstream 中
   // system(temp.c_str());
    while (std::getline(inFile, line))
    {
        currentLine++;
      //printf("%s\n",line.c_str());
        size_t pos1 = line.find("origin");
        if(pos1 != string::npos)
        {
            std::size_t comma_pos1 = line.find(",");
            std::size_t comma_pos2 = line.find(",", comma_pos1 + 1);
            double x = std::stod(line.substr(9, comma_pos1 - 1));
            double y = std::stod(line.substr(comma_pos1 + 1, comma_pos2 - comma_pos1 - 1));
            double z = std::stod(line.substr(comma_pos2 + 1, line.size() - comma_pos2 - 2));
            origin_x=x;
            origin_y=y;
            std::cout << "x: " << x << ", y: " << y << ", z: " << z << std::endl;
            printf("%s\n",line.c_str());
                 
        }
    }
    // 关闭文件
    inFile.close();   
}
void show_req(Tus_g5::MainCtrl::Request &req) {
    printf("==========================================================\n");
    printf("id: %d\n", req.id);
    printf("type: %d\n", req.type);
    printf("keyboard_ctrl_msg:\n");
    printf("  direction: %d\n", req.keyboard_ctrl_msg.direction);
    printf("  speed: %.2f\n", req.keyboard_ctrl_msg.speed);
    printf("navigation_ctrl_msg:\n");
    printf("  loop: %d\n", req.navigation_ctrl_msg.loop);
    printf("  pose_list:\n");
    for (int i = 0; i < req.navigation_ctrl_msg.pose_list.size(); i++) {
        printf("    - position:\n");
        printf("        x: %.2f\n", req.navigation_ctrl_msg.pose_list[i].position.x);
        printf("        y: %.2f\n", req.navigation_ctrl_msg.pose_list[i].position.y);
        printf("        z: %.2f\n", req.navigation_ctrl_msg.pose_list[i].position.z);
        printf("      orientation:\n");
        printf("        x: %.2f\n", req.navigation_ctrl_msg.pose_list[i].orientation.x);
        printf("        y: %.2f\n", req.navigation_ctrl_msg.pose_list[i].orientation.y);
        printf("        z: %.2f\n", req.navigation_ctrl_msg.pose_list[i].orientation.z);
        printf("        w: %.2f\n", req.navigation_ctrl_msg.pose_list[i].orientation.w);
    }
    printf("  name_list:\n");
    for (int i = 0; i < req.navigation_ctrl_msg.name_list.size(); i++) {
        printf("    - %s\n", req.navigation_ctrl_msg.name_list[i].c_str());
    }
    printf("command: %s\n", req.command.c_str());
}

bool main_ctrl_callback(Tus_g5::MainCtrl::Request &req,
                        Tus_g5::MainCtrl::Response &res) {
    // show_req(req);
    switch (req.type)
    {
        case FORCE_STOP:
            state = STATE_IDLE;
            // TODO
            break;
        case ROS_EXIT:
            state = STATE_IDLE;
            // TODO
            break;
        case MAPPING_START:
            if (state == STATE_IDLE) {
                state = STATE_MAPPING;
                res.code = 0;
                Tus_g5::MappingCmd msg;
                msg.isCmd = true;
                msg.cmdId = 1;
                mapping_pub.publish(msg);
            } else {
                res.code = 1;
                res.msg = "状态转移错误，请先回到空闲状态";
            }
            break;
        case MAPPING_END:
            if (state == STATE_MAPPING) {
                state = STATE_IDLE;
                res.code = 0;
                Tus_g5::MappingCmd msg;
                msg.isCmd = true;
                msg.cmdId = 2;
                mapping_pub.publish(msg);
            } else {
                res.code = 1;
                res.msg = "状态转移错误，当前不在建图状态";
            }
            break;
        case MAPPING_MOVE:
            if (state == STATE_MAPPING) {
                res.code = 0;
                Tus_g5::MappingCmd msg;
                msg.isCmd = false;
                msg.direction = req.keyboard_ctrl_msg.direction;
                msg.speed = req.keyboard_ctrl_msg.speed;
                mapping_pub.publish(msg);
            } else {
                res.code = 1;
                res.msg = "状态转移错误，当前不在建图状态";
            }
            break;
        case MAPPING_SAVE:
            if (state == STATE_MAPPING) {
                res.code = 0;
                Tus_g5::MappingCmd msg;
                msg.isCmd = true;
                msg.cmdId = 3;
                msg.graphName = req.navigation_ctrl_msg.name_list[0];
                mapping_pub.publish(msg);
            } else {
                res.code = 1;
                res.msg = "状态转移错误，当前不在建图状态";
            }
            break;
        case NAVIGATION_START:
            if (state == STATE_IDLE) {
                state = STATE_NAVIGATION;
                res.code = 0;
                Tus_g5::NavControl msg;
                msg.op = 0;
                msg.map_name = req.navigation_ctrl_msg.name_list[0];
                string yaml_name="/home/robot/maps/";
                yaml_name=yaml_name+msg.map_name+".yaml";
                GetOrigin(yaml_name);
                res.pose.position.x=origin_x;
                res.pose.position.y=origin_y;
                navigation_pub.publish(msg);
                system("rosrun sound_play soundplay_node.py&");
            } else {
                res.code = 1;
                res.msg = "状态转移错误，请先回到空闲状态";
            }
            break;
        case NAVIGATION_END:
            if (state == STATE_NAVIGATION) {
                state = STATE_IDLE;
                res.code = 0;
                Tus_g5::NavControl msg;
                msg.op = 1;
                navigation_pub.publish(msg);
                system("rosnode kill sound_play");
            } else {
                res.code = 1;
                res.msg = "状态转移错误，当前不在导航状态";
            }
            break;
        case NAVIGATION_PATROL:
            if (state == STATE_NAVIGATION) {
                res.code = 0;
                Tus_g5::NavControl msg;
                msg.op = 2;
                msg.poses = req.navigation_ctrl_msg.pose_list;
                msg.names = req.navigation_ctrl_msg.name_list;
                msg.loop = req.navigation_ctrl_msg.loop;
                navigation_pub.publish(msg);
            } else {
                res.code = 1;
                res.msg = "状态转移错误，请先回到空闲状态";
            }
            break;
        case NAVIGATION_STOP:
            if (state == STATE_NAVIGATION) {
                res.code = 0;
                Tus_g5::NavControl msg;
                msg.op = 3;
                navigation_pub.publish(msg);
            } else {
                res.code = 1;
                res.msg = "状态转移错误，请先回到空闲状态";
            }
            break;
        case USER_CTRL_KEYBOARD:
            if (state == STATE_NAVIGATION) {
                res.code = 0;
                Tus_g5::NavControl msg;
                msg.op = 5;
                msg.direction = req.keyboard_ctrl_msg.direction;
                msg.speed = req.keyboard_ctrl_msg.speed;
                navigation_pub.publish(msg);
            } else {
                res.code = 1;
                res.msg = "状态转移错误，请先回到空闲状态";
            }
            break;
        case USER_CTRL_COMMADN:
            if (state == STATE_NAVIGATION) {
                res.code = 0;
                Tus_g5::NavControl msg;
                msg.op = 6;
                msg.command = req.command;
                msg.poses = req.navigation_ctrl_msg.pose_list;
                msg.names = req.navigation_ctrl_msg.name_list;
                navigation_pub.publish(msg);
            } else {
                res.code = 1;
                res.msg = "状态转移错误，请先回到空闲状态";
            }
            break;
        default:
            res.code = 1;
            res.msg = "未知指令";
    }
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "main_ctrl");
    ros::NodeHandle n;
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    mapping_pub = n.advertise<Tus_g5::MappingCmd>("/map_ctrl", 10);
    navigation_pub = n.advertise<Tus_g5::NavControl>("/nav", 10);
    ros::ServiceServer service = n.advertiseService("/main_ctrl", main_ctrl_callback);
    ros::spin();
    return 0;
}
