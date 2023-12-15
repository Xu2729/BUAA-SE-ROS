#include "ros/ros.h"
#include<std_msgs/String.h>

void subCallback(std_msgs::String   msgs)
{
    printf("has receive\n");
     system("roslaunch Tus_g5 wpb_navigation11.launch");
   // system("roslaunch waterplus_map_tools wpb_navigation11.launch");
    //真实机器人的launch文件
    //system("roslaunch Tus_g5 wpb_home_nav_test.launch");
}
int main(int argc, char *argv[])
{
     ros::init(argc, argv, "start_rviz");
     ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/nav/start", 10,& subCallback);
   
    setlocale(LC_ALL,"");
    while (ros::ok())
    {
        /* code for loop body */
        ros::spinOnce();
    }
    
    
    
    return 0;
}