#include "ros/ros.h"
#include "std_msgs/String.h"

#include <iostream>
#include <csignal>
#include <unistd.h>
int flag=0;
void subCallback(std_msgs::String   msgs)
{
	pid_t pid = getpid();
    
    // 打印进程 ID
    std::cout << "My PID is: " << pid << std::endl;
    printf("flag=%d!!!!!!!!\n",flag);

    system("roslaunch wpr_simulation wpb_navigation11.launch");
	flag++;

	

    // 睡眠 10 秒钟
    sleep(10);

    // 向进程发送 SIGINT 信号
    std::cout << "Sending SIGINT signal..." << std::endl;
    kill(pid, SIGINT);


}
int main(int argc, char *argv[])
{
     ros::init(argc, argv, "sehll");
     ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/shell", 10,& subCallback);
   
    setlocale(LC_ALL,"");
    system("roslaunch wpr_simulation wpb_navigation11.launch");
    while (ros::ok())
    {
        /* code for loop body */
        ros::spinOnce();
    }
    
    
    
    return 0;
}