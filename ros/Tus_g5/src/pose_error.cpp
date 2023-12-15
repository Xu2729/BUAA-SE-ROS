
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "tf/tf.h"
#include<Tus_g5/ErrorMsg.h>
#define min_imu 0.1
ros::Publisher pub_error;
// IMU 回调函数
void IMUCallback(const sensor_msgs::Imu msg)
{
    // 检测消息包中四元数数据是否存在
    if(msg.orientation_covariance[0] < 0)
        return;
    // 四元数转成欧拉角
    tf::Quaternion quaternion(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    );
    double roll, pitch, yaw;
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    // 弧度换算成角度
    roll = roll*180/M_PI;
    pitch = pitch*180/M_PI;
    yaw = yaw*180/M_PI;
    ROS_INFO("滚转= %.0f 俯仰= %.0f 朝向= %.0f", roll, pitch, yaw);
    printf("%f\n",pitch);
    if(pitch>min_imu||pitch<-min_imu)
    {
        ROS_INFO("error!!!!!!!!!!!!!!!!!!!!");
        Tus_g5::ErrorMsg temp;
        temp.type=1;
         temp.message="机器人姿态异常";
         pub_error.publish(temp);
          sleep(1);
    }
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc,argv, "pose_error"); 
  
    ros::NodeHandle n;
      ros::Rate rate(10);
      pub_error = n.advertise<Tus_g5::ErrorMsg>("/error", 1000);
    // 订阅 IMU 的数据话题
    ros::Subscriber sub = n.subscribe("imu/data", 100, IMUCallback);
    while (ros::ok())
  {
   
  
    ros::spinOnce();
  //  rate.sleep();
  }

    return 0;
}
