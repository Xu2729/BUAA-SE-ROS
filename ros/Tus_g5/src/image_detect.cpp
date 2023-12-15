#include <ros/ros.h>
#include <ctime>
#include <cstdio>
#include <iostream>
#include <string>
#include <fstream>
#include <sys/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/String.h"
#include <Tus_g5/SoundMsg.h>
using namespace cv;
using namespace std;

ros::Publisher pub;

time_t last_time=(time_t)0;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    time_t nt = time(NULL);
    if (nt - last_time < 2) {
        if (nt - last_time < 1) {
            sleep(1);
        }
        return;
    }
    last_time = time(NULL);
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    Mat image_compressed;
    int hw = cv_ptr->image.size().width * cv_ptr->image.size().height;
    // printf("================%d %d %d\n", cv_ptr->image.size().width, cv_ptr->image.size().height, hw);
    double scale = 1.0 * 400000 / hw;
    // printf("=============scale = %.2f\n", scale);
    vector<int> tmp_para;
    tmp_para.push_back(CV_IMWRITE_JPEG_QUALITY);
    tmp_para.push_back(75);
    resize(cv_ptr->image, image_compressed, cv_ptr->image.size(), scale, scale, INTER_LINEAR);
    imwrite("/home/robot/tmp.jpg", image_compressed, tmp_para);
    system("python3 /home/robot/send.py");
    FILE *fp = fopen("/home/robot/post_result.txt", "r");
    int res;
    fscanf(fp, "%d", &res);
    fclose(fp);
    Tus_g5::SoundMsg mmmsg;
    mmmsg.op = res;
    pub.publish(mmmsg);
    ROS_INFO("Detect result: %d\n", res);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_detect");
    ros::NodeHandle nh;
    pub = nh.advertise<Tus_g5::SoundMsg>("/my_sound", 10);
    ros::Subscriber sub = nh.subscribe("/kinect2/qhd/image_color_rect", 1, imageCallback);
    ros::spin();

    return 0;
}
