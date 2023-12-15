#include <ros/ros.h>
#include <Tus_g5/SoundMsg.h>
#include <stdio.h>
using namespace std;

void chatterCallBack(const Tus_g5::SoundMsg::ConstPtr& res) {
    string temp = "";
    string speech = (res -> op == 1) ? "warning, fire detected!":
   (res -> op == 2) ? "warning, somke detected!":
   (res -> op == 3) ? "warning, stranger detected!":
    "";
    if (res -> op != 0) {
        temp = "rosrun sound_play say.py \"" + speech + "\"";
        system(temp.c_str());
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "voice_node");
  ros::NodeHandle n;
  ros::Subscriber sound_sub = n.subscribe("/my_sound",10,chatterCallBack);
  
  ros::spin();
  return 0;
}