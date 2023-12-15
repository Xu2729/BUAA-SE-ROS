#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <Tus_g5/PoseSrv.h>

geometry_msgs::Pose now_pose;

static bool debug = true;

void show_pose(geometry_msgs::Pose pose) {
    printf("position:\n");
    printf("  x: %.2f\n", pose.position.x);
    printf("  y: %.2f\n", pose.position.y);
    printf("  z: %.2f\n", pose.position.z);
    printf("orientation:\n");
    printf("  x: %.2f\n", pose.orientation.x);
    printf("  y: %.2f\n", pose.orientation.y);
    printf("  z: %.2f\n", pose.orientation.z);
    printf("  w: %.2f\n", pose.orientation.w);
}

bool current_pose_callback(Tus_g5::PoseSrv::Request &req,
                            Tus_g5::PoseSrv::Response &res) {
    res.pose = now_pose;
    if (debug) {
        printf("call PoseSrv\n");
        show_pose(now_pose);
    }
    return true;
}

void update_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& data) {
    now_pose = (data->pose).pose;
    if (debug) {
        printf("pose update!\n");
        show_pose(now_pose);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_server");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("/cur_pose", current_pose_callback);
    ros::Subscriber sub = n.subscribe("/amcl_pose", 1, update_pose_callback);
    ros::spin();
    return 0;
}