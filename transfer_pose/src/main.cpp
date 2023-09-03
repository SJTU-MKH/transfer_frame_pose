#include <ros/ros.h>
#include <transferPose.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "transfer_frame_node");
    transferPose transfer_pose("./src/transfer_pose/data/XYZ_DISTANCE_YAW.csv");

    ros::spin();
    return 0;
}
