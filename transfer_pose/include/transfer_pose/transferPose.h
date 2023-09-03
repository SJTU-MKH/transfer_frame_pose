#include <iostream>
#include <utility>
#include <iostream>
#include <istream>
#include <streambuf>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <stdlib.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class transferPose
{
private:
    ros::Publisher pose_pub_;
    ros::Subscriber pose_sub_;
    ros::NodeHandle nh;

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
    bool parseCsv(std::string path);
    std::vector<nav_msgs::Path> v_path_pose;
public:
    transferPose(/* args */);
    ~transferPose();
    bool init_flag;
    ros::Time timestamp_;
};

transferPose::transferPose(/* args */)
{
    parseCsv("sf");
    pose_pub_ = nh.advertise<nav_msgs::Path>("Vision_path",10);
    pose_sub_ = nh.subscribe("/airsim_node/drone_1/front_left_custom/Scene", 10, imageCallback);

}

transferPose::~transferPose()
{
}

bool transferPose::parseCsv(std::string path)
{
    std::ifstream csv_data(path, std::ios::in);
    std::string line;
    if (!csv_data.is_open())
    {
        ROS_WARN("Can't open or find %s.", path);
        return false;
    }

    std::istringstream sin;         //将整行字符串line读入到字符串istringstream中
    std::vector<std::string> words; //声明一个字符串向量
    std::string word;

    /* csv header:
    index,original_ImageFile,liadr_ImageFile,X,Y,Z,distance,TimeStamp,top,left,bottom,right,13yaw,pitch,roll
    */
    // 读取标题行
    std::getline(csv_data, line);
    // 读取数据
    while (std::getline(csv_data, line))
    {
        sin.clear();
        sin.str(line);
        words.clear();
        while (std::getline(sin, word, ',')) //将字符串流sin中的字符读到field字符串中，以逗号为分隔符
        {
            words.push_back(word); //将每一格中的数据逐个push   
        }
        
        geometry_msgs::PoseStamped pose_stamped;
        ros::Time timestamp = ros::Time::fromSec(atof(words[7].c_str()));
        pose_stamped.header.frame_id = "global";
        pose_stamped.header.stamp = timestamp;
        pose_stamped.pose.position.x = atof(words[3].c_str());
        pose_stamped.pose.position.y = atof(words[4].c_str());
        pose_stamped.pose.position.z = atof(words[5].c_str());

        tf2::Quaternion quaternion;
        quaternion.setRPY(atof(words[15].c_str()), atof(words[13].c_str()), atof(words[14].c_str()));
        pose_stamped.pose.orientation = tf2::toMsg(quaternion);

        nav_msgs::Path tmp_path_pose;
        tmp_path_pose.header = pose_stamped.header;
        tmp_path_pose.poses.push_back(pose_stamped);
        v_path_pose.push_back(tmp_path_pose);
    }
    csv_data.close();
}

void transferPose::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    if( msg->header.seq == 0)
        init_flag = true;
    timestamp_ = msg->header.stamp;

}

