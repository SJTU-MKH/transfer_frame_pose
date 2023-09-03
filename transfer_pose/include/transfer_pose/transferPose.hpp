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
#include <sensor_msgs/Image.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
class transferPose
{
private:
    ros::Publisher pose_pub_;
    ros::Subscriber image_sub_;
    ros::NodeHandle nh;

    // transform "world" 2 "pose1"
    Eigen::Affine3d inverse_transform;

    // 现在pose2_in_pose1_frame包含了pose2在pose1坐标系下的位姿
    void imageCallback(const sensor_msgs::Image::ConstPtr &msg);

    bool parseCsv(std::string path);
    std::vector<geometry_msgs::PoseStamped> v_path_pose;
    void transfer_ue2first();

public:
    transferPose(std::string csv_path);
    ~transferPose();
    bool init_flag_;
};

transferPose::transferPose(std::string csv_path)
{
    init_flag_ = false;
    // 读取csv 中的stamp和坐标信息
    parseCsv(csv_path);
    // transfer_ue2first();
    pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("Vision_path", 10);
    image_sub_ = nh.subscribe<sensor_msgs::Image>("/airsim_node/drone_1/front_left_custom/Scene", 10, &transferPose::imageCallback, this);
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
        ROS_WARN("Can't open or find %s.", path.c_str());
        return false;
    }
    ROS_INFO("***Parse file: %s", path.c_str());
    std::istringstream sin;         // 将整行字符串line读入到字符串istringstream中
    std::vector<std::string> words; // 声明一个字符串向量
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
        while (std::getline(sin, word, ',')) // 将字符串流sin中的字符读到field字符串中，以逗号为分隔符
        {
            words.push_back(word); // 将每一格中的数据逐个push
        }

        geometry_msgs::PoseStamped pose_stamped;

        uint64_t time_int = atof(words[7].c_str());

        ros::Time timestamp(time_int / 1000000000, (time_int % 1000000000));
        pose_stamped.header.frame_id = "global";
        pose_stamped.header.stamp = timestamp;
        pose_stamped.pose.position.x = atof(words[3].c_str());
        pose_stamped.pose.position.y = atof(words[4].c_str());
        pose_stamped.pose.position.z = atof(words[5].c_str());

        tf2::Quaternion quaternion;
        quaternion.setRPY(atof(words[14].c_str()), atof(words[13].c_str()), atof(words[12].c_str()));
        pose_stamped.pose.orientation = tf2::toMsg(quaternion);
        v_path_pose.push_back(pose_stamped);
    }
    csv_data.close();
    ROS_INFO("--Parse success; over");
}

void transferPose::imageCallback(const sensor_msgs::Image::ConstPtr &msg)
{
    ROS_INFO("*Received: %d", msg->header.seq);
    ros::Time timestamp_ = msg->header.stamp;
    // 参考姿态，第一帧
    if (msg->header.seq == 0)
    {
        init_flag_ = true;
        ROS_INFO("Initialize success!");

        Eigen::Affine3d forward_transform;
        forward_transform.translation() << v_path_pose[0].pose.position.x,
            v_path_pose[0].pose.position.y,
            v_path_pose[0].pose.position.z;
        Eigen::Quaterniond quat = Eigen::Quaterniond(v_path_pose[0].pose.orientation.w,
                                                     v_path_pose[0].pose.orientation.x,
                                                     v_path_pose[0].pose.orientation.y,
                                                     v_path_pose[0].pose.orientation.z).normalized();
        forward_transform.linear() = Eigen::AngleAxisd(quat).toRotationMatrix();

        inverse_transform = forward_transform.inverse();
    }

    // if (init_flag_ && v_path_pose[msg->header.seq].header.stamp == timestamp_)
    if (init_flag_)
    {
        geometry_msgs::PoseStamped pose2_in_pose1_frame;

        // 执行变换
        Eigen::Vector3d original_position(v_path_pose[msg->header.seq].pose.position.x,
                                          v_path_pose[msg->header.seq].pose.position.y,
                                          v_path_pose[msg->header.seq].pose.position.z);

        Eigen::Quaterniond original_quaternion(v_path_pose[msg->header.seq].pose.orientation.w,
                                               v_path_pose[msg->header.seq].pose.orientation.x,
                                               v_path_pose[msg->header.seq].pose.orientation.y,
                                               v_path_pose[msg->header.seq].pose.orientation.z);

        std::cout << "original position:\n" << original_position << std::endl
                  << "original_quat:\n" << original_quaternion.toRotationMatrix() << std::endl;

        Eigen::Quaterniond transformed_quaternion = Eigen::Quaterniond(inverse_transform.rotation()) * original_quaternion;
        Eigen::Vector3d transformed_position = inverse_transform * original_position;


        std::cout << "transformed position:\n" << transformed_position << std::endl
                  << "transformed_quat:\n" << transformed_quaternion.toRotationMatrix() << std::endl;
        // 更新geometry_msgs::Pose对象的位置
        pose2_in_pose1_frame.pose.position.x = transformed_position.x();
        pose2_in_pose1_frame.pose.position.y = transformed_position.y();
        pose2_in_pose1_frame.pose.position.z = transformed_position.z();

        // 更新geometry_msgs::Pose对象的方向
        pose2_in_pose1_frame.pose.orientation.x = transformed_quaternion.x();
        pose2_in_pose1_frame.pose.orientation.y = transformed_quaternion.y();
        pose2_in_pose1_frame.pose.orientation.z = transformed_quaternion.z();
        pose2_in_pose1_frame.pose.orientation.w = transformed_quaternion.w();

        const double kDegreeToRadian = M_PI / 180.;
        // for EuROC dataset
        const double sigma_pv = 0.01;
        const double sigma_rp = 0.5 * kDegreeToRadian;
        const double sigma_yaw = 0.5 * kDegreeToRadian;

        Eigen::Matrix<double, 6, 6> cov;
        cov.setZero();
        cov.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * sigma_pv * sigma_pv;
        cov.block<2, 2>(3, 3) = Eigen::Matrix2d::Identity() * sigma_rp * sigma_rp;
        cov(5, 5) = sigma_yaw * sigma_yaw;

        geometry_msgs::PoseWithCovarianceStamped pose_cov_stamped;
        pose_cov_stamped.header = v_path_pose[msg->header.seq].header;
        pose_cov_stamped.pose.pose = pose2_in_pose1_frame.pose;
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                pose_cov_stamped.pose.covariance[6 * i + j] = cov(i, j);
        pose_pub_.publish(pose_cov_stamped);
        ROS_INFO("Publish transform pose");
        return;
    }
}
