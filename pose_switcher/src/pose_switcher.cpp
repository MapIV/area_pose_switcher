#include "pose_switcher/pose_switcher.h"

PoseSwitcher::PoseSwitcher() : nh_(""), pnh_("")
{
    pose_pub_ = pnh_.advertise<geometry_msgs::PoseStamped>("selected_pose", 1000);
    current_localization_type_pub_ = pnh_.advertise<std_msgs::String>("current_localization_type", 1, true);
    localization_type_sub_ = nh_.subscribe("localization_type", 1,  &PoseSwitcher::callbackLocalizationType, this);
    gnss_pose_sub_ = nh_.subscribe("gnss_pose", 1000, &PoseSwitcher::callbackGNSSPose, this);
    lidar_pose_sub_ = nh_.subscribe("lidar_pose", 1000, &PoseSwitcher::callbackLiDARPose, this);

    
    std_msgs::String localization_type;
    localization_type.data = localization_type_data_;
    current_localization_type_pub_.publish(localization_type);

    ROS_INFO("localization_type: %s", localization_type.data.c_str());
};

PoseSwitcher::~PoseSwitcher(){};

void PoseSwitcher::callbackLocalizationType(const std_msgs::String & localization_type)
{
    ROS_INFO("localization_type has been changed: %s", localization_type.data.c_str());
    localization_type_data_ = localization_type.data;
    current_localization_type_pub_.publish(localization_type);
}

void PoseSwitcher::callbackGNSSPose(const geometry_msgs::PoseStamped & gnss_pose)
{
    bool publication_flag = (localization_type_data_ == "GNSS" || localization_type_data_ == "gnss");
    if(!publication_flag) return;
    pose_pub_.publish(gnss_pose);
}

void PoseSwitcher::callbackLiDARPose(const geometry_msgs::PoseStamped & lidar_pose)
{
    bool publication_flag_lidar = (localization_type_data_ == "LiDAR" || localization_type_data_ == "LIDAR" || localization_type_data_ == "lidar");
    bool publication_flag_ndt = (localization_type_data_ == "Ndt" || localization_type_data_ == "ndt" || localization_type_data_ == "NDT");
    bool publication_flag = publication_flag_lidar || publication_flag_ndt;
    if(!publication_flag) return;
    pose_pub_.publish(lidar_pose);
}