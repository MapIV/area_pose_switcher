#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
class PoseSwitcher
{
public:
  PoseSwitcher();
  ~PoseSwitcher();

private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pose_pub_;
  ros::Publisher current_localization_type_pub_;
  ros::Subscriber gnss_pose_sub_;
  ros::Subscriber lidar_pose_sub_;
  ros::Subscriber localization_type_sub_;

  std:: string localization_type_data_ = "NONE";

  void callbackLocalizationType(const std_msgs::String & localization_type);
  void callbackGNSSPose(const geometry_msgs::PoseStamped & gnss_pose);
  void callbackLiDARPose(const geometry_msgs::PoseStamped & lidar_pose);
};

