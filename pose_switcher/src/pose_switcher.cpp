#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tier4_debug_msgs/msg/float32_stamped.hpp"
#include "map4_localization_msgs/msg/localization_type_stamped.hpp"

#include <deque>

#include "pose_array_interpolator.hpp"
#include "util_func.hpp"

class PoseSwitcher : public rclcpp::Node
{
public:
    PoseSwitcher() : Node("pose_switcher")
    {
        // Parameters
        this->declare_parameter("error_2d_threshold", 0.3); // [m]
        this->declare_parameter("error_yaw_threshold", 1.0); // [deg]
        this->declare_parameter("judge_switching_ealpsed_time_threshold", 3.0);

        this->get_parameter("error_2d_threshold", error_2d_threshold_);
        this->get_parameter("error_yaw_threshold", error_yaw_threshold_);
        this->get_parameter("judge_switching_ealpsed_time_threshold", judge_switching_ealpsed_time_threshold_);

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("selected_pose", 1);
        error_2d_pub_ = this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>("error_2d", 1);
        error_yaw_pub_ = this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>("error_yaw", 1);
        current_localization_type_pub_ = this->create_publisher<map4_localization_msgs::msg::LocalizationTypeStamped>("/localization/pose_estimator/current_localization_type", 1);

        area_localization_type_sub_ = this->create_subscription<map4_localization_msgs::msg::LocalizationTypeStamped>(
            "area_localization_type",
            1,
            [this](const map4_localization_msgs::msg::LocalizationTypeStamped::SharedPtr msg) {
                area_localization_type_msg_ = *msg;
                if(!is_initialized_) {
                    current_localization_type_msg_ = *msg;
                    current_localization_type_msg_.data = map4_localization_msgs::msg::LocalizationTypeStamped::NDT;
                    is_initialized_ = true;
                }
                current_localization_type_pub_->publish(current_localization_type_msg_);

                if(is_switched_){
                    if(area_localization_type_msg_.data != map4_localization_msgs::msg::LocalizationTypeStamped::SWITCHING)
                    {
                        RCLCPP_INFO(this->get_logger(), "is_switched_ reset");
                        is_switched_ = false;
                    }
                }

            });

        gnss_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "gnss_pose",
            1,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                if(!is_gnss_pose_received_) {
                    is_gnss_pose_received_ = true;
                }
                gnss_pose_cov_msg_ptr_array_.push_back(msg);
                gnss_pose_msg_ptr_ = msg;

                if(current_localization_type_msg_.data == map4_localization_msgs::msg::LocalizationTypeStamped::GNSS) {
                   pose_pub_->publish(*msg);
                }
            });

        lidar_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "lidar_pose",
            1,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                if(current_localization_type_msg_.data == map4_localization_msgs::msg::LocalizationTypeStamped::NDT) {
                    pose_pub_->publish(*msg);
                }

                lidar_pose_cov_msg_ptr_array_.push_back(msg);
                lidar_pose_msg_ptr_ = msg;
                if(is_gnss_pose_received_) {
                    double t = rclcpp::Time(area_localization_type_msg_.stamp).seconds() - 1.6843928e+09;
                    // std::cout << "t: " << std::setprecision(9) << t << std::endl;
                    // std::cout << "area_localization_type_msg_.data: " << area_localization_type_msg_.data << std::endl;
                    // RCLCPP_INFO(this->get_logger(), "area_localization_type_msg_.data: %d", area_localization_type_msg_.data);
                    if(is_switched_){
                        // RCLCPP_INFO(this->get_logger(), "is_switched_ == true");
                        return;
                    }

                    Error2dYaw error_2d_yaw;
                    bool is_gnss_area = (area_localization_type_msg_.data == map4_localization_msgs::msg::LocalizationTypeStamped::GNSS);
                    if(!is_gnss_area) {
                        if(!computeError(error_2d_yaw)) {
                            // RCLCPP_WARN(this->get_logger(), "computeError() == false");
                            return;
                        }
                        else
                        {
                            // RCLCPP_INFO(this->get_logger(), "computeError() == true");
                        }
                    }
                    // else
                    // {
                    //     RCLCPP_INFO(this->get_logger(), "is_gnss_area == true");
                    // }

                    bool is_switching_area = (area_localization_type_msg_.data == map4_localization_msgs::msg::LocalizationTypeStamped::SWITCHING);
                    if(is_switching_area)
                    {
                        if(!judgetSwitching(error_2d_yaw))
                        {
                            RCLCPP_WARN(this->get_logger(), "judgetSwitching() == false");
                            return;
                        }
                        else
                        {
                            is_switched_ = true;
                            current_localization_type_msg_.stamp = area_localization_type_msg_.stamp;
                            if(current_localization_type_msg_.data == map4_localization_msgs::msg::LocalizationTypeStamped::GNSS) {
                                current_localization_type_msg_.data = map4_localization_msgs::msg::LocalizationTypeStamped::NDT;
                            }
                            else if(current_localization_type_msg_.data == map4_localization_msgs::msg::LocalizationTypeStamped::NDT) {
                                current_localization_type_msg_.data = map4_localization_msgs::msg::LocalizationTypeStamped::GNSS;
                            }
                        }
                    }

                // std::cout << "-----------------" << std::endl;
                }

            });

        // Initialization
        gnss_pose_msg_ptr_ = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
        lidar_pose_msg_ptr_ = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    }

private:
    struct Error2dYaw {
        double time;
        double error_2d;
        double error_yaw;
    };

    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& quaternion) {
        tf2::Quaternion q(
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        return yaw;
    }

    bool judgetSwitching(const Error2dYaw& error_2d_yaw) {

        if (fabs(error_2d_yaw.error_2d) > error_2d_threshold_ ) {
            judge_switching_first_time_ = -1.0;
            RCLCPP_INFO(this->get_logger(), "2d error is too large");
            return false;
        }
        if(fabs(error_2d_yaw.error_yaw) > error_yaw_threshold_)
        {
            judge_switching_first_time_ = -1.0;
            RCLCPP_INFO(this->get_logger(), "yaw error is too large");
            return false;
        }
        if (judge_switching_first_time_ < 0.0) {
            judge_switching_first_time_ = error_2d_yaw.time;
            RCLCPP_INFO(this->get_logger(), "Start to judge switching?");
            return false;
        }
        double elapsed_time = error_2d_yaw.time - judge_switching_first_time_;
        if (elapsed_time > judge_switching_ealpsed_time_threshold_) {
            judge_switching_first_time_ = -1.0;
            RCLCPP_INFO(this->get_logger(), "Switching!");
            return true;
        }
        RCLCPP_INFO(this->get_logger(), "elapsed_time: %f", elapsed_time);
        return false;
    }

    bool computeError(Error2dYaw& error_2d_yaw) {
        rclcpp::Time sensor_ros_time =  gnss_pose_msg_ptr_->header.stamp;
        if (lidar_pose_cov_msg_ptr_array_.size() <= 1) {
            // RCLCPP_WARN(this->get_logger(), "lidar_pose_cov_msg_ptr_array_.size() <= 1");
            return false;
        }
        PoseArrayInterpolator interpolator(this, sensor_ros_time, lidar_pose_cov_msg_ptr_array_);
        if (!interpolator.is_success())
        {
            RCLCPP_WARN(this->get_logger(), "interpolator.is_success() == false");
            return false;
        }
        if (rclcpp::Time(interpolator.get_current_pose().header.stamp).seconds() == 0.0) {
            RCLCPP_WARN(this->get_logger(), "interpolator.get_current_pose().header.stamp == 0.0");
            return false;
        }
        pop_old_pose(lidar_pose_cov_msg_ptr_array_, sensor_ros_time);

        // geometry_msgs::msg::Pose lidar_pose = lidar_pose_msg_ptr_->pose.pose;
        // geometry_msgs::msg::Pose gnss_pose = interpolator.get_current_pose().pose.pose;
        geometry_msgs::msg::Pose gnss_pose = lidar_pose_msg_ptr_->pose.pose;
        geometry_msgs::msg::Pose lidar_pose = interpolator.get_current_pose().pose.pose;
        // 2D error
        double error_2d = sqrt(pow(gnss_pose.position.x - lidar_pose.position.x, 2) +
                               pow(gnss_pose.position.y - lidar_pose.position.y, 2));
        // yaw error
        double lidar_yaw = getYawFromQuaternion(lidar_pose.orientation);
        double gnss_yaw = getYawFromQuaternion(gnss_pose.orientation);
        double error_yaw = lidar_yaw - gnss_yaw;
        while (error_yaw > M_PI) error_yaw -= 2.0 * M_PI;
        while (error_yaw < -M_PI) error_yaw += 2.0 * M_PI;

        // std::cout << "time: " << std::fixed << std::setprecision(9) << rclcpp::Time(sensor_ros_time).seconds() << std::endl;
        // std::cout << "gnss x y: " << std::fixed << std::setprecision(9) << gnss_pose.position.x << " " << gnss_pose.position.y << std::endl;
        // std::cout << "lidar x y: " << std::fixed << std::setprecision(9) << lidar_pose.position.x << " " << lidar_pose.position.y << std::endl;
        // std::cout << "error_2d: " << error_2d << std::endl;
        // std::cout << "error_yaw: " << error_yaw * 180.0 / M_PI << std::endl;

        tier4_debug_msgs::msg::Float32Stamped error_2d_msg;
        error_2d_msg.stamp = sensor_ros_time;
        error_2d_msg.data = error_2d;
        error_2d_pub_->publish(error_2d_msg);

        tier4_debug_msgs::msg::Float32Stamped error_yaw_msg;
        error_yaw_msg.stamp = sensor_ros_time;
        error_yaw_msg.data = error_yaw;
        error_yaw_pub_->publish(error_yaw_msg);

        error_2d_yaw.time = sensor_ros_time.seconds();
        error_2d_yaw.error_2d = error_2d;
        error_2d_yaw.error_yaw = error_yaw;
        return true;
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<map4_localization_msgs::msg::LocalizationTypeStamped>::SharedPtr current_localization_type_pub_;
    rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr error_2d_pub_;
    rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr error_yaw_pub_;

    rclcpp::Subscription<map4_localization_msgs::msg::LocalizationTypeStamped>::SharedPtr area_localization_type_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr lidar_pose_sub_;

    std::string localization_type_data_;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr gnss_pose_msg_ptr_;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr lidar_pose_msg_ptr_;
    bool is_gnss_pose_received_;

    std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr>
    gnss_pose_cov_msg_ptr_array_, lidar_pose_cov_msg_ptr_array_;

    double error_2d_threshold_, error_yaw_threshold_;
    double judge_switching_ealpsed_time_threshold_;
    double judge_switching_first_time_ = -1.0;

    map4_localization_msgs::msg::LocalizationTypeStamped current_localization_type_msg_, area_localization_type_msg_;

    bool is_initialized_ = false;

    bool is_switched_ = false;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseSwitcher>());
    rclcpp::shutdown();
    return 0;
}
