#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"

class PoseSwitcher : public rclcpp::Node
{
public:
    PoseSwitcher() : Node("pose_switcher")
    {
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("selected_pose", 10);
        current_localization_type_pub_ = this->create_publisher<std_msgs::msg::String>("current_localization_type", 10);

        localization_type_sub_ = this->create_subscription<std_msgs::msg::String>(
            "localization_type",
            10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "localization_type has been changed: %s", msg->data.c_str());
                localization_type_data_ = msg->data;
                current_localization_type_pub_->publish(*msg);
            });

        gnss_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "gnss_pose",
            10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                if (localization_type_data_ == "GNSS" || localization_type_data_ == "gnss") {
                    pose_pub_->publish(*msg);
                }
            });

        lidar_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "lidar_pose",
            10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                if (localization_type_data_ == "LiDAR" || localization_type_data_ == "LIDAR" || localization_type_data_ == "lidar" ||
                    localization_type_data_ == "Ndt" || localization_type_data_ == "ndt" || localization_type_data_ == "NDT") {
                    pose_pub_->publish(*msg);
                }
            });

        RCLCPP_INFO(this->get_logger(), "localization_type: %s", localization_type_data_.c_str());
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_localization_type_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_type_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gnss_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr lidar_pose_sub_;

    std::string localization_type_data_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseSwitcher>());
    rclcpp::shutdown();
    return 0;
}
