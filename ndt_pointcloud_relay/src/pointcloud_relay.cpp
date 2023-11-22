#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "map4_localization_msgs/msg/localization_type_stamped.hpp"

class PointCloudRelay : public rclcpp::Node
{
public:
  PointCloudRelay()
  : Node("point_cloud_relay")
  {
    // pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensing/lidar/top/outlier_filtered/relayed_pointcloud", 10);
    // pointcloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/sensing/lidar/top/outlier_filtered/pointcloud", 10, std::bind(&PointCloudRelay::pointcloud_callback, this, std::placeholders::_1));
    pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensing/lidar/top/outlier_filtered/relayed_pointcloud", rclcpp::SensorDataQoS().keep_last(10));
    pointcloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/sensing/lidar/top/outlier_filtered/pointcloud", rclcpp::SensorDataQoS(), std::bind(&PointCloudRelay::pointcloud_callback, this, std::placeholders::_1));

    localization_subscription_ = this->create_subscription<map4_localization_msgs::msg::LocalizationTypeStamped>("/area_localization_type", 1, std::bind(&PointCloudRelay::localization_callback, this, std::placeholders::_1));
  }

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (area_localization_type_ != map4_localization_msgs::msg::LocalizationTypeStamped::GNSS) {
      pointcloud_publisher_->publish(*msg);
    }
  }

  void localization_callback(const map4_localization_msgs::msg::LocalizationTypeStamped::SharedPtr msg)
  {
    area_localization_type_ = msg->data;
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
  rclcpp::Subscription<map4_localization_msgs::msg::LocalizationTypeStamped>::SharedPtr localization_subscription_;

  int area_localization_type_ = map4_localization_msgs::msg::LocalizationTypeStamped::NDT;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudRelay>());
  rclcpp::shutdown();
  return 0;
}

