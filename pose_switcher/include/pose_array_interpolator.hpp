#ifndef POSE_ARRAY_INTERPOLATOR_HPP
#define POSE_ARRAY_INTERPOLATOR_HPP

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <deque>

#include "util_func.hpp"

class PoseArrayInterpolator
{
private:
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

public:
  PoseArrayInterpolator(
    rclcpp::Node * node, const rclcpp::Time target_ros_time,
    const std::deque<PoseWithCovarianceStamped::ConstSharedPtr> & pose_msg_ptr_array,
    const double & pose_timeout_sec, const double & pose_distance_tolerance_meters);

  PoseArrayInterpolator(
    rclcpp::Node * node, const rclcpp::Time target_ros_time,
    const std::deque<PoseWithCovarianceStamped::ConstSharedPtr> & pose_msg_ptr_array);

  PoseWithCovarianceStamped get_current_pose();
  PoseWithCovarianceStamped get_old_pose();
  PoseWithCovarianceStamped get_new_pose();
  bool is_success();

private:
  rclcpp::Logger logger_;
  rclcpp::Clock clock_;
  const PoseWithCovarianceStamped::SharedPtr current_pose_ptr_;
  PoseWithCovarianceStamped::SharedPtr old_pose_ptr_;
  PoseWithCovarianceStamped::SharedPtr new_pose_ptr_;
  bool success_;

  bool validate_time_stamp_difference(
    const rclcpp::Time & target_time, const rclcpp::Time & reference_time,
    const double time_tolerance_sec) const;
  bool validate_position_difference(
    const geometry_msgs::msg::Point & target_point,
    const geometry_msgs::msg::Point & reference_point, const double distance_tolerance_m_) const;
};

#endif  // POSE_ARRAY_INTERPOLATOR_HPP