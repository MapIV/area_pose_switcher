// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GNSS_AREA_FILTER__GNSS_AREA_FILTER_HPP_
#define GNSS_AREA_FILTER__GNSS_AREA_FILTER_HPP_

#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include <lanelet2_core/LaneletMap.h>
#include "lanelet2_extension/utility/utilities.hpp"

#include "autoware_auto_mapping_msgs/msg/had_map_bin.hpp"
#include "map4_localization_msgs/msg/localization_type_stamped.hpp"

class GNSSAreaFilter : public rclcpp::Node
{
public:
  GNSSAreaFilter();

  static bool poseExistsInGNSSAvailableArea(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose,
    const lanelet::ConstPolygons3d & gnss_available_areas);
  static bool pointExistsInPolygon(
    const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Polygon & polygon);

private:
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr lanelet2_map_bin_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<map4_localization_msgs::msg::LocalizationTypeStamped>::SharedPtr area_localization_type_pub_;

  std::string polygon_tag_name_, polygon_tag_name2_;
  std::optional<lanelet::ConstPolygons3d> gnss_available_areas_opt_, switching_areas_opt_;

  void onLanelet2MapBin(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr map_bin);
  void onPose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose);

  int count_ = 0;
};

#endif  // GNSS_AREA_FILTER__GNSS_AREA_FILTER_HPP_
