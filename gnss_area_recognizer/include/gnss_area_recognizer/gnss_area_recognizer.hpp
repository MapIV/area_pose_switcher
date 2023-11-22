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

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/String.h>

#include <lanelet2_extension/utility/utilities.h>

#include <autoware_lanelet2_msgs/MapBin.h>

class GNSSAreaRecognizer
{
public:
  GNSSAreaRecognizer();

  static bool poseExistsInGNSSAvailableArea(
    const geometry_msgs::PoseWithCovarianceStamped & pose,
    const lanelet::ConstPolygons3d & gnss_available_areas);
  static bool pointExistsInPolygon(
    const geometry_msgs::Point & point, const geometry_msgs::Polygon & polygon);

private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber lanelet2_map_bin_sub_;
  ros::Subscriber pose_sub_;

  ros::Publisher pose_pub_;
  ros::Publisher string_pub_;

  std::string polygon_tag_name_;
  std::string type_of_localization_in_area_;
  std::string type_of_localization_out_of_area_;
  std::optional<lanelet::ConstPolygons3d> gnss_available_areas_opt_;

  void onLanelet2MapBin(const autoware_lanelet2_msgs::MapBin::ConstPtr map_bin);
  void onPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr pose);
};

#endif  // GNSS_AREA_FILTER__GNSS_AREA_FILTER_HPP_
