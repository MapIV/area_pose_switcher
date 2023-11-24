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

#include <functional>
#include <memory>
#include <string>

#include <gnss_area_recognizer/gnss_area_recognizer.hpp>

#include <lanelet2_extension/utility/message_conversion.h>

GNSSAreaRecognizer::GNSSAreaRecognizer() : nh_(), pnh_()
{
  lanelet2_map_bin_sub_ =
    nh_.subscribe("input_lanelet2_map_bin_topic", 10, &GNSSAreaRecognizer::onLanelet2MapBin, this);
  pose_sub_ =
    nh_.subscribe("input_pose_with_covariance_topic", 10, &GNSSAreaRecognizer::onPose, this);

  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
    "output_pose_with_covariance_topic", 10);
  string_pub_ = nh_.advertise<std_msgs::String>("/localization/util/localization_type", 10);

  pnh_.param<std::string>("polygon_tag_name", polygon_tag_name_, "gnss_available_area");
  pnh_.param<std::string>("type_of_localization_in_area", type_of_localization_in_area_, "GNSS");
  pnh_.param<std::string>(
    "type_of_localization_out_of_area", type_of_localization_out_of_area_, "NDT");
}

void GNSSAreaRecognizer::onLanelet2MapBin(const autoware_lanelet2_msgs::MapBin::ConstPtr map_bin)
{
  lanelet::LaneletMapPtr lanelet_map_ptr{std::make_shared<lanelet::LaneletMap>()};
  lanelet::utils::conversion::fromBinMsg(*map_bin, lanelet_map_ptr);

  lanelet::ConstPolygons3d gnss_available_areas{};
  ROS_WARN("size:%d", lanelet_map_ptr->polygonLayer.size());
  for (const auto & poly : lanelet_map_ptr->polygonLayer) {
    const std::string type{poly.attributeOr(lanelet::AttributeName::Type, "none")};
    ROS_WARN("type:%s", type.c_str());
    if (type == polygon_tag_name_) {
      gnss_available_areas.push_back(poly);
    }
  }
  gnss_available_areas_opt_ = gnss_available_areas;
}

void GNSSAreaRecognizer::onPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr pose)
{
  if (!gnss_available_areas_opt_.has_value()) {
    ROS_WARN("No lanelet2 map.");
    return;
  }

  if (GNSSAreaRecognizer::poseExistsInGNSSAvailableArea(*pose, gnss_available_areas_opt_.value())) {
    pose_pub_.publish(*pose);
    std_msgs::String string_msg;
    string_msg.data = type_of_localization_in_area_;
    string_pub_.publish(string_msg);
  } else {
    std_msgs::String string_msg;
    string_msg.data = type_of_localization_out_of_area_;
    string_pub_.publish(string_msg);
  }
}

bool GNSSAreaRecognizer::poseExistsInGNSSAvailableArea(
  const geometry_msgs::PoseWithCovarianceStamped & pose,
  const lanelet::ConstPolygons3d & gnss_available_areas)
{
  geometry_msgs::Polygon polygon;
  for (const auto & gnss_available_area : gnss_available_areas) {
    lanelet::utils::conversion::toGeomMsgPoly(gnss_available_area, &polygon);
    if (GNSSAreaRecognizer::pointExistsInPolygon(pose.pose.pose.position, polygon)) {
      return true;
    }
  }
  // ROS_WARN(get_logger(), "poseExistsInGNSSAvailableArea false");
  return false;
}

bool GNSSAreaRecognizer::pointExistsInPolygon(
  const geometry_msgs::Point & point, const geometry_msgs::Polygon & polygon)
{
  // polygons with fewer than 3 sides are excluded
  if (polygon.points.size() < 3) {
    return false;
  }

  bool in_poly{false};
  float x1{0.0f};
  float x2{0.0f};
  float y1{0.0f};
  float y2{0.0f};

  uint32_t nr_poly_points = polygon.points.size();
  // start with the last point to make the check last point<->first point the first one
  float xold{polygon.points.at(nr_poly_points - 1).x};
  float yold{polygon.points.at(nr_poly_points - 1).y};
  for (const auto & poly_p : polygon.points) {
    const float xnew{poly_p.x};
    const float ynew{poly_p.y};
    if (xnew > xold) {
      x1 = xold;
      x2 = xnew;
      y1 = yold;
      y2 = ynew;
    } else {
      x1 = xnew;
      x2 = xold;
      y1 = ynew;
      y2 = yold;
    }

    if (
      (xnew < point.x) == (point.x <= xold) &&
      (point.y - y1) * (x2 - x1) < (y2 - y1) * (point.x - x1)) {
      in_poly = !in_poly;
    }
    xold = xnew;
    yold = ynew;
  }

  return in_poly;
}
