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

#include "gnss_area_filter/gnss_area_filter.hpp"

#include "lanelet2_extension/utility/message_conversion.hpp"

GNSSAreaFilter::GNSSAreaFilter()
: Node("gnss_area_filter"),
  polygon_tag_name_(declare_parameter("polygon_tag_name", "gnss_available_area")),
  polygon_tag_name2_(declare_parameter("polygon_tag_name2", "switching_area"))
{
  lanelet2_map_bin_sub_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "input_lanelet2_map_bin_topic", rclcpp::QoS{1}.transient_local(),
    std::bind(&GNSSAreaFilter::onLanelet2MapBin, this, std::placeholders::_1));
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "input_pose_with_covariance_topic", 10,
    std::bind(&GNSSAreaFilter::onPose, this, std::placeholders::_1));

  area_localization_type_pub_ = this->create_publisher<tier4_localization_msgs::msg::LocalizationTypeStamped>(
    "area_localization_type", 10);
}

void GNSSAreaFilter::onLanelet2MapBin(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr map_bin)
{
  lanelet::LaneletMapPtr lanelet_map_ptr{std::make_shared<lanelet::LaneletMap>()};
  lanelet::utils::conversion::fromBinMsg(*map_bin, lanelet_map_ptr);

  lanelet::ConstPolygons3d gnss_available_areas{}, switching_areas{};
  for (const auto & poly : lanelet_map_ptr->polygonLayer) {
    const std::string type{poly.attributeOr(lanelet::AttributeName::Type, "none")};
    if (type == polygon_tag_name_) {
      gnss_available_areas.push_back(poly);
    }
    else if (type == polygon_tag_name2_) {
      switching_areas.emplace_back(poly);
    }
  }
  gnss_available_areas_opt_ = gnss_available_areas;
  switching_areas_opt_ = switching_areas;
}

void GNSSAreaFilter::onPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose)
{
  if (!gnss_available_areas_opt_.has_value() || !switching_areas_opt_.has_value()) {
    RCLCPP_WARN(get_logger(), "No lanelet2 map.");
    return;
  }

  tier4_localization_msgs::msg::LocalizationTypeStamped area_localization_type{};
  if (GNSSAreaFilter::poseExistsInGNSSAvailableArea(*pose, gnss_available_areas_opt_.value())) {
    area_localization_type.data = tier4_localization_msgs::msg::LocalizationTypeStamped::GNSS;
  }
  else if (GNSSAreaFilter::poseExistsInGNSSAvailableArea(*pose, switching_areas_opt_.value())) {
    area_localization_type.data = tier4_localization_msgs::msg::LocalizationTypeStamped::SWITCHING;
  }
  else
  {
    area_localization_type.data = tier4_localization_msgs::msg::LocalizationTypeStamped::NDT;
  }
  area_localization_type.stamp = pose->header.stamp;
  count_++;
  if (count_ % 10 == 0) {
    area_localization_type_pub_->publish(area_localization_type);
  }
}

bool GNSSAreaFilter::poseExistsInGNSSAvailableArea(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose,
  const lanelet::ConstPolygons3d & gnss_available_areas)
{
  geometry_msgs::msg::Polygon polygon;
  for (const auto & gnss_available_area : gnss_available_areas) {
    lanelet::utils::conversion::toGeomMsgPoly(gnss_available_area, &polygon);
    if (GNSSAreaFilter::pointExistsInPolygon(pose.pose.pose.position, polygon)) {
      return true;
    }
  }
  return false;
}

bool GNSSAreaFilter::pointExistsInPolygon(
  const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Polygon & polygon)
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
      (point.y - y1) * (x2 - x1) < (y2 - y1) * (point.x - x1))
    {
      in_poly = !in_poly;
    }
    xold = xnew;
    yold = ynew;
  }

  return in_poly;
}
