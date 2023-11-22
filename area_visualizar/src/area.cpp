// Copyright 2023 Autoware Foundation
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

// #include "pose_estimator_manager/rule_helper/eagleye_area.hpp"
#include <area_visualizar/area.hpp>

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/logging.hpp>

#include <boost/geometry/geometry.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <unordered_set>
#include <vector>

namespace pose_estimator_manager::rule_helper
{
using BoostPoint = boost::geometry::model::d2::point_xy<double>;
using BoostPolygon = boost::geometry::model::polygon<BoostPoint>;

struct Area::Impl
{
  explicit Impl(rclcpp::Logger logger) : logger_(logger) {}
  std::vector<BoostPolygon> bounding_boxes_;
  void init(HADMapBin::ConstSharedPtr msg);
  std::string debug_string() const;
  MarkerArray debug_marker_array() const { return marker_array_; }

private:
  rclcpp::Logger logger_;
  MarkerArray marker_array_;
};

Area::Area(const rclcpp::Logger & logger) : logger_(logger)
{
  impl_ = std::make_shared<Impl>(logger_);
}


Area::Area(rclcpp::Node * node) : Area(node->get_logger())
{
  vector_map_subscriber_ = node->create_subscription<HADMapBin>(
    "/map/vector_map", 10,
    [this](const HADMapBin::ConstSharedPtr msg) {
      this->init(msg);
    });

  node->declare_parameter<std::string>("eagleye_area", "default_value");
  std::string eagleye_area_value;
  if (node->get_parameter("eagleye_area", eagleye_area_value)) {
    // パラメータの使用または処理
    // 例: `eagleye_area_value`を使用した初期化や設定
  }

  auto parameter_callback = [this](const std::vector<rclcpp::Parameter> &parameters) {
    for (const auto &parameter : parameters) {
      if (parameter.get_name() == "eagleye_area") {
        // `eagleye_area` パラメータが変更された場合の処理
        // 例: 新しい値を取得し、クラス内での使用を更新
      }
    }
  };
  node->register_param_change_callback(parameter_callback);
}

void Area::init(HADMapBin::ConstSharedPtr msg)
{
  vector_map_is_initialized_ = true;
  impl_->init(msg);
}

bool Area::vector_map_initialized() const
{
  return vector_map_is_initialized_;
}

std::string Area::debug_string() const
{
  return impl_->debug_string();
}

Area::MarkerArray Area::debug_marker_array() const
{
  return impl_->debug_marker_array();
}

void Area::Impl::init(HADMapBin::ConstSharedPtr msg)
{
  if (!bounding_boxes_.empty()) {
    // already initialized
    return;
  }

  lanelet::LaneletMapPtr lanelet_map(new lanelet::LaneletMap);
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map);

  const auto & po_layer = lanelet_map->polygonLayer;
  const std::unordered_set<std::string> bounding_box_labels_ = {"eagleye_area"};

  RCLCPP_INFO_STREAM(logger_, "Polygon layer size: " << po_layer.size());
  for (const auto & polygon : po_layer) {
    if (!polygon.hasAttribute(lanelet::AttributeName::Type)) {
      continue;
    }

    const lanelet::Attribute attr = polygon.attribute(lanelet::AttributeName::Type);
    RCLCPP_INFO_STREAM(logger_, "a polygon attribute: " << attr.value());

    if (bounding_box_labels_.count(attr.value()) == 0) {
      continue;
    }

    Marker marker;
    marker.type = Marker::LINE_STRIP;
    marker.scale.set__x(0.2f).set__y(0.2f).set__z(0.2f);
    marker.color.set__r(1.0f).set__g(1.0f).set__b(0.0f).set__a(1.0f);
    marker.ns = "eagleye_area";
    marker.header.frame_id = "map";
    marker.id = marker_array_.markers.size();

    BoostPolygon poly;
    for (const lanelet::ConstPoint3d & p : polygon) {
      poly.outer().push_back(BoostPoint(p.x(), p.y()));

      geometry_msgs::msg::Point point_msg;
      point_msg.set__x(p.x()).set__y(p.y()).set__z(p.z());
      marker.points.push_back(point_msg);
    }
    // to enclose the polygon
    poly.outer().push_back(poly.outer().front());

    bounding_boxes_.push_back(poly);
    marker.points.push_back(marker.points.front());
    marker_array_.markers.push_back(marker);
  }
}

std::string Area::Impl::debug_string() const
{
  std::stringstream ss;
  for (const BoostPolygon & box : bounding_boxes_) {
    for (const auto point : box.outer()) {
      ss << point.x() << "," << point.y() << " ";
    }
    ss << "\n";
  }
  return ss.str();
}

}  // namespace pose_estimator_manager::rule_helper
