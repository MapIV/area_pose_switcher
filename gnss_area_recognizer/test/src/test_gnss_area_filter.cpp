// Copyright 2021 TierIV
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

#include "gtest/gtest.h"

#include "gnss_area_filter/gnss_area_filter.hpp"

TEST(GNSSAreaFilterTest, PoseExistsInPolygon)
{
  geometry_msgs::msg::Polygon polygon;
  geometry_msgs::msg::Point32 point;
  point.x = 0.0;
  point.y = 0.0;
  polygon.points.push_back(point);
  point.x = 1.0;
  point.y = 0.0;
  polygon.points.push_back(point);
  point.x = 1.0;
  point.y = 1.0;
  polygon.points.push_back(point);
  point.x = 0.0;
  point.y = 1.0;
  polygon.points.push_back(point);

  geometry_msgs::msg::Point targetPoint;
  targetPoint.x = 1.5;
  targetPoint.y = 1.5;
  EXPECT_EQ(GNSSAreaFilter::pointExistsInPolygon(targetPoint, polygon), false);

  targetPoint.x = 0.5;
  targetPoint.y = 0.5;
  EXPECT_EQ(GNSSAreaFilter::pointExistsInPolygon(targetPoint, polygon), true);
}

int main(int argc, char * argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
