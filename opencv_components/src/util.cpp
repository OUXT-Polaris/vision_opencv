// Copyright (c) 2023 OUXT Polaris
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

#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <opencv_components/util.hpp>
#include <vision_msgs/msg/point2_d.hpp>
#include <vision_msgs/msg/pose2_d.hpp>

namespace opencv_components
{
cv::Rect toCVRect(const vision_msgs::msg::BoundingBox2D & msg)
{
  return cv::Rect(
    static_cast<int>(msg.center.position.x - 0.5 * msg.size_x),
    static_cast<int>(msg.center.position.y - 0.5 * msg.size_y), static_cast<int>(msg.size_x),
    static_cast<int>(msg.size_y));
}

vision_msgs::msg::BoundingBox2D toROSRect(const cv::Rect & rect)
{
  return vision_msgs::build<vision_msgs::msg::BoundingBox2D>()
    .center(vision_msgs::build<vision_msgs::msg::Pose2D>()
              .position(vision_msgs::build<vision_msgs::msg::Point2D>()
                          .x(rect.x + rect.width * 0.5)
                          .y(rect.y + rect.height * 0.5))
              .theta(0))
    .size_x(rect.width)
    .size_y(rect.height);
}

BoostRect toBoostRect(const cv::Rect & rect)
{
  return BoostRect(
    BoostPoint(rect.x, rect.y), BoostPoint(rect.x + rect.width, rect.y + rect.height));
}

BoostPolygon toBoostPolygon(const cv::Rect & rect)
{
  BoostPolygon poly;
  boost::geometry::exterior_ring(poly) =
    boost::assign::list_of<BoostPoint>(rect.x, rect.y)(rect.x + rect.width, rect.y)(
      rect.x, rect.y + rect.height)(rect.x + rect.width, rect.y + rect.height);
  return poly;
}

double getIoU(const cv::Rect & rect0, const cv::Rect & rect1)
{
  const auto box0 = toBoostRect(rect0);
  const auto box1 = toBoostRect(rect1);
  if (boost::geometry::disjoint(box0, box1)) {
    return 0;
  }
  std::vector<BoostPolygon> uinon_polygon;
  std::vector<BoostPolygon> intersection_polygon;
  boost::geometry::union_(toBoostPolygon(rect0), toBoostPolygon(rect1), uinon_polygon);
  boost::geometry::intersection(toBoostPolygon(rect0), toBoostPolygon(rect1), intersection_polygon);
  assert(intersection_polygon.size() == 1);
  assert(uinon_polygon.size() == 1);
  return boost::geometry::area(intersection_polygon[0]) / boost::geometry::area(uinon_polygon[0]);
}
}  // namespace opencv_components
