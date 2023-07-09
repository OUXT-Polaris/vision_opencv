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

#ifndef OPENCV_COMPONENTS__UTIL_HPP_
#define OPENCV_COMPONENTS__UTIL_HPP_

#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <opencv2/opencv.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>

namespace opencv_components
{
using BoostPoint = boost::geometry::model::d2::point_xy<int>;
using BoostRect = boost::geometry::model::box<BoostPoint>;
using BoostPolygon = boost::geometry::model::polygon<BoostPoint>;

cv::Rect toCVRect(const vision_msgs::msg::BoundingBox2D & msg);
vision_msgs::msg::BoundingBox2D toROSRect(const cv::Rect & rect);
BoostRect toBoostRect(const cv::Rect & rect);
BoostPolygon toBoostPolygon(const cv::Rect & rect);
double getIoU(const cv::Rect & rect0, const cv::Rect & rect1);
}  // namespace opencv_components

#endif  // OPENCV_COMPONENTS__UTIL_HPP_
