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

#include <opencv2/opencv.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>

namespace opencv_components
{
cv::Rect toCVRect(const vision_msgs::msg::BoundingBox2D & msg);
}  // namespace opencv_components

#endif  // OPENCV_COMPONENTS__UTIL_HPP_
