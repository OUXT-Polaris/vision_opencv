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

#ifndef OPENCV_COMPONENTS__MULTI_OBJECT_TRACKER_HPP_
#define OPENCV_COMPONENTS__MULTI_OBJECT_TRACKER_HPP_

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv_components/hungarian.hpp>
#include <opencv_components/util.hpp>
#include <optional>
#include <perception_msgs/msg/detection2_d.hpp>
#include <perception_msgs/msg/detection2_d_array.hpp>
#include <rclcpp/rclcpp.hpp>

namespace opencv_components
{
/// @sa https://docs.opencv.org/4.x/d0/d0a/classcv_1_1Tracker.html
enum class TrackingMethod { CSRT, DA_SIAM_RPN, GOTURN, KCF, MIL, /*NANO*/ };

class ObjectTracker
{
public:
  explicit ObjectTracker(
    const TrackingMethod method, const cv::Mat & image,
    const perception_msgs::msg::Detection2D & detection,
    const rclcpp::Duration & lifetime = rclcpp::Duration(std::chrono::milliseconds(100)));
  std::optional<cv::Rect> update(
    const cv::Mat & image, const perception_msgs::msg::Detection2D & detection);
  std::optional<cv::Rect> getRect() const;

private:
  const cv::Ptr<cv::Tracker> tracker_;
  const rclcpp::Duration lifetime_;
  const rclcpp::Time initialize_timestamp_;
  std::optional<cv::Rect> rect_;
};

class MultiObjectTracker
{
public:
  explicit MultiObjectTracker(
    const rclcpp::Duration & lifetime = rclcpp::Duration(std::chrono::milliseconds(100)));
  void update(const cv::Mat & image, const perception_msgs::msg::Detection2DArray & detections);

private:
  std::vector<cv::Rect> getRects() const;
  const rclcpp::Duration lifetime_;
  std::vector<ObjectTracker> trackers_;
};
}  // namespace opencv_components

#endif  // OPENCV_COMPONENTS__MULTI_OBJECT_TRACKER_HPP_
