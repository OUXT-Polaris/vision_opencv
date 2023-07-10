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

#ifndef OPENCV_COMPONENTS__OPENCV_CAMERA_HPP_
#define OPENCV_COMPONENTS__OPENCV_CAMERA_HPP_

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <atomic>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv_components/multi_object_tracker.hpp>
#include <rclcpp/rclcpp.hpp>

#include "opencv_components/visibility_control.h"

namespace opencv_components
{
class TrackingComponent : public rclcpp::Node
{
public:
  explicit TrackingComponent(const rclcpp::NodeOptions & options);

  virtual ~TrackingComponent();

private:
  void detectionCallback(
    const sensor_msgs::msg::Image & image,
    const perception_msgs::msg::Detection2DArray & detections);
  MultiObjectTracker tracker_;
  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
  message_filters::Subscriber<perception_msgs::msg::Detection2DArray> detections_sub_;
  std::unique_ptr<message_filters::TimeSynchronizer<
    sensor_msgs::msg::Image, perception_msgs::msg::Detection2DArray>>
    synchronizer_;
};

}  // namespace opencv_components

#endif  // OPENCV_COMPONENTS__OPENCV_CAMERA_HPP_
