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

#include <opencv_components/tracking_component.hpp>

namespace opencv_components
{
TrackingComponent::TrackingComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("tracking_node", options),
  tracker_(0.5),
  image_sub_(this, "image"),
  detections_sub_(this, "detections_2d"),
  synchronizer_(std::make_unique<message_filters::TimeSynchronizer<
                  sensor_msgs::msg::Image, perception_msgs::msg::Detection2DArray>>(
    image_sub_, detections_sub_, 10))
{
}

TrackingComponent::~TrackingComponent() {}

}  // namespace opencv_components
