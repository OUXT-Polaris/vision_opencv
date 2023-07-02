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

#include <opencv_components/opencv_camera_component.hpp>

namespace opencv_components
{

OpenCVCameraComponent::OpenCVCameraComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("opencv_camera_node", options),
  parameter_listener_(get_node_parameters_interface()),
  parameters_(parameter_listener_.get_params()),
  camera_pub_(this, "camera"),
  capture_(parameters_.camera_id),
  capture_thread_([this]() {
    cv::Mat frame;
    while (is_capturing_.load() && capture_.read(frame)) {
      if (frame.empty()) {
        continue;
      }
      camera_pub_.publish(
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg(),
        std::make_shared<sensor_msgs::msg::CameraInfo>(sensor_msgs::msg::CameraInfo()));
    }
  })
{
  is_capturing_.store(true);
}

OpenCVCameraComponent::~OpenCVCameraComponent()
{
  is_capturing_.store(false);
  capture_thread_.join();
}

}  // namespace opencv_components
