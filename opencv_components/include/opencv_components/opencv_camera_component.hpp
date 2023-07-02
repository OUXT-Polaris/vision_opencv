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

#include <atomic>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv_camera_parameters.hpp>
#include <rclcpp/rclcpp.hpp>

#include "opencv_components/visibility_control.h"

namespace opencv_components
{

class OpenCVCameraComponent : public rclcpp::Node
{
public:
  explicit OpenCVCameraComponent(const rclcpp::NodeOptions & options);

  virtual ~OpenCVCameraComponent();

private:
  opencv_camera_components::ParamListener parameter_listener_;
  opencv_camera_components::Params parameters_;
  image_transport::CameraPublisher camera_pub_;
  cv::VideoCapture capture_;
  std::thread capture_thread_;
  std::atomic<bool> is_capturing_;
};

}  // namespace opencv_components

#endif  // OPENCV_COMPONENTS__OPENCV_CAMERA_HPP_
