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

#ifndef OPENCV_COMPONENTS__OPENCV_MARK_MATCH_HPP_
#define OPENCV_COMPONENTS__OPENCV_MARK_MATCH_HPP_

#include <atomic>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "opencv_components/visibility_control.h"

namespace match_components
{
class OpenCVMatchComponent : public rclcpp::Node
{
public:
  explicit OpenCVMatchComponent(const rclcpp::NodeOptions & options);

  virtual ~OpenCVMatchComponent();

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  image_transport::CameraPublisher image_pub_;
  void call_back(const sensor_msgs::msg::Image::SharedPtr image_msg);
  
  std::vector<cv::Vec4i> triangle_hierarchy,circle_hierarchy,cross_hierarchy;
  cv::Mat sample_circle,sample_cross,sample_triangle,triangle_hsv,circle_hsv,cross_hsv,triangle_split[3],circle_split[3],cross_split[3],
          img_hsv,dst,mediam,img_split[3],temp,edge;

  cv::Mat drawing;
  double match;
};

}  // namespace match_components

#endif  // OPENCV_COMPONENTS__OPENCV_CAMERA_HPP_