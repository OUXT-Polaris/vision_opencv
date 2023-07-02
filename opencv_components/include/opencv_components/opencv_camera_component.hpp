#ifndef OPENCV_COMPONENTS__OPENCV_CAMERA_HPP_
#define OPENCV_COMPONENTS__OPENCV_CAMERA_HPP_

#include <atomic>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
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
  image_transport::CameraPublisher camera_pub_;
  cv::VideoCapture capture_;
  std::thread capture_thread_;
  std::atomic<bool> is_capturing_;
};

}  // namespace opencv_components

#endif  // OPENCV_COMPONENTS__OPENCV_CAMERA_HPP_
