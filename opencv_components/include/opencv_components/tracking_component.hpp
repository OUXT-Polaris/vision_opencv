#ifndef OPENCV_COMPONENTS__OPENCV_CAMERA_HPP_
#define OPENCV_COMPONENTS__OPENCV_CAMERA_HPP_

#include <atomic>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
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
  cv::Ptr<cv::Tracker> tracker_;
};

}  // namespace opencv_components

#endif  // OPENCV_COMPONENTS__OPENCV_CAMERA_HPP_
