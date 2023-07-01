#include <opencv_components/opencv_camera_component.hpp>

namespace opencv_components
{

OpenCVCameraComponent::OpenCVCameraComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("opencv_camera_node", options),
  camera_pub_(this, "camera"),
  capture_(0),
  capture_thread_([this]() {
    cv::Mat frame;
    while (capture_.read(frame)) {
    }
  })
{
}

OpenCVCameraComponent::~OpenCVCameraComponent() {}

}  // namespace opencv_components
