#include <opencv_components/opencv_camera_component.hpp>

namespace opencv_components
{

OpenCVCameraComponent::OpenCVCameraComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("opencv_camera_node", options),
  parameter_listener_(get_node_parameters_interface()),
  parameters_(parameter_listener_.get_params()),
  camera_pub_(this, "camera"),
  capture_(parameters.camera_id),
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
