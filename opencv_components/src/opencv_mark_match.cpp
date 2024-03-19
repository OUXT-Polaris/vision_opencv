
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv_components/opencv_mark_match.hpp>
#include <rclcpp_components/register_node_macro.hpp>  //

namespace opencv_components
{
OpenCVMatchComponent::OpenCVMatchComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("opencv_mark_match", options),
  parameter_listener_(get_node_parameters_interface()),
  parameters_(parameter_listener_.get_params()),
  image_pub_(this, "image_")
{
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "camera", 1, [this](const sensor_msgs::msg::Image::SharedPtr image) { call_back(image); });
}

OpenCVMatchComponent::~OpenCVMatchComponent() {}

void OpenCVMatchComponent::call_back(const sensor_msgs::msg::Image::SharedPtr image_msg)
{
  std::string triangle =
    ament_index_cpp::get_package_share_directory("opencv_components") + "/picture/red_triangle.jpg";
  std::string circle =
    ament_index_cpp::get_package_share_directory("opencv_components") + "/picture/circle.jpg";
  std::string cross =
    ament_index_cpp::get_package_share_directory("opencv_components") + "/picture/cross.jpg";

  cv::imread(triangle).copyTo(sample_triangle);
  cv::imread(circle).copyTo(sample_circle);
  cv::imread(cross).copyTo(sample_cross);

  cv_bridge::CvImage bridge;
  sensor_msgs::msg::Image image = *image_msg.get();
  std::vector<std::vector<cv::Point> > triangle_contours, circle_contours, cross_contours, contours,
    matched_contours, selected_contours;

  const cv::Mat image_cv = cv_bridge::toCvCopy(image_msg)->image;

  if (sample_triangle.empty()) RCLCPP_INFO_STREAM(this->get_logger(), "no triangle_picture");
  if (sample_circle.empty()) RCLCPP_INFO_STREAM(this->get_logger(), "no circle_picture");
  if (sample_cross.empty()) RCLCPP_INFO_STREAM(this->get_logger(), "no cross_picture");

  if (image_cv.empty()) RCLCPP_INFO_STREAM(this->get_logger(), "no movie");

  cv::cvtColor(sample_triangle, triangle_hsv, cv::COLOR_BGR2HSV_FULL);
  cv::cvtColor(sample_circle, circle_hsv, cv::COLOR_BGR2HSV_FULL);
  cv::cvtColor(sample_cross, cross_hsv, cv::COLOR_BGR2HSV_FULL);

  cv::split(triangle_hsv, triangle_split);
  cv::split(circle_hsv, circle_split);
  cv::split(cross_hsv, cross_split);

  cv::dilate(triangle_split[2], temp, cv::Mat(), cv::Point(-1, -1), 3);
  cv::erode(temp, temp, cv::Mat(), cv::Point(-1, -1), 3 * 2);
  cv::dilate(temp, temp, cv::Mat(), cv::Point(-1, -1), 3);

  cv::findContours(
    temp, triangle_contours, triangle_hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  cv::dilate(circle_split[2], temp, cv::Mat(), cv::Point(-1, -1), 3);
  cv::erode(temp, temp, cv::Mat(), cv::Point(-1, -1), 3 * 2);
  cv::dilate(temp, temp, cv::Mat(), cv::Point(-1, -1), 3);

  cv::findContours(temp, circle_contours, circle_hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  cv::dilate(cross_split[2], temp, cv::Mat(), cv::Point(-1, -1), 3);
  cv::erode(temp, temp, cv::Mat(), cv::Point(-1, -1), 3 * 2);
  cv::dilate(temp, temp, cv::Mat(), cv::Point(-1, -1), 3);

  cv::findContours(temp, cross_contours, cross_hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  cv::cvtColor(image_cv, img_hsv, cv::COLOR_BGR2HSV_FULL);

  cv::cvtColor(image_cv, img_hsv, cv::COLOR_BGR2HSV_FULL);

  cv::split(img_hsv, img_split);

  cv::Canny(img_split[2], dst, 85, 85);

  cv::findContours(dst, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  for (size_t u = 0; u < contours.size(); u++) {
    for (size_t f = 0; f < triangle_contours.size(); f++) {
      match = cv::matchShapes(contours[u], triangle_contours[f], cv::CONTOURS_MATCH_I1, 0);
      if (parameters_.similarity > match)  //近似閾値
      {
        matched_contours.push_back(contours[u]);
      }
    }

    for (size_t f = 0; f < circle_contours.size(); f++) {
      match = cv::matchShapes(contours[u], circle_contours[f], cv::CONTOURS_MATCH_I1, 0);
      if (parameters_.similarity > match)  //近似閾値
      {
        matched_contours.push_back(contours[u]);
      }
    }

    for (size_t f = 0; f < cross_contours.size(); f++) {
      match = cv::matchShapes(contours[u], cross_contours[f], cv::CONTOURS_MATCH_I1, 0);
      if (parameters_.similarity > match)  //近似閾値
      {
        matched_contours.push_back(contours[u]);
      }
    }
  }

  for (size_t f = 0; f < matched_contours.size(); f++) {
    if (parameters_.min_area < cv::contourArea(matched_contours[f]))  //最小面積閾値
    {
      if (parameters_.max_area > cv::contourArea(matched_contours[f])) {  //最大面積閾値
        selected_contours.push_back(matched_contours[f]);
      }
    }
  }

  cv::Mat drawing = cv::Mat::zeros(dst.size(), CV_8UC3);

  for (size_t i = 0; i < selected_contours.size(); i++) {
    cv::Scalar color = cv::Scalar(255, 0, 0);
    cv::drawContours(drawing, selected_contours, (int)i, color);
  }

  image_pub_.publish(
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", drawing).toImageMsg(),
    std::make_shared<sensor_msgs::msg::CameraInfo>(sensor_msgs::msg::CameraInfo()));
}
}  // namespace opencv_components

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_components::OpenCVMatchComponent)
