
#include <opencv_components/opencv_mark_match.hpp>
namespace match_components
{
OpenCVMatchComponent::OpenCVMatchComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("opencv_mark_match", options)
{
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "camera", 1, [this](const sensor_msgs::msg::Image::SharedPtr image) {
      call_back(image);
    });  //1,引数の型２，トピック名３，バッファサイズ４，関数オブジェクと
}  //ラムダは関数オブジェクト

OpenCVMatchComponent::~OpenCVMatchComponent() {}

void OpenCVMatchComponent::call_back(const sensor_msgs::msg::Image::SharedPtr image_msg)
{
  cv_bridge::CvImage bridge;
  sensor_msgs::msg::Image image = *image_msg.get();
  bridge.toImageMsg(image);

    // =Mat

  while(true){
    cv::imshow("draw",bridge.image);
    if(cv::waitKey(1) >= 0) break;
  }

}
}  // namespace match_components
