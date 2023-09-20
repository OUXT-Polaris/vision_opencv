
#include <opencv_components/opencv_mark_match.hpp>
namespace match_components
{
OpenCVMatchComponent::OpenCVMatchComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("opencv_mark_match", options),
  image_pub_(this, "image_")
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
  const cv::Mat image_cv = cv_bridge::toCvCopy(image_msg)->image;

  cv::imread("../picture/demo2.jpg",0).copyTo(src2);

  image_cv.copyTo(src);

  std::vector<std::vector<cv::Point> > contours,contours3,contours4;
  /*std::vector<std::vector<cv::Point> > contours2;
  cv::findContours(src2, contours2, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );*/

  cv::cvtColor(src,src,cv::COLOR_BGR2HSV_FULL);

  cv::split(src,fusion);

  cv::medianBlur(fusion[2],mediam,3);

  cv::threshold(mediam,dst,0,255,cv::THRESH_OTSU);

  if(dst){
    
  }

  cv::findContours(dst, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  image_pub_.publish(
        cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", dst).toImageMsg(),
        std::make_shared<sensor_msgs::msg::CameraInfo>(sensor_msgs::msg::CameraInfo()));
  }
}  // namespace match_components
