
#include <opencv_components/opencv_mark_match.hpp>
namespace match_components
{
OpenCVMatchComponent::OpenCVMatchComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("opencv_mark_match", options),
  image_pub_(this, "image_")
{
  image_sub_ =   create_subscription<sensor_msgs::msg::Image>(
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

  cv::imread("demo2.jpg",0).copyTo(sample);

  std::vector<std::vector<cv::Point> > contours,sample_contours,contours2,contours3;

  //cv::findContours(sample, sample_contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );

  cv::cvtColor(image_cv,img_hsv,cv::COLOR_BGR2HSV_FULL);

  cv::split(img_hsv,img_split);

  cv::medianBlur(img_split[2],mediam,3);

  cv::threshold(mediam,dst,0,255,cv::THRESH_OTSU);

  cv::findContours(dst, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  for(int u = 0;u < contours.size();u++)
  {
    int p = 0;
    for(int f = 0;f < sample_contours.size();f++)
    {
       match = cv::matchShapes(contours[u],sample_contours[f],cv::CONTOURS_MATCH_I1,0);
      if(match < 0.15)
      {
       p = 1;
      }
    }

    if(p != 0){
      contours2.push_back(contours[u]);
    }
  }

  for( size_t i = 0; i < contours2.size(); i++ ) {
    if(90 < cv::contourArea(contours2[i]))contours3.push_back(contours2[i]);
  }

  //cv::Mat drawing = cv::Mat::zeros(dst.size(), CV_8UC3);

  cv::Mat drawing;

  std::cout << "koko" << std::endl;

  for( size_t i = 0; i< contours3.size(); i++ ) {
    cv::Scalar color = cv::Scalar(255, 0, 0);
    cv::drawContours(drawing, contours3, (int)i, color);
  }

  image_pub_.publish(
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", drawing).toImageMsg(),
    std::make_shared<sensor_msgs::msg::CameraInfo>(sensor_msgs::msg::CameraInfo()));
  }
}  // namespace match_components
