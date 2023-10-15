
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
  std::vector<std::vector<cv::Point> > sample_contours,contours,contours2,contours3;
  std::vector<cv::Vec4i> hierarchy;
  cv::Mat sample,img_hsv,dst,mediam,img_split[3],temp;

  cv_bridge::CvImage bridge;
  sensor_msgs::msg::Image image = *image_msg.get();
  const cv::Mat image_cv = cv_bridge::toCvCopy(image_msg)->image;

  cv::imread("/home/yuasa/vision_test/picture/demo2.jpg",0).copyTo(sample);

  if(sample.empty())std::cout << "no picture" << std::endl;
  if(image_cv.empty())std::cout << "no movie" << std::endl;
  
  cv::dilate(sample, temp, cv::Mat(), cv::Point(-1,-1), 3);

  std::cout << "picture" << std::endl;
  
  cv::erode(temp, temp, cv::Mat(), cv::Point(-1,-1), 3*2);
  cv::dilate(temp, temp, cv::Mat(), cv::Point(-1,-1), 3);

  cv::findContours(temp, sample_contours, hierarchy,  cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
  
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

  cv::Mat drawing = cv::Mat::zeros(dst.size(), CV_8UC3);

  for( size_t i = 0; i< contours3.size(); i++ ) {
    cv::Scalar color = cv::Scalar(255, 0, 0);
    cv::drawContours(drawing, contours3, (int)i, color);
  }

  image_pub_.publish(
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", drawing).toImageMsg(),
    std::make_shared<sensor_msgs::msg::CameraInfo>(sensor_msgs::msg::CameraInfo()));
  }
}  // namespace match_components
