
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
  const cv::Mat image_cv = cv_bridge::toCvCopy(image_msg)->image;
  
  cv::imread("../picture/demo2.jpg",0).copyTo(src2);
  
  cv::findContours(src2, contours2, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );

  while(true){
    std::vector<std::vector<cv::Point> > contours,contours3,contours4;

    cv::cvtColor(image_cv,src,cv::COLOR_BGR2HSV_FULL);

    cv::split(src,fusion);

    cv::medianBlur(fusion[2],mediam,3);

    cv::threshold(mediam,dst,0,255,cv::THRESH_OTSU);

    cv::findContours(dst, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );

    for(int u = 0;u < contours.size();u++)
    {
        int p = 0;
        for(int f = 0;f < contours2.size();f++)
        {
            match = cv::matchShapes(contours[u],contours2[f],cv::CONTOURS_MATCH_I1,0);
            if(match < 0.15)
            {
            p = 1;
            }
        }

        if(p != 0){
            contours3.push_back(contours[u]);
        }
    }

    for( size_t i = 0; i < contours3.size(); i++ ) {
        if(90 < cv::contourArea(contours3[i]))contours4.push_back(contours3[i]);
    }

    drawing = cv::Mat::zeros(dst.size(), CV_8UC3);

    for( size_t i = 0; i< contours4.size(); i++ ) {
        cv::Scalar color = cv::Scalar(255, 0, 0);
        cv::drawContours(drawing, contours4, (int)i, color);
    }

        cv::imshow("draw", drawing);
        if(cv::waitKey(1) >= 0) break;
  }
 }
}  // namespace match_components
