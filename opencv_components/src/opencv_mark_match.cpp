
#include <opencv_components/opencv_mark_match.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace match_components
{
OpenCVMatchComponent::OpenCVMatchComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("opencv_mark_match", options),
  image_pub_(this, "image_")
{
  image_sub_ =   create_subscription<sensor_msgs::msg::Image>(
    "camera", 1, [this](const sensor_msgs::msg::Image::SharedPtr image) {
      call_back(image);
    }); 
}

OpenCVMatchComponent::~OpenCVMatchComponent() {}

void OpenCVMatchComponent::call_back(const sensor_msgs::msg::Image::SharedPtr image_msg){

  std::string triangle = ament_index_cpp::get_package_share_directory("opencv_components") + "/picture/red_triangle.jpg";
  std::string circle = ament_index_cpp::get_package_share_directory("opencv_components") + "/picture/circle.jpg";
  std::string cross = ament_index_cpp::get_package_share_directory("opencv_components") + "/picture/cross.jpg";

  cv::imread(triangle).copyTo(sample[0]);
  cv::imread(circle).copyTo(sample[1]);
  cv::imread(cross).copyTo(sample[2]);

  cv_bridge::CvImage bridge;
  sensor_msgs::msg::Image image = *image_msg.get();
  std::vector<std::vector<cv::Point> > sample_contours[3],contours,contours2,contours3;

  const cv::Mat image_cv = cv_bridge::toCvCopy(image_msg)->image;

  for(int i=0;i<3;i++){
    if(sample[i].empty())RCLCPP_INFO_STREAM(this->get_logger(), "no picture");
  }
  
  if(image_cv.empty())RCLCPP_INFO_STREAM(this->get_logger(),"no movie");

  for(int i=0;i<3;i++){
    cv::cvtColor(sample[i],sample_hsv[i],cv::COLOR_BGR2HSV_FULL);

    cv::split(sample_hsv[i],sample_split[i]);

    cv::dilate(sample_split[i][2], temp, cv::Mat(), cv::Point(-1,-1), 3);
    cv::erode(temp, temp, cv::Mat(), cv::Point(-1,-1), 3*2);
    cv::dilate(temp, temp, cv::Mat(), cv::Point(-1,-1), 3);

    cv::findContours(temp, sample_contours[i], hierarchy[i],  cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
  }
  
  cv::cvtColor(image_cv,img_hsv,cv::COLOR_BGR2HSV_FULL);

  cv::split(img_hsv,img_split);

  cv::medianBlur(img_split[2],mediam,3);

  cv::threshold(mediam,dst,0,255,cv::THRESH_OTSU);

  cv::findContours(dst, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  for(int i=0;i<3;i++){
    for(int u = 0;u < contours.size();u++)
    {
      int p = 0;
      for(int f = 0;f < sample_contours[i].size();f++)
      {
        match = cv::matchShapes(contours[u],sample_contours[i][f],cv::CONTOURS_MATCH_I1,0);
        if(1.5>match)//近似閾値
        {
          p = 1;
        }
      }

      if(p != 0){
        contours2.push_back(contours[u]);
      }
    }
  }

    for( size_t f = 0; f < contours2.size(); f++ ) {
      if(80 < cv::contourArea(contours2[f]))//最小面積閾値
      {
        if(18000 > cv::contourArea(contours2[f])){//最大面積閾値
          contours3.push_back(contours2[f]);
        }
      }
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
}