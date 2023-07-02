#include <opencv_components/tracking_component.hpp>

namespace opencv_components
{

TrackingComponent::TrackingComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("tracking_node", options), tracker_(cv::TrackerKCF::create())
{
}

TrackingComponent::~TrackingComponent() {}

}  // namespace opencv_components
