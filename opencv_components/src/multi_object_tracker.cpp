// Copyright (c) 2023 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <opencv_components/multi_object_tracker.hpp>

namespace opencv_components
{
ObjectTracker::ObjectTracker(
  const TrackingMethod method, const cv::Mat & image,
  const perception_msgs::msg::Detection2D & detection, const rclcpp::Duration & lifetime)
: tracker_([](const auto method) -> cv::Ptr<cv::Tracker> {
    switch (method) {
      case TrackingMethod::CSRT:
        return cv::TrackerCSRT::create();
      case TrackingMethod::DA_SIAM_RPN:
        return cv::TrackerDaSiamRPN::create();
      case TrackingMethod::GOTURN:
        return cv::TrackerGOTURN::create();
      case TrackingMethod::KCF:
        return cv::TrackerKCF::create();
      case TrackingMethod::MIL:
        return cv::TrackerMIL::create();
      /*
      case TrackingMethod::NANO:
        return cv::TrackerNano::create();
      */
      default:
        return cv::TrackerDaSiamRPN::create();
    }
  }(method)),
  lifetime_(lifetime),
  initialize_timestamp_(detection.header.stamp),
  rect_(toCVRect(detection.bbox))
{
  tracker_->init(image, toCVRect(detection.bbox));
}

bool ObjectTracker::isExpired(const rclcpp::Time & stamp) const
{
  return (rclcpp::Time(stamp) - initialize_timestamp_) >= lifetime_;
}

std::optional<cv::Rect> ObjectTracker::update(const cv::Mat & image, const rclcpp::Time & stamp)
{
  const auto update_tracker = [this](const auto & image) {
    auto rect = cv::Rect();
    return tracker_->update(image, rect) ? [this](const auto & rect){rect_ = rect; return rect;}(rect) : std::optional<cv::Rect>();
  };
  return isExpired(stamp) ? std::optional<cv::Rect>() : update_tracker(image);
}

std::optional<cv::Rect> ObjectTracker::getRect() const { return rect_; }

MultiObjectTracker::MultiObjectTracker(
  const double iou_threashold, const rclcpp::Duration & lifetime)
: iou_threashold(iou_threashold), lifetime_(lifetime)
{
}

void MultiObjectTracker::update(
  const cv::Mat & image, const perception_msgs::msg::Detection2DArray & detections)
{
  /// @note Update tracker
  for (auto tracker : trackers_) {
    tracker.update(image, detections.header.stamp);
  }
  /// @note Remove expired/failed trackers
  for (auto tracker = trackers_.begin(); tracker != trackers_.end(); tracker++) {
    if (tracker->isExpired(detections.header.stamp) || !tracker->getRect()) {
      trackers_.erase(tracker);
    }
  }

  const auto rects = getRects();
  /// @note If rects are empty, it means no trackers exist.
  if (rects.empty()) {
    assert(trackers_.size() == 0);
    for (const auto & detection : detections.detections) {
      trackers_.emplace_back(
        ObjectTracker(TrackingMethod::DA_SIAM_RPN, image, detection, lifetime_));
    }
  } else {
    if (trackers_.size() >= detections.detections.size()) {
      /// @note In this case, n = trackers_.size(), m = detections.detections.size()
      /// @sa https://kopricky.github.io/code/NetworkFlow/hungarian.html
      const auto construct_cost_matrix =
        [this](const auto & detections) -> std::vector<std::vector<double>> {
        std::vector<std::vector<double>> matrix;
        for (const auto & tracker : trackers_) {
          std::vector<double> row;
          for (const auto & detection : detections.detections) {
            assert(tracker.getRect());
            row.push_back(1 - getIoU(tracker.getRect().value(), toCVRect(detection.bbox)));
          }
          matrix.emplace_back(row);
        }
        return matrix;
      };
      const auto solution = Hungarian(construct_cost_matrix(detections)).solve();
      /// @note assing tracker
      size_t detection_index = 0;
      for (const auto tracker_index : solution.second) {
        /// @note Detection are exists, but no tracker existing.
        if (tracker_index == -1) {
          trackers_.emplace_back(ObjectTracker(
            TrackingMethod::DA_SIAM_RPN, image, detections.detections[detection_index], lifetime_));
        }
        /// @note Hungarian solver found mathing.
        else {
          auto tracker_itr = trackers_.begin();
          std::advance(tracker_itr, tracker_index);
          /// @note Detection and tracker are not associated, so create new tracker.
          if (
            getIoU(
              tracker_itr->getRect().value(),
              toCVRect(detections.detections[detection_index].bbox)) <= iou_threashold) {
            trackers_.emplace_back(ObjectTracker(
              TrackingMethod::DA_SIAM_RPN, image, detections.detections[detection_index],
              lifetime_));
          }
          /// @note Detection and tracker are associated, so remove old tracker and create new tracker.
          else {
            trackers_.erase(tracker_itr);
            trackers_.emplace_back(ObjectTracker(
              TrackingMethod::DA_SIAM_RPN, image, detections.detections[detection_index],
              lifetime_));
          }
        }
        detection_index++;
      }
    } else {
      /// @note In this case, n = detections.detections.size(), m = trackers_.size()
      /// @sa https://kopricky.github.io/code/NetworkFlow/hungarian.html
      const auto construct_cost_matrix =
        [this](const auto & detections) -> std::vector<std::vector<double>> {
        std::vector<std::vector<double>> matrix;
        for (const auto & detection : detections.detections) {
          std::vector<double> row;
          for (const auto & tracker : trackers_) {
            assert(tracker.getRect());
            row.push_back(1 - getIoU(tracker.getRect().value(), toCVRect(detection.bbox)));
          }
          matrix.emplace_back(row);
        }
        return matrix;
      };
      const auto solution = Hungarian(construct_cost_matrix(detections)).solve();
      /// @note assing tracker
      size_t tracker_index = 0;
      for (const auto detection_index : solution.second) {
        /// @note New tracking target detected.
        if (detection_index == -1) {
          trackers_.emplace_back(ObjectTracker(
            TrackingMethod::DA_SIAM_RPN, image, detections.detections[detection_index], lifetime_));
        }
        /// @note Hungarian solver found mathing.
        else {
          auto tracker_itr = trackers_.begin();
          std::advance(tracker_itr, tracker_index);
          /// @note Detection and tracker are not associated, so create new tracker.
          if (
            getIoU(
              tracker_itr->getRect().value(),
              toCVRect(detections.detections[detection_index].bbox)) <= iou_threashold) {
            trackers_.emplace_back(ObjectTracker(
              TrackingMethod::DA_SIAM_RPN, image, detections.detections[detection_index],
              lifetime_));
          }
          /// @note Detection and tracker are associated, so remove old tracker and create new tracker.
          else {
            trackers_.erase(tracker_itr);
            trackers_.emplace_back(ObjectTracker(
              TrackingMethod::DA_SIAM_RPN, image, detections.detections[detection_index],
              lifetime_));
          }
        }
        tracker_index++;
      }
    }
  }
}

std::vector<cv::Rect> MultiObjectTracker::getRects() const
{
  std::vector<cv::Rect> rects;
  for (const auto & tracker : trackers_) {
    if (const auto rect = tracker.getRect()) {
      rects.emplace_back(rect.value());
    }
  }
  return rects;
}
}  // namespace opencv_components
