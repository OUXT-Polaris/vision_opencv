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
ObjectTracjer::ObjectTracjer(const TrackingMethod method)
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
  }(method))
{
}
}  // namespace opencv_components
