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

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv_components/hungarian.hpp>
#include <perception_msgs/msg/detection2_d.hpp>

namespace opencv_components
{
/// @sa https://docs.opencv.org/4.x/d0/d0a/classcv_1_1Tracker.html
enum class TrackingMethod { CSRT, DA_SIAM_RPN, GOTURN, KCF, MIL, /*NANO*/ };

class ObjectTracjer
{
public:
  explicit ObjectTracjer(const TrackingMethod method);

private:
  cv::Ptr<cv::Tracker> tracker_;
};

class MultiObjectTracker
{
private:
};
}  // namespace opencv_components
