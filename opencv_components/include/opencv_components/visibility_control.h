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

#ifndef OPENCV_COMPONENTS__VISIBILITY_CONTROL_H_
#define OPENCV_COMPONENTS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define OPENCV_COMPONENTS_EXPORT __attribute__((dllexport))
#define OPENCV_COMPONENTS_IMPORT __attribute__((dllimport))
#else
#define OPENCV_COMPONENTS_EXPORT __declspec(dllexport)
#define OPENCV_COMPONENTS_IMPORT __declspec(dllimport)
#endif
#ifdef OPENCV_COMPONENTS_BUILDING_LIBRARY
#define OPENCV_COMPONENTS_PUBLIC OPENCV_COMPONENTS_EXPORT
#else
#define OPENCV_COMPONENTS_PUBLIC OPENCV_COMPONENTS_IMPORT
#endif
#define OPENCV_COMPONENTS_PUBLIC_TYPE OPENCV_COMPONENTS_PUBLIC
#define OPENCV_COMPONENTS_LOCAL
#else
#define OPENCV_COMPONENTS_EXPORT __attribute__((visibility("default")))
#define OPENCV_COMPONENTS_IMPORT
#if __GNUC__ >= 4
#define OPENCV_COMPONENTS_PUBLIC __attribute__((visibility("default")))
#define OPENCV_COMPONENTS_LOCAL __attribute__((visibility("hidden")))
#else
#define OPENCV_COMPONENTS_PUBLIC
#define OPENCV_COMPONENTS_LOCAL
#endif
#define OPENCV_COMPONENTS_PUBLIC_TYPE
#endif

#endif  // OPENCV_COMPONENTS__VISIBILITY_CONTROL_H_
