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
