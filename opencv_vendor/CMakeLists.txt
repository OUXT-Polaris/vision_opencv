cmake_minimum_required(VERSION 3.8)
project(opencv_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

macro(build_opencv)
  include(ExternalProject)
  externalproject_add(opencv-ext
    DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/opencv_build
    DOWNLOAD_COMMAND curl -k -L https://github.com/opencv/opencv/archive/refs/tags/4.8.0.tar.gz -o opencv.tar.gz &&
      tar -xzvf opencv.tar.gz &&
      curl -k -L https://github.com/opencv/opencv_contrib/archive/refs/tags/4.8.0.tar.gz -o opencv-contrib.tar.gz &&
      tar -xzvf opencv-contrib.tar.gz
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/opencv_install -DOPENCV_EXTRA_MODULES_PATH=${CMAKE_CURRENT_BINARY_DIR}/opencv_build/opencv_contrib-4.8.0/modules
    SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/opencv_build/opencv-4.8.0
  )

  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/opencv_install
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}
  )
endmacro()

build_opencv()
ament_export_include_directories(opencv_install/include)

ament_export_libraries(
  opencv_calib3d
  opencv_core
  opencv_dnn
  opencv_features2d
  opencv_flann
  # opencv_gapi
  opencv_highgui
  opencv_imgcodecs
  opencv_imgproc
  opencv_ml
  opencv_objdetect
  opencv_photo
  opencv_stitching
  opencv_videoio
  opencv_video
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
