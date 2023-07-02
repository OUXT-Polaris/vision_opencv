cmake_minimum_required(VERSION 3.8)
project(opencv_components)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include
  ${opencv_vendor_INCLUDE_DIRS}/opencv4
)

generate_parameter_library(
  opencv_camera_parameters
  parameters/opencv_camera_parameters.yaml
)

ament_auto_add_library(cv_bridge SHARED
  src/cv_bridge/cv_bridge.cpp
  src/cv_bridge/cv_mat_sensor_msgs_image_type_adapter.cpp
  src/cv_bridge/rgb_colors.cpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/opencv_camera_component.cpp
  src/tracking_component.cpp
  src/multi_object_tracker.cpp
)
target_link_libraries(${PROJECT_NAME}
  opencv_core
  cv_bridge
  opencv_camera_parameters
)

ament_auto_add_executable(opencv_camera_node
  src/opencv_camera_node.cpp
)
target_link_libraries(opencv_camera_node
  ${PROJECT_NAME}
)

ament_auto_add_executable(tracking_node
  src/tracking_node.cpp
)
target_link_libraries(tracking_node
  ${PROJECT_NAME}
)

# # launch/conifg file
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

ament_auto_package()
