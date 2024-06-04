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

generate_parameter_library(
  opencv_match_parameters
  parameters/opencv_match_parameters.yaml
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/opencv_camera_component.cpp
  src/tracking_component.cpp
  src/multi_object_tracker.cpp
  src/util.cpp
  src/opencv_mark_match.cpp
)
target_link_libraries(${PROJECT_NAME}
  opencv_core
  opencv_camera_parameters
  opencv_match_parameters
)
rclcpp_components_register_nodes(${PROJECT_NAME} "opencv_components::OpenCVMatchComponent")
ament_export_include_directories(include/opencv_match_parameters)

ament_auto_add_executable(opencv_camera_node
  src/opencv_camera_node.cpp
)
target_link_libraries(opencv_camera_node
  ${PROJECT_NAME}
)

ament_auto_add_executable(opencv_mark_match
  src/opencv_mark_node.cpp
)
target_link_libraries(opencv_mark_match
  ${PROJECT_NAME}
)

install(DIRECTORY picture DESTINATION share/${PROJECT_NAME})

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
