cmake_minimum_required(VERSION 3.8)
project(opencv_components)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(cv_bridge REQUIRED)

include_directories(include
  ${opencv_vendor_INCLUDE_DIRS}/opencv4
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/opencv_camera_component.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  cv_bridge
)

target_link_libraries(${PROJECT_NAME}
  opencv_core
)

ament_auto_add_executable(opencv_camera_node
  src/opencv_camera_node.cpp
)
target_link_libraries(opencv_camera_node
  ${PROJECT_NAME}
)

# launch/conifg file
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
