# add library
add_library(${PROJECT_NAME} cv_bridge.cpp rgb_colors.cpp)
include(GenerateExportHeader)
generate_export_header(${PROJECT_NAME} EXPORT_FILE_NAME ${PROJECT_NAME}/${PROJECT_NAME}_export.h)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${sensor_msgs_TARGETS}
  opencv_core
  opencv_imgproc
  opencv_imgcodecs)
target_link_libraries(${PROJECT_NAME} PRIVATE
  Boost::headers
  rcpputils::rcpputils)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/${PROJECT_NAME}_export.h
  DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME})
  