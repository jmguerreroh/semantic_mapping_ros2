cmake_minimum_required(VERSION 3.8)
project(semantic_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.8 REQUIRED QUIET)
find_package(pcl_conversions REQUIRED)
find_package(image_geometry REQUIRED)
find_package(depth_image_proc REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yolov8_msgs REQUIRED)
find_package(semantic_mapping_interfaces REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(dependencies
  rclcpp
  image_transport
  cv_bridge
  sensor_msgs
  std_msgs
  OpenCV
  PCL
  pcl_conversions
  image_geometry
  depth_image_proc
  Eigen3
  message_filters
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
  yolov8_msgs
  semantic_mapping_interfaces
)

# OpenCV and PCL node
include_directories(include)
add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/SemanticMapping.cpp
  src/${PROJECT_NAME}/DetectorParser.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(parser src/main_detector_parser.cpp)
ament_target_dependencies(parser ${dependencies})
target_link_libraries(parser ${PROJECT_NAME})

add_executable(mapper src/main_semantic_mapping.cpp)
ament_target_dependencies(mapper ${dependencies})
target_link_libraries(mapper ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  parser
  mapper
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
