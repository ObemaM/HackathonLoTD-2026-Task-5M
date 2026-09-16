cmake_minimum_required(VERSION 3.8)
project(metro_obstacle_detector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree search segmentation)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(detector_core
  src/axis_mapping.cpp
)
target_include_directories(detector_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(detector_core Eigen3::Eigen)

add_executable(detector_node
  src/detector_node.cpp
)
target_include_directories(detector_node PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(detector_node PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(detector_node
  detector_core
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)
ament_target_dependencies(detector_node
  geometry_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2_ros
  visualization_msgs
)

install(TARGETS detector_core detector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_axis_mapping test/test_axis_mapping.cpp)
  target_link_libraries(test_axis_mapping detector_core Eigen3::Eigen)
  target_include_directories(test_axis_mapping PRIVATE include)
endif()

ament_package()

