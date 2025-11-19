cmake_minimum_required(VERSION 3.5)
project(floor_removal_rgbd)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common segmentation filters)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Core library: plane_remover
add_library(plane_remover
  src/plane_remover.cpp
)

ament_target_dependencies(plane_remover
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
)

target_link_libraries(plane_remover
  ${PCL_LIBRARIES}
)

# Floor removal server node
add_executable(floor_removal_node
  src/server_node.cpp
)

ament_target_dependencies(floor_removal_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
)

target_link_libraries(floor_removal_node
  plane_remover
  ${PCL_LIBRARIES}
)

# Install executables
install(TARGETS
  floor_removal_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install libraries
install(TARGETS
  plane_remover
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY
  include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
