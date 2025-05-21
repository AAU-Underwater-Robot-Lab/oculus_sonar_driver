cmake_minimum_required(VERSION 3.8)
project(oculus_sonar_driver)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-std=c++17)

# Find ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(marine_acoustic_msgs REQUIRED)
find_package(g3log_ros REQUIRED)
find_package(liboculus REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(g3log REQUIRED)

# Ensure CMake can find g3log_rosConfig.cmake
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}/g3log_ros")

# Include headers
include_directories(
  include
  ../liboculus/include
  ../liboculus/thirdparty
  ${CMAKE_INSTALL_PREFIX}/include/g3log
  ${CMAKE_INSTALL_PREFIX}/include
)
include_directories(${CMAKE_SOURCE_DIR}/liboculus)

# Add g3log_ros library directory for linker to find g3log
link_directories(${CMAKE_INSTALL_PREFIX}/g3log_ros/lib)

# Build the ROS2 node executable only
add_executable(oculus_driver_node
  src/oculus_driver_node.cpp
)

target_link_libraries(oculus_driver_node
  liboculus::oculus
  g3log
)

ament_target_dependencies(oculus_driver_node
  rclcpp
  sensor_msgs
  std_msgs
  marine_acoustic_msgs
  g3log_ros
  liboculus
)

# Install the node executable
install(TARGETS oculus_driver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch and config files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install headers, but exclude ping_to_simple_ping_result_msg.h
install(DIRECTORY include/
  DESTINATION include/
)

# Export package
ament_package()
