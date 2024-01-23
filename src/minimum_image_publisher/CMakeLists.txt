cmake_minimum_required(VERSION 3.0.2)
project(minimum_image_publisher)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(_opencv_version 4)
find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
  find_package(OpenCV 3 REQUIRED)
  set(_opencv_version 3)
endif()

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  depthai_bridge
  depthai_ros_msgs
  message_filters
  nodelet
  roscpp
  sensor_msgs
  std_msgs
  vision_msgs
  nav_msgs
  image_transport
)
find_package(depthai CONFIG REQUIRED)

catkin_package(
  LIBRARIES minimum_image_publisher
  CATKIN_DEPENDS
    camera_info_manager
    cv_bridge
    depthai_bridge
    depthai_ros_msgs
    message_filters
    nodelet
    roscpp
    sensor_msgs
    std_msgs
    vision_msgs
    nav_msgs
    image_transport
  DEPENDS
  OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(image_publisher_node src/image_publisher.cpp)
add_executable(image_publisher_ros_node src/image_publisher_ros.cpp)
add_executable(image_publisher_ros_single_detection_node src/image_publisher_ros_single_detection.cpp)
target_link_libraries(image_publisher_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  depthai::core
  yaml-cpp
)
target_link_libraries(image_publisher_ros_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  depthai::core
  yaml-cpp
)
target_link_libraries(image_publisher_ros_single_detection_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  depthai::core
  yaml-cpp
)