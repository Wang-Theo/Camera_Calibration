cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_CXX_STANDARD 17)
project(cam_cali)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  tf
)

# set(OpenCV_DIR "~/opencv-3.4.3/build")   
# find_package( OpenCV 3.4.3 REQUIRED)
find_package( OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
)

# add library
add_library(cam_cali
    src/cam_node.cpp
    src/line_extract.cpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_subdirectory(test)


