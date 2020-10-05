cmake_minimum_required(VERSION 2.8.3)
project(ros_mono_vo)

include_directories("/usr/include/eigen3")


## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
        nav_msgs
)
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)

catkin_package(
        INCLUDE_DIRS include
DEPENDS OpenCV
        LIBRARIES ${PROJECT_NAME}
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
include/util.h
include/util.cpp
${OpenCV_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
)

add_executable(mono_vo_node src/mono_vo.cpp)
target_link_libraries(mono_vo_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Pangolin_LIBRARIES})


