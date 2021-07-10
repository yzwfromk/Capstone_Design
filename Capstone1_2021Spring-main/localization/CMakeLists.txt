cmake_minimum_required(VERSION 2.8.3)
project(localization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  core_msgs
  pcl_conversions
  pcl_ros
)

find_package(OpenCV REQUIRED )
catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
#  sensor_msgs
#  std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)
add_executable(localization_node src/localization.cpp)
add_dependencies(localization_node core_msgs_generate_messages_cpp)

target_link_libraries(localization_node
  ${catkin_LIBRARIES} ${OpenCV_LIBS}
)
