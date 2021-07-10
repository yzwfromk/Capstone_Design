cmake_minimum_required(VERSION 2.8.3)
project(data_integrate)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  sensor_msgs
  std_msgs
  core_msgs
)

find_package(OpenCV REQUIRED )
find_package( cv_bridge REQUIRED )

catkin_package(
  CATKIN_DEPENDS roscpp
  core_msgs
  image_transport
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)
add_executable(data_show_node src/data_showshowshow.cpp)
add_dependencies(data_show_node core_msgs_generate_messages_cpp)


add_executable(data_integration_node src/data_integration.cpp)
add_dependencies(data_integration_node core_msgs_generate_messages_cpp)

add_executable(path_planner src/path_planner.cpp)
add_dependencies(path_planner core_msgs_generate_messages_cpp)

target_link_libraries(data_show_node
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)

target_link_libraries(data_integration_node
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)
target_link_libraries(path_planner
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)

