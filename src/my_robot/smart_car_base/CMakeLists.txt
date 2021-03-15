cmake_minimum_required(VERSION 2.8.3)
project(smart_car_base)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  rosserial_python
  sensor_msgs
  std_msgs
  tf
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES smart_car_base
  CATKIN_DEPENDS roscpp rospy tf nav_msgs geometry_msgs sensor_msgs std_msgs 
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(smart_car_base src/smart_car_base.cpp)
add_executable(smart_car_base_node src/smart_car_base_node.cpp)
target_link_libraries(smart_car_base_node smart_car_base ${catkin_LIBRARIES})

