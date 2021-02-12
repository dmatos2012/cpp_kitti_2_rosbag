cmake_minimum_required(VERSION 2.8.3)
project(cpp_kitti_2_bag)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  pcl_conversions
  genmsg
)
find_package(Eigen3 REQUIRED)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cpp_kitti_2_bag
#  CATKIN_DEPENDS cv_bridge image_transport roscpp std_msgs pcl_ros
#  DEPENDS system_lib
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(kitti_convert_bag
  src/KittiParser.cpp
  src/KittiTransforms.cpp
)

target_link_libraries(kitti_convert_bag stdc++fs  ${catkin_LIBRARIES})

add_executable(main src/main.cpp)
target_link_libraries(main kitti_convert_bag stdc++fs ${catkin_LIBRARIES})





