cmake_minimum_required(VERSION 2.8.3)
project(robomuse_description)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg urdf tf nav_msgs message_generation
             pluginlib
             cmake_modules
             geometric_shapes
	     sensor_msgs
             )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    interactive_markers
)
find_package(Eigen REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
)