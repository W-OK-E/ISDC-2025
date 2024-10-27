cmake_minimum_required(VERSION 3.0.2)
project(mapping)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
 INCLUDE_DIRS 
#  LIBRARIES mars_gazebo
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Declare the executable
add_executable(depth src/mapping.cpp)

# Link the executable to required libraries
target_link_libraries(depth
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}

)