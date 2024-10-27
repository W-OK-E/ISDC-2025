cmake_minimum_required(VERSION 3.0.2)
project(mapping)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)



# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Declare the executable
add_executable(depth2 src/mapping.cpp)

# Link the executable to required libraries
target_link_libraries(depth2
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)