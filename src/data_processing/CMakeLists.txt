cmake_minimum_required(VERSION 3.0.2)
project(data_processing)

# Use C++17 for std::filesystem and modern features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  pcl_ros
  pcl_conversions
  message_filters
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

###################################
## catkin package configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs cv_bridge pcl_ros pcl_conversions
)

###########
## Build ##
###########

# Include directories for C++ headers
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Build C++ executable
add_executable(data_saver_node src/data_saver_node.cpp)
add_executable(capture_subscriber_node src/capture_subscriber_node.cpp)
add_executable(capture_subscriber_econ_node src/capture_subscriber_econ_node.cpp)

# Link required libraries
target_link_libraries(data_saver_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(capture_subscriber_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(capture_subscriber_econ_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
#############
## Install ##
#############

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/subscriber.py
  scripts/no_sync_subscriber.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install C++ executable
install(TARGETS data_saver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
