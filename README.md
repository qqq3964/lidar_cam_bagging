# Transform Bag Files into Images and Point Clouds
This script converts bag files into synchronized camera images and LiDAR point clouds using the Econ system camera and Robosense LiDAR.
## Dependencies
The following libraries are required to build and run this project:

- ROS (Robot Operating System)
(Tested with ROS Noetic)

- catkin (build system for ROS packages)

- OpenCV (tested with OpenCV 4.x)

- PCL (Point Cloud Library, tested with PCL 1.10 or higher)

Make sure the following ROS packages are installed:

- roscpp

- rospy

- std_msgs

- sensor_msgs

- cv_bridge

- pcl_ros

- pcl_conversions

- message_filters

Install system dependencies if not already installed:
```bash
sudo apt-get install ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-cv-bridge ros-noetic-message-filters
sudo apt-get install libpcl-dev libopencv-dev
```
## 1. Build and Setup
```bash
catkin_make
source devel/setup.bash
```
## 2. Run the Launch File
```bash
roslaunch src/data_processing/launch/sync_capture_econ_save.launch 
```
## 3. Usage Instructions
- Press `s` to save the current synchronized image and point cloud to the designated folders.
- Press `q` to interrupt and exit the program.

