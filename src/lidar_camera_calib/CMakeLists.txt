cmake_minimum_required(VERSION 2.8.3)
project(lidar_camera_calib)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  catkin REQUIRED cmake_modules
  roscpp
  rospy
  std_msgs
  tf
  rosbag
  cv_bridge
  laser_geometry
)

find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED )
find_package( Ceres REQUIRED )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES find_camera_pose
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS  EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

set(LINK_LIBRARIES
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${CERES_LIBRARIES}
)

add_executable(lidar_camera_calib src/findTransform.cpp 
                                  src/loadBag.cpp 
                                  src/objectPose.cpp 
                                  src/omniModel.cpp
                                  src/optimizer.cpp)
target_link_libraries(lidar_camera_calib ${LINK_LIBRARIES})

add_executable(calib_visualization src/visualization.cpp 
                                  src/loadBag.cpp 
                                  src/objectPose.cpp 
                                  src/omniModel.cpp
                                  src/optimizer.cpp)
target_link_libraries(calib_visualization ${LINK_LIBRARIES})

