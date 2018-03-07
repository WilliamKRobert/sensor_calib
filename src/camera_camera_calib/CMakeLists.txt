cmake_minimum_required(VERSION 3.0.0)
project(camera_camera_calib)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

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
)

find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED )
find_package( Ceres REQUIRED )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs
  # DEPENDS system_lib
)
 
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  test
  ../Thirdparty/apriltags
  src/Thirdparty/apriltags/build/lib
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

# catkin_add_nosetests(test)
set(LINK_LIBRARIES
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${CERES_LIBRARIES}
  apriltags
)

file(GLOB_RECURSE SOURCE_FILES "src/*.cpp")



add_executable(stereoCameraExample test/stereoCameraExample.cpp ${SOURCE_FILES})
target_link_libraries(stereoCameraExample ${LINK_LIBRARIES})

add_executable(stereoCameraSimulation test/stereoCameraSimulation.cpp ${SOURCE_FILES})
target_link_libraries(stereoCameraSimulation ${LINK_LIBRARIES})


add_executable(singleCameraSimulation test/singleCameraSimulation.cpp ${SOURCE_FILES})
target_link_libraries(singleCameraSimulation ${LINK_LIBRARIES})


add_executable(singleCameraExample test/singleCameraExample.cpp 
                                   ${SOURCE_FILES})
target_link_libraries(singleCameraExample ${LINK_LIBRARIES})
#[[



add_executable(omniReprojectionTest test/omniReprojectionTest.cpp 
                                   ${SOURCE_FILES}
                                   )
target_link_libraries(omniReprojectionTest ${LINK_LIBRARIES})

add_executable(ocamReprojectionTest test/ocamReprojectionTest.cpp 
                                   ${SOURCE_FILES}
                                   )
target_link_libraries(ocamReprojectionTest ${LINK_LIBRARIES})
]]
#[[
add_executable(synchronizerTest test/synchronizerTest.cpp 
                                   src/loadBag.cpp 
                                   )
target_link_libraries(synchronizerTest ${catkin_LIBRARIES})
target_link_libraries(synchronizerTest ${OpenCV_LIBRARIES})

add_executable(linearSolverTest test/linearSolverTest.cpp 
                                   ${SOURCE_FILES}
                                   )
target_link_libraries(linearSolverTest ${LINK_LIBRARIES})


add_executable(stereoCameraSyn test/syntheticDataSimulation_backup.cpp 
                                   ${SOURCE_FILES}
                                   )
target_link_libraries(stereoCameraSyn ${LINK_LIBRARIES})
]]
