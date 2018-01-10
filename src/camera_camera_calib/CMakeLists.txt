cmake_minimum_required(VERSION 2.8.3)
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

#link_directories(../Thirdparty/apriltags/build/lib)
add_executable(camera_camera_calib test/testCalibrate.cpp 
                                   ${SOURCE_FILES}
                                   )
target_link_libraries(camera_camera_calib ${LINK_LIBRARIES})


add_executable(omni_reproj_test test/testOmniReprojection.cpp 
                                   ${SOURCE_FILES}
                                   )
target_link_libraries(omni_reproj_test ${LINK_LIBRARIES})
#target_link_libraries(omni_reproj_test apriltags)

add_executable(ocam_reproj_test test/testOcamReprojection.cpp 
                                   ${SOURCE_FILES}
                                   )
target_link_libraries(ocam_reproj_test ${LINK_LIBRARIES})


add_executable(testSyntheticData test/testSyntheticData.cpp 
                                   ${SOURCE_FILES}
                                   )
target_link_libraries(testSyntheticData ${LINK_LIBRARIES})


add_executable(testSynchronizer test/testSynchronizer.cpp 
                                   src/loadBag.cpp 
                                   )
target_link_libraries(testSynchronizer ${catkin_LIBRARIES})
target_link_libraries(testSynchronizer ${OpenCV_LIBRARIES})

add_executable(linearSolverComp test/linearSolverComp.cpp 
                                   ${SOURCE_FILES}
                                   )
target_link_libraries(linearSolverComp ${LINK_LIBRARIES})


