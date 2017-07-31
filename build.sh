echo "Building AprilTags"
cd src/Thirdparty/apriltags
make

echo "Building slam_sensor_calib"
cd ../../../
catkin_make
