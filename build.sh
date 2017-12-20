echo "Building AprilTags"
cd ./src/Thirdparty/apriltags
mkdir build
cd build
cmake ..
make
sudo make install

echo "Building slam_sensor_calib"
cd ../../../../
catkin_make
