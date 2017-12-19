# sensor_calib

LIDAR-Camera extrinsic parameters calibration using Ceres optimization library. Our solution is suitable for the omnidirectional camera model with large field of view.

### First, the poses of a checkerboard are estimated using PnP algorithm.
Since we are using an omnidirectional camera to shoot video, we need a camera model that can reproject the object points (in checkerboard frame) onto the image points (in camera frame, suppose camera intrinsic parameters are known before calibration). The camera model here we use is decribed in the Kalibr camera calibration toolbox.

### Second, the LIDAR scan is transformed to a point cloud with 3D coordinates in LIDAR frame.
The LIDAR messages published is in the format described here: http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html.

### Finally, a nonlinear optimizaton problem is solved in Ceres.
The cost function of the optimization problem is the squared sum of distances between LIDAR scan points (in LIDAR frame) and checkerboard plane (expressed in LIDAR frame).

The parameters to be optimized are the transform between LIDAR and camera: translation and rotation vectors (totally six parameters).
