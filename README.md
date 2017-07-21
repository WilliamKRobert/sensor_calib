# lidar-camera-calib

LIDAR-Camera extrinsic parameters calibration using Ceres optimization library.

### First, the poses of a checkerboard is estimated using PnP algorithm.
Since we are using an omnidirectional camera to shoot video, we need a camera model that can undistort the image points.
The camera model here we use is decribed in the Kalibr camera calibration toolbox.

### Second, the lidar scan is transformed to a point cloud with 3D coordinates in LIDAR frame.

### Finally, a nonlinear optimizaton problem is solved in Ceres.
The cost function of the optimization problem is the squared sum of distances between LIDAR scan points (in LIDAR frame) and checkerboard plane (expressed in LIDAR frame).

The parameters to be optimized are the transform between LIDAR and camera: translation and rotation vectors (totally six parameters).
