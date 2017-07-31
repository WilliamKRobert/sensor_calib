/**
 * LIDAR-Camera calibration.
 *
 * Muyuan Lin, 2017
 */

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <iostream>
#include <vector>
#include <math.h> 

#include <opencv2/opencv.hpp>

#include <Eigen/Dense>
#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include "camera_camera_calib/omniModel.h"



class optimizer
{
public:
    optimizer(){}
    void bundleAdjustment(OmniModel &cam0, OmniModel &cam1,
                         const std::vector<std::vector<cv::KeyPoint> > kps_vec_0,
                         const std::vector<std::vector<cv::KeyPoint> > kps_vec_1,
                         double* parameter
                         );
};

// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed be located at the image center).
struct SnavelyReprojectionError {
    SnavelyReprojectionError( OmniModel cam0,  OmniModel cam1,
                             cv::KeyPoint kp0, cv::KeyPoint kp1)
    : _cam0(cam0), _cam1(cam1), 
    _keypoint0(kp0), _keypoint1(kp1) {}
    
    template <typename T>
    bool operator()(const T* const transform,
                    T* residuals) const {
        // transform[0,1,2] are the angle-axis rotation.
        // transform[3,4,5] are the translation.
        
        // step 1: calculate point coordinates of each lidar scan in target frame
        Eigen::Vector2d keypoint1(_keypoint1.pt.x, _keypoint1.pt.y);
        Eigen::Vector3d outpoint;
        _cam1.keypointToEuclidean(keypoint1, outpoint);

        T p[3], q[3];
        p[0] = T(outpoint(0));
        p[1] = T(outpoint(1));
        p[2] = T(outpoint(2));
        ceres::AngleAxisRotatePoint(transform, p, q);
        q[0] += T(transform[3]);
        q[1] += T(transform[4]);
        q[2] += T(transform[5]);

        Eigen::Matrix<T, 3, 1> inputpoint(q[0], q[1], q[2]);
        Eigen::Matrix<T, 2, 1> reproj_kp0;
        _cam0.euclideanToKeypoint(inputpoint, reproj_kp0);
 
        // the residuals is the distance between scan point and the checkerboard plane
        residuals[0] = T(_keypoint0.pt.x) - reproj_kp0(0);
        residuals[1] = T(_keypoint0.pt.y) - reproj_kp0(1);
        return true;
    }
    
    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(OmniModel cam0, OmniModel cam1, 
                                       cv::KeyPoint keypoint0, cv::KeyPoint keypoint1) {
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 6>(
                    new SnavelyReprojectionError(cam0, cam1,
                                                 keypoint0, keypoint1)));
    }
    
    OmniModel _cam0, _cam1;
    cv::KeyPoint _keypoint0;
    cv::KeyPoint _keypoint1;
};

#endif
