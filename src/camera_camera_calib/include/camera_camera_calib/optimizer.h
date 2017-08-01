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
    void bundleAdjustment(vector<Eigen::Matrix4d>& tagPoses0,
                                 vector<Eigen::Matrix4d>& tagPoses1,
                                 double* parameter);
};

// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed be located at the image center).
struct SnavelyReprojectionError {
    SnavelyReprojectionError( Eigen::Matrix4d tagPose0,  Eigen::Matrix4d tagPose1)
    : _tagPose0(tagPose0), _tagPose1(tagPose1) {}
    
    template <typename T>
    bool operator()(const T* const transform,
                    T* residuals) const {
        // transform[0,1,2] are the angle-axis rotation.
        // transform[3,4,5] are the translation.
        
        // step 1: calculate point coordinates of each lidar scan in target frame
        // Eigen::Vector2d keypoint1(_keypoint1.pt.x, _keypoint1.pt.y);
        // Eigen::Vector3d outpoint;
        // _cam1.keypointToEuclidean(keypoint1, outpoint);

        // T p[3], q[3];
        // p[0] = T(outpoint(0));
        // p[1] = T(outpoint(1));
        // p[2] = T(outpoint(2));
        // ceres::AngleAxisRotatePoint(transform, p, q);
        // q[0] += T(transform[3]);
        // q[1] += T(transform[4]);
        // q[2] += T(transform[5]);

        // Eigen::Matrix<T, 3, 1> inputpoint(q[0], q[1], q[2]);
        // Eigen::Matrix<T, 2, 1> reproj_kp0;
        // _cam0.euclideanToKeypoint(inputpoint, reproj_kp0);
 
        // // the residuals is the distance between scan point and the checkerboard plane
        // residuals[0] = T(_keypoint0.pt.x) - reproj_kp0(0);
        // residuals[1] = T(_keypoint0.pt.y) - reproj_kp0(1);

        Eigen::Matrix3d R = tagPose1.block<3,3>(0,0);
        Eigen::Quaternionf r_pose(R.cast<float>());
        Eigen::Vector3f t_vec_pose(tagPose1(0,3), tagPose1(1,3), tagPose1(2,3));
        Eigen::Translation<float, 3> t_pose(t_vec_pose);

        Eigen::Transform<float,3, Eigen::Affine> combined = t_lidar_frame * r_lidar_frame * t_pose * r_pose * center_t;
        return true;
    }
    
    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(igen::Matrix4d tagPose0,  Eigen::Matrix4d tagPose1) {
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 6, 6>(
                    new SnavelyReprojectionError(tagPose0, tagPose1)));
    }
    
    Eigen::Matrix4d _tagPose0;
    Eigen::Matrix4d _tagPose1;
};

#endif
