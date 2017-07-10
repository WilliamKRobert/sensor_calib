/**
 * LIDAR-Camera calibration.
 *
 * Muyuan Lin, 2017
 */

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

#include <Eigen/Dense>
#include "ceres/ceres.h"
#include "ceres/rotation.h"

class optimizer
{
public:
    optimizer(){}
    void bundleAdjustment(const std::vector<std::vector<cv::Point3f> > lidar_scan,
                          const std::vector<Eigen::Matrix4d> object_poses,
                          const cv::Size patternsize,
                          const double square_size,
                          const double cube_depth,
                          double* parameter
                                 );
private:
    double* initial_transform_guess;
};
    // Templated pinhole camera model for used with Ceres.  The camera is
    // parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
    // focal length and 2 for radial distortion. The principal point is not modeled
    // (i.e. it is assumed be located at the image center).


struct SnavelyReprojectionError {
    SnavelyReprojectionError(Eigen::Vector3d scan_pt, Eigen::Matrix4d object_pose)
    : _scan_pt(scan_pt), _object_pose(object_pose) {}
    
    template <typename T>
    bool operator()(const T* const transform,
                    T* residuals) const {
        // transform[0,1,2] are the angle-axis rotation.
        // transform[3,4,5] are the translation.
        
        // calculate the unit norm vector of the target plane
        Eigen::Vector4d norm_vec_homo(0,0,1,1);
        norm_vec_homo = _object_pose * norm_vec_homo;
        Eigen::Vector3d norm_vec(norm_vec_homo(0), norm_vec_homo(1), norm_vec_homo(2));
        norm_vec.normalize();
        T nvec[3];
        nvec[0] = T(norm_vec(0));
        nvec[1] = T(norm_vec(1));
        nvec[2] = T(norm_vec(2));

        // calculate the vector from left up corner to the scan point
        T point[3];
        T p[3];
        point[0] = T(_scan_pt(0));
        point[1] = T(_scan_pt(1));
        point[2] = T(_scan_pt(2));
        ceres::AngleAxisRotatePoint(transform, point, p);
        p[0] += transform[3];
        p[1] += transform[4];
        p[2] += transform[5];
        T q[3];
        q[0] = T(_object_pose(0,3));
        q[1] = T(_object_pose(1,3));
        q[2] = T(_object_pose(2,3));
        T pq[3];
        pq[0] = p[0]-q[0];
        pq[1] = p[1]-q[1];
        pq[2] = p[2]-q[2];

        // the residuals is the distance between scan point and the checkerboard plane
        residuals[0] = pq[0]*nvec[0] + pq[1]*nvec[1] + pq[2]*nvec[2];
        return true;
    }
    
    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(Eigen::Vector3d scan_pt,
                                       Eigen::Matrix4d object_pose) {
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 1, 6>(
                                                                                new SnavelyReprojectionError(scan_pt, object_pose)));
    }
    
    Eigen::Vector3d _scan_pt;
    Eigen::Matrix4d _object_pose;
};

#endif
