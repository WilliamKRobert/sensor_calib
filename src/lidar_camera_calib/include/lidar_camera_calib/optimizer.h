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
                          double* parameter,
                          cv::Mat init_rvec,
                          cv::Mat init_tvec
                                 );
private:
    double* initial_transform_guess;
};
    // Templated pinhole camera model for used with Ceres.  The camera is
    // parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
    // focal length and 2 for radial distortion. The principal point is not modeled
    // (i.e. it is assumed be located at the image center).


struct SnavelyReprojectionError {
    SnavelyReprojectionError(Eigen::Vector3f scan_pt, Eigen::Matrix4d object_pose)
    : _scan_pt(scan_pt), _object_pose(object_pose) {}
    
    template <typename T>
    bool operator()(const T* const transform,
                    T* residuals) const {
        // transform[0,1,2] are the angle-axis rotation.
        // transform[3,4,5] are the translation.
        
        // step 1: calculate point coordinates of each lidar scan in target frame
        T p[3], q[3];
        p[0] = T(_scan_pt(0));
        p[1] = T(_scan_pt(1));
        p[2] = T(_scan_pt(2));
        ceres::AngleAxisRotatePoint(transform, p, q);
        q[0] += T(transform[3]);
        q[1] += T(transform[4]);
        q[2] += T(transform[5]);

        Eigen::Matrix4d object_pose = _object_pose.inverse();

        T scan_in_checkerboard_frame[3];
        scan_in_checkerboard_frame[0] = T(object_pose(0,0))*q[0] + T(object_pose(0,1))*q[1] + T(object_pose(0,2))*q[2] + T(object_pose(0,3));
        scan_in_checkerboard_frame[1] = T(object_pose(1,0))*q[0] + T(object_pose(1,1))*q[1] + T(object_pose(1,2))*q[2] + T(object_pose(1,3));
        scan_in_checkerboard_frame[2] = T(object_pose(2,0))*q[0] + T(object_pose(2,1))*q[1] + T(object_pose(2,2))*q[2] + T(object_pose(2,3));

        // step 2: 

        // calculate the unit norm vector of the target plane
        // Eigen::Vector4d n0(0,0,1,1);
        // n0 = _object_pose * n0;
        // T n1[3], n2[3];
        // n1[0] = T(n0(0));
        // n1[1] = T(n0(1));
        // n1[2] = T(n0(2));
        // ceres::AngleAxisRotatePoint(transform, n1, n2);

        // T len = sqrt(n2[0]*n2[0] + n2[1]*n2[1] + n2[2]*n2[2]);
        // T nvec[3];
        // nvec[0] = T(n2[0]/len);
        // nvec[1] = T(n2[1]/len);
        // nvec[2] = T(n2[2]/len);

        // /*
        //  * find a vector from scan point to any point on the checkerboard plane 
        //  * here choose (0,0,0) (in checkerboard frame)
        //  */
        // // calculate the vector from left up corner to the scan point
        // T p[3];
        // p[0] = T(_scan_pt(0));
        // p[1] = T(_scan_pt(1));
        // p[2] = T(_scan_pt(2));
        

        // // coordinate of up left corner
        // T q[3], q_prime[3];
        // q[0] = T(_object_pose(0,3));
        // q[1] = T(_object_pose(1,3));
        // q[2] = T(_object_pose(2,3));
        // ceres::AngleAxisRotatePoint(transform, q, q_prime);
        // q_prime[0] += T(transform[3]);
        // q_prime[1] += T(transform[4]);
        // q_prime[2] += T(transform[5]);

        // q: 0, 0, 0
        T pq[3];
        pq[0] = scan_in_checkerboard_frame[0];
        pq[1] = scan_in_checkerboard_frame[1];
        pq[2] = scan_in_checkerboard_frame[2];

        T nvec[3];
        nvec[0] = T(0);
        nvec[1] = T(0);
        nvec[2] = T(1);
 
        // the residuals is the distance between scan point and the checkerboard plane
        residuals[0] = pq[0]*nvec[0] + pq[1]*nvec[1] + pq[2]*nvec[2];
        return true;
    }
    
    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(Eigen::Vector3f scan_pt,
                                       Eigen::Matrix4d object_pose) {
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 1, 6>(
                                                                                new SnavelyReprojectionError(scan_pt, object_pose)));
    }
    
    Eigen::Vector3f _scan_pt;
    Eigen::Matrix4d _object_pose;
};

#endif
