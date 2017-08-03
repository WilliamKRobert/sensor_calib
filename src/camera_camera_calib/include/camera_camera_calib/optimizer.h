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
    void bundleAdjustment(OmniModel& cam0,
                          std::vector<cv::Point2f>& cam0_imgPts,
                          std::vector<cv::Point3f>& cam1_objPts,
                          double* parameter);
};

// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 6 parameters: 3 for rotation, 3 for translation,
struct SnavelyReprojectionError {
    SnavelyReprojectionError( OmniModel cam0, cv::Point2f cam0_imgPts,  cv::Point3f cam1_objPts)
    : _cam0(cam0), _cam0_imgPts(cam0_imgPts), _cam1_objPts(cam1_objPts) {}
    
    template <typename T>
    bool operator()(const T* const transform,
                    T* residuals) const {
        // transform[0,1,2] are the angle-axis rotation.
        // transform[3,4,5] are the translation.
        
        // step 1: transform the object points in cam1 frame to that in cam0 frame
        // std::cout << "Debug info:" << std::endl;
        // std::cout <<"cam1 obj pts:" << std::endl;
        // std::cout << _cam1_objPts.x << " "  << _cam1_objPts.y << " " << _cam1_objPts.z << std::endl;
        if (isnan(_cam1_objPts.x) || isnan(_cam1_objPts.y) || isnan(_cam1_objPts.z) 
                  || isnan(_cam0_imgPts.x) || isnan(_cam0_imgPts.y)){
            std::cout << "invalid value!" << std::endl;
            return false;
        }
        T p[3], q[3];
        p[0] = T(_cam1_objPts.x);
        p[1] = T(_cam1_objPts.y);
        p[2] = T(_cam1_objPts.z);
        ceres::AngleAxisRotatePoint(transform, p, q);
        q[0] += T(transform[3]);
        q[1] += T(transform[4]);
        q[2] += T(transform[5]);
        // std::cout <<"q:" << std::endl;
        // std::cout << q[0] << " "  << q[1] << " " << q[2] << std::endl;
        // reproject object points to image points in cam0 frame
        Eigen::Matrix<T, 3, 1> eigen_objPt;
        Eigen::Matrix<T, 2, 1> outKeypoint;
        eigen_objPt << q[0], q[1], q[2];

        // std::cout<< eigen_objPt(0) << " " << eigen_objPt(1) << " " << eigen_objPt(2) << std::endl;
        bool isValid = _cam0.euclideanToKeypoint(eigen_objPt, outKeypoint);
        
        // // the residuals is the distance between scan point and the checkerboard plane
        residuals[0] = T(_cam0_imgPts.x) - outKeypoint(0);
        
        // std::cout << _cam0_imgPts.x << " " << outKeypoint(0) << std::endl;
        residuals[1] = T(_cam0_imgPts.y) - outKeypoint(1);
        // std::cout << _cam0_imgPts.y << " " << outKeypoint(1) << std::endl;


        return true;
    }
    
    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(OmniModel cam0, cv::Point2f cam0_imgPts, cv::Point3f cam1_objPts) {
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 6>(
                    new SnavelyReprojectionError(cam0, cam0_imgPts, cam1_objPts)));
    }

    cv::Point2f _cam0_imgPts;    
    cv::Point3f _cam1_objPts;
    OmniModel _cam0;
};

#endif
