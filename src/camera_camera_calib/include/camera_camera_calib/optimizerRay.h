/**
 * LIDAR-Camera calibration.
 *
 * Muyuan Lin, 2017
 */

#ifndef OPTIMIZERRAY_H
#define OPTIMIZERRAY_H

#include <iostream>
#include <vector>
#include <math.h> 

#include <opencv2/opencv.hpp>

#include <Eigen/Dense>
#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include "camera_camera_calib/omniModel.h"
#include "camera_camera_calib/ocamCalibModel.h"



class optimizer
{
public:
    optimizer(){}
    void bundleAdjustment(OmniModel& cam0,
                          std::vector<cv::Point2f>& cam0_imgPts,
                          std::vector<cv::Point3f>& cam1_rays,
                          double* parameter);

    void bundleAdjustment(OCamCalibModel& ocamcalib_cam0,
                          std::vector<std::vector<cv::Point3f> >& cam0_rays,
                          std::vector<std::vector<cv::Point3f> >& cam1_rays,
                          double* parameter);
};

// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 6 parameters: 3 for rotation, 3 for translation,
struct SnavelyReprojectionError {
    SnavelyReprojectionError(OCamCalibModel ocamcalib_cam0, std::vector<cv::Point3f> cam0_ray,  std::vector<cv::Point3f> cam1_ray)
    : _ocamcalib_cam0(ocamcalib_cam0), _cam0_ray(cam0_ray), _cam1_ray(cam1_ray) {}
    
    template <typename T>
    bool operator()(const T* const transform,
                    T* residuals) const {
        // transform[0,1,2] are the angle-axis rotation.
        // transform[3,4,5] are the translation.
        
        // step 1: transform the object points in cam1 frame to that in cam0 frame
        Eigen::Matrix<T, 3, 1> point1, point2;
        point1 << T(0), T(0), T(0);
        point2 << transform[3], transform[4], transform[5];

        std::vector<Eigen::Matrix<T, 3, 1> > vec_rays1, vec_rays2;
        std::vector<Eigen::Matrix<T, 3, 1> > vec_xm, vec_xn;
        residuals[0] = T(0);
        for (size_t i=0; i<4; i++){
            Eigen::Matrix<T, 3, 1> ray1, ray2;
            ray1 << T(_cam0_ray[i].x), T(_cam0_ray[i].y), T(_cam0_ray[i].z);
            vec_rays1.push_back(ray1);

            T p[3], q[3];
            p[0] = T(_cam1_ray[i].x);
            p[1] = T(_cam1_ray[i].y);
            p[2] = T(_cam1_ray[i].z);
            ceres::AngleAxisRotatePoint(transform, p, q);
            ray2 << q[0], q[1], q[2];        
            vec_rays2.push_back(ray2);


            Eigen::Matrix<T, 3, 1> outTriangulatedPoint;
            T outGap;
            T outS0, outS1;
            Eigen::Matrix<T, 3, 1> xm, xn;
            _ocamcalib_cam0.triangulate(point1, ray1, point2, ray2,
                                         outTriangulatedPoint, outGap,
                                         outS0, outS1,
                                         xm, xn);

            vec_xm.push_back(xm);
            vec_xn.push_back(xn);

            residuals[0] += outGap;
        }

        T ratio0 = T(100), ratio1 = T(50);
        if (transform[5] < T(-0.21))
            residuals[1] = ratio0*(T(-0.21) - transform[5]);
        else if( transform[5] > -0.19)
            residuals[1] = ratio0*(transform[5] - T(-0.19));
        else
            residuals[1] = T(0);

        for (size_t i=0; i<4; i++){
            residuals[2+i] = T(0);
            residuals[8+i] = T(0);
            Eigen::Matrix<T, 3, 1> p1 = vec_xm[i%4];
            Eigen::Matrix<T, 3, 1> p2 = vec_xm[ (i+1)%4];
            T dist = euclideanDistance(p1, p2);
            residuals[2+i] = ratio1 * (dist - T(0.088)*T(0.088));

            Eigen::Matrix<T, 3, 1> p3 = vec_xn[i%4];
            Eigen::Matrix<T, 3, 1> p4 = vec_xn[ (i+1)%4];
            dist = euclideanDistance(p3, p4);
            residuals[8+i] = ratio1 * (dist - T(0.088)*T(0.088));
     
        }

        Eigen::Matrix<T, 3, 1> p1 = vec_xm[0];
        Eigen::Matrix<T, 3, 1> p2 = vec_xm[2];
        T dist = euclideanDistance(p1, p2);
        residuals[6] = ratio1 * (dist - T(0.1244)*T(0.1244));

        p1 = vec_xm[1];
        p2 = vec_xm[3];
        dist = euclideanDistance(p1, p2);
        residuals[7] = ratio1 * (dist - T(0.1244)*T(0.1244));

        p1 = vec_xn[0];
        p2 = vec_xn[2];
        dist = euclideanDistance(p1, p2);
        residuals[12] = ratio1 * (dist - T(0.1244)*T(0.1244));

        p1 = vec_xn[1];
        p2 = vec_xn[3];
        dist = euclideanDistance(p1, p2);
        residuals[13] = ratio1 * (dist - T(0.1244)*T(0.1244));

        return true;
    }
    
    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(OCamCalibModel ocamcalib_cam0, std::vector<cv::Point3f> cam0_ray, std::vector<cv::Point3f> cam1_ray) {
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 14, 6>(
                    new SnavelyReprojectionError(ocamcalib_cam0, cam0_ray, cam1_ray)));
    }

    template <typename T>
    T euclideanDistance(const Eigen::Matrix<T, 3, 1> p1, const Eigen::Matrix<T, 3, 1> p2) const{
        return (p1(0, 0) - p2(0, 0)) * (p1(0, 0) - p2(0, 0)) 
                    + (p1(1, 0) - p2(1, 0)) * (p1(1, 0) - p2(1, 0))
                    + (p1(2, 0) - p2(2, 0)) * (p1(2, 0) - p2(2, 0));
    }

    std::vector<cv::Point3f> _cam0_ray;    
    std::vector<cv::Point3f> _cam1_ray;
    OCamCalibModel _ocamcalib_cam0;
    OmniModel _cam0;
};

#endif
