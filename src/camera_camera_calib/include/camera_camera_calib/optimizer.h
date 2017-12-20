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
#include "camera_camera_calib/ocamCalibModel.h"



class optimizer
{
public:
    optimizer(){}
    void bundleAdjustment(OmniModel& cam0,
                          std::vector<cv::Point2f>& cam0_imgPts,
                          std::vector<cv::Point3f>& cam1_objPts,
                          double* parameter);

    void bundleAdjustment(OCamCalibModel& ocamcalib_cam0,
                                 std::vector<std::vector<cv::Point2f> >& cam0_imgPts,
                                 std::vector<std::vector<cv::Point3f> >& cam0_objPts,
                                 double* poses0,
                                 OCamCalibModel& ocamcalib_cam1,
                                 std::vector<std::vector<cv::Point2f> >& cam1_imgPts,
                                 std::vector<std::vector<cv::Point3f> >& cam1_objPts,
                                 double* poses1,
                                 double* parameter);
};

// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 6 parameters: 3 for rotation, 3 for translation,
struct ReprojectionError1 {
    ReprojectionError1(OCamCalibModel ocamcalib_cam0, cv::Point2f cam0_imgPts, cv::Point3f cam1_objPts)
    : _ocamcalib_cam0(ocamcalib_cam0), _cam0_imgPts(cam0_imgPts), _cam1_objPts(cam1_objPts) {}
    
    template <typename T>
    bool operator()(const T* const transform,
                    const T* const pose1,
                    T* residuals) const {
        // transform[0,1,2] are the angle-axis rotation.
        // transform[3,4,5] are the translation.
        // ============================================================================
        // project object points in cam1 frame to cam0 image
        // ============================================================================
        T p[3], q[3], t[3];
        p[0] = T(_cam1_objPts.x);
        p[1] = T(_cam1_objPts.y);
        p[2] = T(_cam1_objPts.z);

        ceres::AngleAxisRotatePoint(pose1, p, q);
        q[0] += T(pose1[3]); q[1] += T(pose1[4]); q[2] += T(pose1[5]);

        ceres::AngleAxisRotatePoint(transform, q, t);
        t[0] += T(transform[3]); t[1] += T(transform[4]); t[2] += T(transform[5]);

        T obj_pt_in_ocamcalib[3] = {t[0], t[1], t[2]};
        T img_pt_projected[2];
        _ocamcalib_cam0.world2cam(img_pt_projected, obj_pt_in_ocamcalib);
        
        // // the residuals is the distance between scan point and the checkerboard plane
        residuals[0] = T(_cam0_imgPts.x) - img_pt_projected[0];
        residuals[1] = T(_cam0_imgPts.y) - img_pt_projected[1];
        return true;
    }
    
    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(OCamCalibModel ocamcalib_cam0, cv::Point2f cam0_imgPts, cv::Point3f cam1_objPts) {
        return (new ceres::AutoDiffCostFunction<ReprojectionError1, 2, 6, 6>(
                    new ReprojectionError1(ocamcalib_cam0, cam0_imgPts, cam1_objPts)));
    }

    cv::Point2f _cam0_imgPts;    
    cv::Point3f _cam1_objPts;
    OCamCalibModel _ocamcalib_cam0;
};


struct ReprojectionError2 {
    ReprojectionError2(OCamCalibModel ocamcalib_cam1, cv::Point2f cam1_imgPts, cv::Point3f cam0_objPts)
    : _ocamcalib_cam1(ocamcalib_cam1), _cam1_imgPts(cam1_imgPts), _cam0_objPts(cam0_objPts) {}
    
    template <typename T>
    bool operator()(const T* const transform,
                    const T* const pose0,
                    T* residuals) const {
        // transform[0,1,2] are the angle-axis rotation.
        // transform[3,4,5] are the translation.
        // ============================================================================
        // project object points in cam0 frame to cam1 image
        // ============================================================================
        T p1[3], q1[3], t1[3];
        p1[0] = T(_cam0_objPts.x);
        p1[1] = T(_cam0_objPts.y);
        p1[2] = T(_cam0_objPts.z);

        ceres::AngleAxisRotatePoint(pose0, p1, q1);
        q1[0] += T(pose0[3]); q1[1] += T(pose0[4]); q1[2] += T(pose0[5]);

        T inv_transform[6];
        T tvec[3], tvec_inv[3];
        for (size_t i=0; i<3; i++){
            inv_transform[i] = -transform[i];
            tvec[i] = -transform[i+3];
        }


        ceres::AngleAxisRotatePoint(inv_transform, tvec, tvec_inv);
        for (size_t i=3; i<6; i++){
            inv_transform[i] = tvec_inv[i-3];
        }

        ceres::AngleAxisRotatePoint(inv_transform, q1, t1);
        t1[0] += T(inv_transform[3]); t1[1] += T(inv_transform[4]); t1[2] += T(inv_transform[5]);

        T obj_pt_in_ocamcalib1[3] = {t1[0], t1[1], t1[2]};
        T img_pt_projected1[2];
        _ocamcalib_cam1.world2cam(img_pt_projected1, obj_pt_in_ocamcalib1);
      
        residuals[0] = T(_cam1_imgPts.x) - img_pt_projected1[0];
        residuals[1] = T(_cam1_imgPts.y) - img_pt_projected1[1];
        return true;
    }
    
    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(OCamCalibModel ocamcalib_cam1, cv::Point2f cam1_imgPts, cv::Point3f cam0_objPts) {
        return (new ceres::AutoDiffCostFunction<ReprojectionError2, 2, 6, 6>(
                    new ReprojectionError2(ocamcalib_cam1, cam1_imgPts, cam0_objPts)));
    }

    cv::Point2f _cam1_imgPts;    
    cv::Point3f _cam0_objPts;
    OCamCalibModel _ocamcalib_cam1;

};

struct ReprojectionError3 {
    ReprojectionError3(OCamCalibModel ocamcalib_cam0, cv::Point2f cam0_imgPts, cv::Point3f cam0_objPts)
    : _ocamcalib_cam0(ocamcalib_cam0), _cam0_imgPts(cam0_imgPts), _cam0_objPts(cam0_objPts) {}
    
    template <typename T>
    bool operator()(const T* const pose0,
                    T* residuals) const {
        // ============================================================================
        // project object points in cam0 frame to cam1 image
        // ============================================================================
        T p[3], q[3], t[3];
        p[0] = T(_cam0_objPts.x);
        p[1] = T(_cam0_objPts.y);
        p[2] = T(_cam0_objPts.z);

        ceres::AngleAxisRotatePoint(pose0, p, q);
        q[0] += T(pose0[3]); q[1] += T(pose0[4]); q[2] += T(pose0[5]);

        T img_pt_projected[2];
        _ocamcalib_cam0.world2cam(img_pt_projected, q);
        
        // // the residuals is the distance between scan point and the checkerboard plane
        residuals[0] = T(_cam0_imgPts.x) - img_pt_projected[0];
        residuals[1] = T(_cam0_imgPts.y) - img_pt_projected[1];
        return true;
    }
    
    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(OCamCalibModel ocamcalib_cam0, cv::Point2f cam0_imgPts, cv::Point3f cam0_objPts) {
        return (new ceres::AutoDiffCostFunction<ReprojectionError3, 2, 6>(
                    new ReprojectionError3(ocamcalib_cam0, cam0_imgPts, cam0_objPts)));
    }

    cv::Point2f _cam0_imgPts;    
    cv::Point3f _cam0_objPts;
    OCamCalibModel _ocamcalib_cam0;

};


#endif
