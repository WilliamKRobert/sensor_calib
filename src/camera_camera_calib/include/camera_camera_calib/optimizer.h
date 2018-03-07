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
#include "camera_camera_calib/utility.h"



class optimizer
{
public:
    optimizer(){}
    void bundleAdjustment(OmniModel& cam0,
                          std::vector<cv::Point2f>& cam0_imgPts,
                          std::vector<cv::Point3f>& cam1_objPts,
                          double* parameter);

    template <typename T>
    void bundleAdjustment(OCamCalibModel& ocamcalib_cam0,
                         std::vector<std::vector<cv::Point_<T> > >& cam0_imgPts,
                         std::vector<std::vector<cv::Point3_<T> > >& cam0_objPts,
                         double* poses0,
                         OCamCalibModel& ocamcalib_cam1,
                         std::vector<std::vector<cv::Point_<T> > >& cam1_imgPts,
                         std::vector<std::vector<cv::Point3_<T> > >& cam1_objPts,
                         double* poses1,
                         double* parameter);

    void bundleAdjustment(OCamCalibModel& ocamcalib_cam,
             const std::vector<std::vector<cv::Point2d > >& imgPts,
             const std::vector<std::vector<cv::Point3d > >& objPts,
             double* poses);

    template <typename T>
    void singleCameraBundleAdjustment(
        const std::vector<std::vector<cv::Point_<T> > >&img_pt_vec, 
        const std::vector<std::vector<cv::Point3_<T> > >&obj_pt_vec, 
        size_t taylor_order, T* parameter, T* pose_array);

    template <typename T>
    void singleCameraBundleAdjustment(
        const std::vector<std::vector<cv::Point_<T> > >&img_pt_vec, 
        const std::vector<std::vector<cv::Point3_<T> > >&obj_pt_vec,
        T* pose_array, 
        size_t taylor_order, T* parameter);

    template <typename T>
    void singleCameraForwardBA(
        const std::vector<std::vector<cv::Point_<T> > >&img_pt_vec, 
        const std::vector<std::vector<cv::Point3_<T> > >&obj_pt_vec, 
        size_t taylor_order, T* parameter, T* pose_array);
};

struct ReprojectionErrorCam0 {
    ReprojectionErrorCam0(OCamCalibModel ocamcalib_cam0, cv::Point2f cam0_imgPts, 
                        cv::Point3f cam1_objPts, double *pose1)
    : _ocamcalib_cam0(ocamcalib_cam0), _cam0_imgPts(cam0_imgPts), 
        _cam1_objPts(cam1_objPts), _pose1(pose1) {}
    
    template <typename T>
    bool operator()(const T* const transform,
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

        T pose1[6];
        for (size_t i=0; i<6; i++)
            pose1[i] = T(_pose1[i]);
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
    
    static ceres::CostFunction* Create(OCamCalibModel ocamcalib_cam0, 
                                    cv::Point2f cam0_imgPts, 
                                    cv::Point3f cam1_objPts, 
                                    double* pose1) {
        return (new ceres::AutoDiffCostFunction<ReprojectionErrorCam0, 2, 6>(
                    new ReprojectionErrorCam0(ocamcalib_cam0, cam0_imgPts, 
                                                    cam1_objPts, pose1)));
    }

    cv::Point2f _cam0_imgPts;    
    cv::Point3f _cam1_objPts;
    OCamCalibModel _ocamcalib_cam0;
    double* _pose1;
};


struct ReprojectionErrorCam1 { //back projection on cam1
    ReprojectionErrorCam1(OCamCalibModel ocamcalib_cam1, cv::Point2f cam1_imgPts, 
                            cv::Point3f cam0_objPts, double *pose0)
    : _ocamcalib_cam1(ocamcalib_cam1), _cam1_imgPts(cam1_imgPts),\
     _cam0_objPts(cam0_objPts),_pose0(pose0) {}
    
    template <typename T>
    bool operator()(const T* const transform,
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

        T pose0[6];
        for (size_t i=0; i<6; i++)
            pose0[i] = T(_pose0[i]);
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
    static ceres::CostFunction* Create(OCamCalibModel ocamcalib_cam1, 
                                        cv::Point2f cam1_imgPts, 
                                        cv::Point3f cam0_objPts,
                                        double *pose0) {
        return (new ceres::AutoDiffCostFunction<ReprojectionErrorCam1, 2, 6>(
                    new ReprojectionErrorCam1(ocamcalib_cam1, cam1_imgPts, 
                                            cam0_objPts, pose0)));
    }

    cv::Point2f _cam1_imgPts;    
    cv::Point3f _cam0_objPts;
    OCamCalibModel _ocamcalib_cam1;
    double* _pose0;

};

struct PoseReprojectionError {
    PoseReprojectionError(OCamCalibModel ocamcalib_cam, cv::Point2d imgPts, cv::Point3d objPts)
    : _ocamcalib_cam(ocamcalib_cam), _imgPts(imgPts), _objPts(objPts) {}
    
    template <typename T>
    bool operator()(const T* const pose,
                    T* residuals) const {
        // ============================================================================
        // project object points in cam0 frame to cam1 image
        // ============================================================================
        T p[3], q[3], t[3];
        p[0] = T(_objPts.x);
        p[1] = T(_objPts.y);
        p[2] = T(_objPts.z);

        ceres::AngleAxisRotatePoint(pose, p, q);
        q[0] += T(pose[3]); q[1] += T(pose[4]); q[2] += T(pose[5]);

        T img_pt_projected[2];
        _ocamcalib_cam.world2cam(img_pt_projected, q);
        
        // // the residuals is the distance between scan point and the checkerboard plane
        residuals[0] = T(_imgPts.x) - img_pt_projected[0];
        residuals[1] = T(_imgPts.y) - img_pt_projected[1];
        return true;
    }
    
    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(OCamCalibModel ocamcalib_cam, 
                                        cv::Point2d imgPts, cv::Point3d objPts) {
        return (new ceres::AutoDiffCostFunction<PoseReprojectionError, 2, 6>(
                new PoseReprojectionError(ocamcalib_cam, imgPts, objPts)));
    }

    cv::Point2d _imgPts;    
    cv::Point3d _objPts;
    OCamCalibModel _ocamcalib_cam;

};


struct singleCameraReprojectionError{
    singleCameraReprojectionError(
        cv::Point2d imgPts, cv::Point3d objPts, size_t taylor_order)
        :_imgPts(imgPts), _objPts(objPts), _taylor_order(taylor_order)
    {}


    bool operator()(const double* const parameter,
                    const double* const pose,
                    double* residuals) const 
    {
        std::vector<double> poly;
        for (size_t i=0; i<_taylor_order; i++){
            poly.push_back(parameter[i]);
        }

        size_t width = 1050;
        size_t height = 1050;

        double c = parameter[_taylor_order];
        double d = parameter[_taylor_order+1];
        double e = parameter[_taylor_order+2];

        double xc = parameter[_taylor_order+3];
        double yc = parameter[_taylor_order+4];

        OCamCalibModel cam(width, height, poly, poly, xc, yc, c, d, e);

        double p[3], q[3], t[3];
        p[0] = _objPts.x;
        p[1] = _objPts.y;
        p[2] = _objPts.z;

        ceres::AngleAxisRotatePoint(pose, p, q);
        q[0] += pose[3]; q[1] += pose[4]; q[2] += pose[5];

        double img_pt_projected[2];

        
        cam.world2cam(img_pt_projected, q);
        
        // the residuals is the distance between scan point and the checkerboard plane
        residuals[0] = _imgPts.x - img_pt_projected[0];
        residuals[1] = _imgPts.y - img_pt_projected[1];
        return true;
    }

    static ceres::CostFunction* Create(
        cv::Point2f imgPts, cv::Point3f objPts, size_t taylor_order) {
        return (new ceres::NumericDiffCostFunction<
                singleCameraReprojectionError, ceres::CENTRAL, 2, 11+5, 6>(
                new singleCameraReprojectionError(imgPts, objPts, taylor_order)));
    }


    cv::Point2d _imgPts;    
    cv::Point3d _objPts;
    size_t _taylor_order;
};


struct intrinsicsReprojectionError{
    intrinsicsReprojectionError(
        cv::Point2d imgPts, cv::Point3d objPts, double *pose, size_t taylor_order)
        :_imgPts(imgPts), _objPts(objPts), _pose(pose), _taylor_order(taylor_order)
    {}


    bool operator()(const double* const parameter,
                    double* residuals) const 
    {
        std::vector<double> poly;
        // poly.push_back(parameter[0]);
        // poly.push_back(0.0);
        for (size_t i=0; i<_taylor_order; i++){
            poly.push_back(parameter[i]);
        }

        size_t width = 1050;
        size_t height = 1050;

        double c = parameter[_taylor_order];
        double d = parameter[_taylor_order+1];
        double e = parameter[_taylor_order+2];

        double xc = parameter[_taylor_order+3];
        double yc = parameter[_taylor_order+4];

        OCamCalibModel cam(width, height, poly, xc, yc, c, d, e);

        double p[3], q[3], t[3];
        p[0] = _objPts.x;
        p[1] = _objPts.y;
        p[2] = _objPts.z;

        ceres::AngleAxisRotatePoint(_pose, p, q);
        q[0] += _pose[3]; q[1] += _pose[4]; q[2] += _pose[5];

        double img_pt_projected[2];

        
        cam.world2cam(img_pt_projected, q);
        
        residuals[0] = _imgPts.x - img_pt_projected[0];
        residuals[1] = _imgPts.y - img_pt_projected[1];
        return true;
    }

    static ceres::CostFunction* Create(
        cv::Point2f imgPts, cv::Point3f objPts, double *pose, size_t taylor_order) {
        return (new ceres::NumericDiffCostFunction<
                intrinsicsReprojectionError, ceres::CENTRAL, 2, 14+5>(
                new intrinsicsReprojectionError(imgPts, objPts, pose, taylor_order)));
    }


    cv::Point2d _imgPts;    
    cv::Point3d _objPts;
    double *_pose;
    size_t _taylor_order;
};



// projection error
// single camera calibration
// forward projection function is optimized
struct forwardIntrinsicsReprojectionError{
    forwardIntrinsicsReprojectionError(
        cv::Point2d imgPts, cv::Point3d objPts, size_t taylor_order)
        :_imgPts(imgPts), _objPts(objPts), _taylor_order(taylor_order)
    {}


    bool operator()(const double* const parameter,
                    const double* const pose,
                    double* residuals) const 
    {
        std::vector<double> poly;
        poly.push_back(parameter[0]);
        poly.push_back(0.0);
        for (size_t i=1; i<_taylor_order; i++){
            poly.push_back(parameter[i]);
        }

        size_t width = 1050;
        size_t height = 1050;

        double c = parameter[_taylor_order];
        double d = parameter[_taylor_order+1];
        double e = parameter[_taylor_order+2];

        double xc = parameter[_taylor_order+3];
        double yc = parameter[_taylor_order+4];

        OCamCalibModel cam(width, height, poly, poly, xc, yc, c, d, e);

        double p[3], q[3], t[3];
        p[0] = _objPts.x;
        p[1] = _objPts.y;
        p[2] = _objPts.z;

        ceres::AngleAxisRotatePoint(pose, p, q);
        q[0] += pose[3]; q[1] += pose[4]; q[2] += pose[5];

        double img_pt_projected[2];

        
        cam.world2cam_naive(img_pt_projected, q);
        
        // the residuals is the distance between scan point and the checkerboard plane
        residuals[0] = _imgPts.x - img_pt_projected[0];
        residuals[1] = _imgPts.y - img_pt_projected[1];
        return true;
    }

    static ceres::CostFunction* Create(
        cv::Point2f imgPts, cv::Point3f objPts, size_t taylor_order) {
        return (new ceres::NumericDiffCostFunction<
                forwardIntrinsicsReprojectionError, ceres::CENTRAL, 2, 5+5, 6>(
                new forwardIntrinsicsReprojectionError(imgPts, objPts, taylor_order)));
    }


    cv::Point2d _imgPts;    
    cv::Point3d _objPts;
    size_t _taylor_order;
};
#endif
