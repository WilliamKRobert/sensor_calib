/**
 * This file is part of lidar_camera_calib.
 *
 * Muyuan Lin, 2017
 * For more information see <https://github.com/muyuanlin/sensor-calib>
 */

#include "camera_camera_calib/optimizer.h"
#include "camera_camera_calib/omniModel.h"
#include "camera_camera_calib/ocamCalibModel.h"

/*
 * optimize transform between camera and camera
 * input: 
        image points of cam0:
        object points in cam 1 frame:
        parameter: 6 elements array which stores the transform between cameras
        
 */
template <typename T>
void optimizer::bundleAdjustment(OCamCalibModel& ocamcalib_cam0,
             std::vector<std::vector<cv::Point_<T> > >& cam0_imgPts,
             std::vector<std::vector<cv::Point3_<T> > >& cam0_objPts,
             double* poses0,
             OCamCalibModel& ocamcalib_cam1,
             std::vector<std::vector<cv::Point_<T> > >& cam1_imgPts,
             std::vector<std::vector<cv::Point3_<T> > >& cam1_objPts,
             double* poses1,
             double* parameter
            )
{   
    // Make Ceres automatically detect the bundle structure. Note that the
    // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
    // for standard bundle adjustment problems. 
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    // options.gradient_tolerance = 1e-16;
    // options.function_tolerance = 1e-16;
    // If solve small to medium sized problems, consider setting
    // use_explicit_schur_complement as true
    options.use_explicit_schur_complement = true;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;

    // Create residuals for each observation in the bundle adjustment problem. The
    // parameters for cameras and points are added automatically.
    ceres::Problem problem_transform; 

    for (size_t i = 0; i < cam0_imgPts.size(); ++i) {
        for (size_t j=0; j < cam0_imgPts[i].size(); ++j ){
            ceres::CostFunction* cs0 = 
                ReprojectionErrorCam0::Create( ocamcalib_cam0, 
                cam0_imgPts[i][j], cam1_objPts[i][j], &poses1[6*i]);

            problem_transform.AddResidualBlock(cs0, NULL /* squared loss */, 
                &parameter[0]);

            ceres::CostFunction* cs1 = 
                ReprojectionErrorCam1::Create( ocamcalib_cam1, 
                cam1_imgPts[i][j], cam0_objPts[i][j], &poses0[6*i]);

            problem_transform.AddResidualBlock(cs1, NULL /* squared loss */, 
                &parameter[0]);

        // ceres::CostFunction* cs0 = ReprojectionError1::Create( ocamcalib_cam0, cam0_imgPts[i][j], cam1_objPts[i][j]);
        // problem.AddResidualBlock(cs0, NULL /* squared loss */, &parameter[0], &poses1[6*i]);

        // ceres::CostFunction* cs1 = ReprojectionError2::Create(ocamcalib_cam1, cam1_imgPts[i][j], cam0_objPts[i][j]);
        // problem.AddResidualBlock(cs1, NULL  squared loss , &parameter[0], &poses0[6*i]);
        }
        
    }

    ceres::Solve(options, &problem_transform, &summary);
    std::cout << summary.FullReport() << "\n";
}

template void optimizer::bundleAdjustment<float>(
             OCamCalibModel& ocamcalib_cam0,
             std::vector<std::vector<cv::Point_<float> > >& cam0_imgPts,
             std::vector<std::vector<cv::Point3_<float> > >& cam0_objPts,
             double* poses0,
             OCamCalibModel& ocamcalib_cam1,
             std::vector<std::vector<cv::Point_<float> > >& cam1_imgPts,
             std::vector<std::vector<cv::Point3_<float> > >& cam1_objPts,
             double* poses1,
             double* parameter
             );

template void optimizer::bundleAdjustment<double>(
             OCamCalibModel& ocamcalib_cam0,
             std::vector<std::vector<cv::Point_<double> > >& cam0_imgPts,
             std::vector<std::vector<cv::Point3_<double> > >& cam0_objPts,
             double* poses0,
             OCamCalibModel& ocamcalib_cam1,
             std::vector<std::vector<cv::Point_<double> > >& cam1_imgPts,
             std::vector<std::vector<cv::Point3_<double> > >& cam1_objPts,
             double* poses1,
             double* parameter
             );                    


// FOR OPTIMIZATION OF TARGET POSE OBSERVED IN A SINGLE CAMERA
void optimizer::bundleAdjustment(OCamCalibModel& ocamcalib_cam,
             const std::vector<std::vector<cv::Point2d > >& imgPts,
             const std::vector<std::vector<cv::Point3d > >& objPts,
             double *poses
            )
{   
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.use_explicit_schur_complement = true;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;

    ceres::Problem problem;
    
    for (size_t i = 0; i < imgPts.size(); ++i) {
        for (size_t j=0; j < imgPts[i].size(); ++j ){
            ceres::CostFunction* cs = 
                PoseReprojectionError::Create(ocamcalib_cam, 
                    imgPts[i][j], objPts[i][j]);
            problem.AddResidualBlock(cs, NULL /* squared loss */, 
                    &poses[6*i]);
        }
        
    }
    
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
}  



// FOR OPTIMIZATION OF A SINGLE CAMERA
template <typename T>
void optimizer::singleCameraBundleAdjustment(
    const std::vector<std::vector<cv::Point_<T> > >&img_pt_vec, 
    const std::vector<std::vector<cv::Point3_<T> > >&obj_pt_vec, 
    size_t taylor_order, T* parameter, T* pose_array)
{   
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.use_explicit_schur_complement = true;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;

    ceres::Problem problem;
    
    for (size_t i = 0; i < img_pt_vec.size(); ++i) {
        for (size_t j=0; j < img_pt_vec[i].size(); ++j ){
            ceres::CostFunction* cs = 
                singleCameraReprojectionError::Create(
                img_pt_vec[i][j], obj_pt_vec[i][j], taylor_order);
            problem.AddResidualBlock(cs, NULL /* squared loss */, 
                    &parameter[0], &pose_array[6*i]);
        }
        
    }
    
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
}

template void optimizer::singleCameraBundleAdjustment<double>(
    const std::vector<std::vector<cv::Point_<double> > >&img_pt_vec, 
    const std::vector<std::vector<cv::Point3_<double> > >&obj_pt_vec, 
    size_t taylor_order, 
    double* parameter, 
    double* pose_array);  





// FOR OPTIMIZATION OF A SINGLE CAMERA
template <typename T>
void optimizer::singleCameraBundleAdjustment(
    const std::vector<std::vector<cv::Point_<T> > >&img_pt_vec, 
    const std::vector<std::vector<cv::Point3_<T> > >&obj_pt_vec, 
    T* pose_array,
    size_t taylor_order, T* parameter)
{   
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.use_explicit_schur_complement = true;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;

    ceres::Problem problem;
    
    for (size_t i = 0; i < img_pt_vec.size(); ++i) {
        for (size_t j=0; j < img_pt_vec[i].size(); ++j ){
            ceres::CostFunction* cs = 
                intrinsicsReprojectionError::Create(
                img_pt_vec[i][j], obj_pt_vec[i][j], &pose_array[6*i], taylor_order);
            problem.AddResidualBlock(cs, NULL /* squared loss */, 
                    &parameter[0]);
        }
        
    }
    
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
}

template void optimizer::singleCameraBundleAdjustment<double>(
    const std::vector<std::vector<cv::Point_<double> > >&img_pt_vec, 
    const std::vector<std::vector<cv::Point3_<double> > >&obj_pt_vec, 
    double* pose_array,
    size_t taylor_order, 
    double* parameter);  

    
// FOR OPTIMIZATION OF INTRINSIC PARAMETERS OF A SINGLE CAMERA
// OPTIMIZE FORWARD PARAMETERS
template <typename T>
void optimizer::singleCameraForwardBA(
    const std::vector<std::vector<cv::Point_<T> > >&img_pt_vec, 
    const std::vector<std::vector<cv::Point3_<T> > >&obj_pt_vec, 
    size_t taylor_order, T* parameter, T* pose_array)
{   
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.use_explicit_schur_complement = true;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 50;
    ceres::Solver::Summary summary;

    ceres::Problem problem;
    
    for (size_t i = 0; i < img_pt_vec.size(); ++i) {
        for (size_t j=0; j < img_pt_vec[i].size(); ++j ){
            ceres::CostFunction* cs = 
                forwardIntrinsicsReprojectionError::Create(
                img_pt_vec[i][j], obj_pt_vec[i][j], taylor_order);
            problem.AddResidualBlock(cs, NULL /* squared loss */, 
                    &parameter[0], &pose_array[6*i]);
        }
        
    }
    
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
}

template void optimizer::singleCameraForwardBA<double>(
    const std::vector<std::vector<cv::Point_<double> > >&img_pt_vec, 
    const std::vector<std::vector<cv::Point3_<double> > >&obj_pt_vec, 
    size_t taylor_order, 
    double* parameter, 
    double* pose_array);  