/**
 * This file is part of lidar_camera_calib.
 *
 * Muyuan Lin, 2017
 * For more information see <https://github.com/muyuanlin/lidar-camera-calib>
 */

#include "camera_camera_calib/optimizerRay.h"
#include "camera_camera_calib/omniModel.h"
#include "camera_camera_calib/ocamCalibModel.h"

/*
 * optimize transform between camera and camera
 * input: 
        image points of cam0:
        object points in cam 1 frame:
        parameter: 6 elements array which stores the transform between cameras
        
 */
void optimizer::bundleAdjustment(OCamCalibModel& ocamcalib_cam0,
                                 std::vector<std::vector<cv::Point3f> >& cam0_rays,
                                 std::vector<std::vector<cv::Point3f> >& cam1_rays,
                                 double* parameter
                                 )
{   
    // Create residuals for each observation in the bundle adjustment problem. The
    // parameters for cameras and points are added automatically.
    ceres::Problem problem; 
    
    for (size_t i = 0; i < cam0_rays.size(); ++i) {
        // Each Residual block takes a point and a camera as input and outputs a 1
        // dimensional residual. 
            ceres::CostFunction* cs = SnavelyReprojectionError::Create( ocamcalib_cam0, cam0_rays[i], cam1_rays[i]);
            problem.AddResidualBlock(cs, NULL /* squared loss */, &parameter[0]);
        
    }

    // Make Ceres automatically detect the bundle structure. Note that the
    // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
    // for standard bundle adjustment problems. 
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    // If solve small to medium sized problems, consider setting
    // use_explicit_schur_complement as true
    // options.use_explicit_schur_complement = true;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
}
    
