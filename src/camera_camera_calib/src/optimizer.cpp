/**
 * This file is part of lidar_camera_calib.
 *
 * Muyuan Lin, 2017
 * For more information see <https://github.com/muyuanlin/lidar-camera-calib>
 */

#include "camera_camera_calib/optimizer.h"
#include "camera_camera_calib/omniModel.h"

/*
 * optimize transform between LIDAR and camera
 * input: 
        object_points:
        lidar_scan:
        object_poses: a list of object poses of each frame
        parameter: 6 elements array which stores the transform between LIDAR and camera
        
 */
void optimizer::bundleAdjustment(OmniModel &cam0,
                                 OmniModel &cam1,
                                 const std::vector<std::vector<cv::KeyPoint> > kps_vec_0,
                                 const std::vector<std::vector<cv::KeyPoint> > kps_vec_1,
                                 double* parameter
                                 )
{   
    // Create residuals for each observation in the bundle adjustment problem. The
    // parameters for cameras and points are added automatically.
    ceres::Problem problem; 
    for (size_t i = 0; i < kps_vec_0.size(); ++i) {
        for (size_t j=0; j < kps_vec_0[i].size(); j++){
        // Each Residual block takes a point and a camera as input and outputs a 1
        // dimensional residual. 
            ceres::CostFunction* cs = SnavelyReprojectionError::Create( cam0, cam1,
                                                                        kps_vec_0[i][j], kps_vec_1[i][j]);
            problem.AddResidualBlock(cs, NULL /* squared loss */, &parameter[0]);
        }
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

    
