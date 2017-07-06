/**
 * This file is part of vo.
 *
 * Muyuan Lin, 2016
 * For more information see <https://github.com/muyuanlin/vo>
 */

#include "optimizer.h"

#include "ceres/ceres.h"
#include "ceres/rotation.h"


/*
 * optimize transform between LIDAR and camera
 * input: 
        object_points:
        lidar_scan:
        object_poses: a list of object poses of each frame
        parameter: 6 elements array which stores the transform between LIDAR and camera
        
 */
void optimizer::bundleAdjustment(const std::vector<cv::Point3f> object_points, 
                                 const std::vector<std::vector<cv::Point3f> > lidar_scan.
                                 const std::vector<Eigen::Matrix4d> object_poses,
                                 double* paramter
                                 )
{ 
    // set inital estimation, checkerboad dimension

    // cull out points of lidar scan that are within checkerboard
    
    //
    int num_cameras_  = 1;
    int num_observations = local_map.num_observations();
    int num_points_ = local_map.num_points();   
    
    // Create residuals for each observation in the bundle adjustment problem. The
    // parameters for cameras and points are added automatically.
    ceres::Problem problem;
    
    int camera_tag = local_map.camera_index_[0];
    int track_index = 0;
    for (int i = 0; i < local_map.num_observations(); ++i) {
        // Each Residual block takes a point and a camera as input and outputs a 2
        // dimensional residual. Internally, the cost function stores the observed
        // image location and compares the reprojection against the observation.
        if (track_index > 100){
            if (camera_tag == local_map.camera_index_[i]){
                continue;
            }
            else{
                camera_tag = local_map.camera_index_[i];
                track_index = 0;
                continue;
            }
        }
        else{
            double x, y;
            Reprojection(&paramter[local_map.camera_index_[i]*6], &paramter[num_cameras_*6 + local_map.point_index_[i]*3],
                         x, y);
            x = observations[2 * i + 0] - x;
            y = observations[2 * i + 1] - y;
//            std::cout <<x <<" " <<y <<" " << observations[2 * i + 0] <<" " <<observations[2 * i + 1] <<std::endl;
            if ( x*x + y*y < 300){
                double *point_i = &paramter[6*num_cameras_ + local_map.point_index_[i]*3];
                ceres::CostFunction* cost_function =
                SnavelyReprojectionError::Create(observations[2 * i + 0],
                                                 observations[2 * i + 1],
                                                 point_i[0],
                                                 point_i[1],
                                                 point_i[2]);
                problem.AddResidualBlock(cost_function,
                                         NULL /* squared loss */,
                                         &paramter[local_map.camera_index_[i]*6]//,
                                         //&paramter[num_cameras_*6 + local_map.point_index_[i]*3]
                                         );
                    
                track_index++;
            }
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


