#include <iostream>

#include "camera_camera_calib/omniModel.h"

using namespace std;

/*
 * Most of codes are adapted from Kalibr.
 */

/** 
 * \brief Apply distortion to input point (from the normalised plane)
 *        and calculate jacobian
 *
 * \param mx_u undistorted x coordinate of point on the normalised plane
 * \param my_u undistorted y coordinate of point on the normalised plane
 * \param dx return value, to obtain the distorted point : mx_d = mx_u+dx_u
 * \param dy return value, to obtain the distorted point : my_d = my_u+dy_u
 */
void OmniModel::distortion(const double mx_u, const double my_u, 
                double *dx_u, double *dy_u, 
                double *dxdmx, double *dydmx,
                double *dxdmy, double *dydmy) const{
  double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

  mx2_u = mx_u * mx_u;
  my2_u = my_u * my_u;
  mxy_u = mx_u * my_u;
  rho2_u = mx2_u + my2_u;
  rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
  *dx_u = mx_u * rad_dist_u + 2 * p1 * mxy_u + p2 * (rho2_u + 2 * mx2_u);
  *dy_u = my_u * rad_dist_u + 2 * p2 * mxy_u + p1 * (rho2_u + 2 * my2_u);

  *dxdmx = 1 + rad_dist_u + k1 * 2 * mx2_u + k2 * rho2_u * 4 * mx2_u
      + 2 * p1 * my_u + 6 * p2 * mx_u;
  *dydmx = k1 * 2 * mx_u * my_u + k2 * 4 * rho2_u * mx_u * my_u
      + p1 * 2 * mx_u + 2 * p2 * my_u;
  *dxdmy = *dydmx;
  *dydmy = 1 + rad_dist_u + k1 * 2 * my2_u + k2 * rho2_u * 4 * my2_u
      + 6 * p1 * my_u + 2 * p2 * mx_u;
}


// Use Gauss-Newton to undistort.
void OmniModel::undistortGN(const double u_d, const double v_d, double *u,
                                     double *v) const {
  *u = u_d;
  *v = v_d;

  double ubar = u_d, vbar = v_d;
  double hat_u_d, hat_v_d;

  double ERROR = 1e-16;
  Eigen::Vector2d e;
  Eigen::Matrix2d F;
  do{
    distortion(ubar, vbar, &hat_u_d, &hat_v_d, &F(0, 0), &F(1, 0), &F(0, 1),
               &F(1, 1));

    e << u_d - ubar - hat_u_d, v_d - vbar - hat_v_d;
    Eigen::Vector2d du = (F.transpose() * F).inverse() * F.transpose() * e;

    ubar += du[0];
    vbar += du[1];

  }while(e.dot(e) > ERROR); 

  *u = ubar;
  *v = vbar;
}

// assume the undistorted image size is the same as distorted image
void OmniModel::undistortImage(const cv::Mat distorted, cv::Mat undistorted){
    size_t nn = 0;
    for (int y=0; y<undistorted.rows; y++){
      for (int x=0; x<undistorted.cols; x++){
        double ux, uy;
        undistortGN(x, y, &ux, &uy);
        if (uy >=0 && uy < undistorted.rows && ux >=0 && ux < undistorted.cols){
          ux = int(ux);
          uy = int(uy);
          if (++nn < 20) cout << uy << " " << ux << " " << y << " " <<x << endl;
          undistorted.at<double>(uy, ux) = distorted.at<double>(uy, ux);    
        }
      }
    }
}

bool OmniModel::isUndistortedKeypointValid(
    const double rho2_d) const {
  double one_over_xixi_m_1 = 1.0 / (xi * xi - 1);
  return xi <= 1.0 || rho2_d <= one_over_xixi_m_1;
}

bool OmniModel::keypointToEuclidean(
    const Eigen::Vector2d & keypoint,
    Eigen::Vector3d & outPoint) const{

  // EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
  //     Eigen::MatrixBase<DERIVED_P>, 3);
  // EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
  //     Eigen::MatrixBase<DERIVED_K>, 2);

  // Eigen::MatrixBase<DERIVED_P> & outPoint = const_cast<Eigen::MatrixBase<
  //     DERIVED_P> &>(outPointConst);
  // outPoint.derived().resize(3);

  // Unproject...
  double recip_fu = 1.0 / fu;
  double recip_fv = 1.0 / fv;
  outPoint[0] = recip_fu * (keypoint[0] - u0);
  outPoint[1] = recip_fv * (keypoint[1] - v0);

  // Re-distort
  // _distortion.undistort(outPoint.derived().template head<2>());
  Eigen::Vector2d rectified_point;
  undistortGN(outPoint[0], outPoint[1], &rectified_point[0], &rectified_point[1]);
  //cout << "Undistort Point: " << rectified_point[0] << " " << rectified_point[1] << endl;
  outPoint[0] = rectified_point[0];
  outPoint[1] = rectified_point[1];

  double rho2_d = outPoint[0] * outPoint[0] + outPoint[1] * outPoint[1];

  if (!isUndistortedKeypointValid(rho2_d))
    return false;

  outPoint[2] = 1 - xi * (rho2_d + 1) / (xi + sqrt(1 + (1 - xi * xi) * rho2_d));

  return true;

}


/* Estimate the transformation of the camera with respect to the calibration target
 *        On success out_T_t_c is filled in with the transformation that takes points from
 *        the camera frame to the target frame
 * input: Ms: image points
 *        Ps: object points in world frame
 * output: out_T_t_c estimated object pose
           return true on success
 *
 * These functions were developed with the help of Lionel Heng and the excellent camodocal
 * https://github.com/hengli/camodocal
 */
bool OmniModel::estimateTransformation(
    std::vector<cv::Point2f> Ms,
    std::vector<cv::Point3f> Ps,
    Eigen::Matrix4d &  out_T_t_c) {
  // using detail::square;

  // Convert all target corners to a fakey pinhole view.
  size_t count = 0;
  for (size_t i = 0; i < Ms.size(); ++i) {
    Eigen::Vector3d targetPoint(Ps[i].x, Ps[i].y, Ps[i].z);
    Eigen::Vector2d imagePoint(Ms[i].x, Ms[i].y);
    Eigen::Vector3d backProjection;

    if (keypointToEuclidean(imagePoint, backProjection)
        && backProjection[2] > 0.0) {
      double x = backProjection[0];
      double y = backProjection[1];
      double z = backProjection[2];
      Ps.at(count).x = targetPoint[0];
      Ps.at(count).y = targetPoint[1];
      Ps.at(count).z = targetPoint[2];

      Ms.at(count).x = x / z;
      Ms.at(count).y = y / z;
      ++count;
    } else {
    // Debug
    }
  }

  Ps.resize(count);
  Ms.resize(count);

  std::vector<double> distCoeffs(4, 0.0);

  cv::Mat rvec(3, 1, CV_64F);
  cv::Mat tvec(3, 1, CV_64F);

  if (Ps.size() < 4) {
  // SM_DEBUG_STREAM(
  // "At least 4 points are needed for calling PnP. Found " << Ps.size());
    return false;
  }

  // Call the OpenCV pnp function.
  cv::solvePnP(Ps, Ms, cv::Mat::eye(3, 3, CV_64F), distCoeffs, rvec, tvec);

  // convert the rvec/tvec to a transformation
  cv::Mat C_camera_model = cv::Mat::eye(3, 3, CV_64F);
  Eigen::Matrix4d T_camera_model = Eigen::Matrix4d::Identity();
  cv::Rodrigues(rvec, C_camera_model);
  for (int r = 0; r < 3; ++r) {
    T_camera_model(r, 3) = tvec.at<double>(r, 0);
    for (int c = 0; c < 3; ++c) {
      T_camera_model(r, c) = C_camera_model.at<double>(r, c);
    }
  }
  // out_T_t_c = T_camera_model.inverse();
  out_T_t_c = T_camera_model; // object pose in camera frame
  return true;
}


