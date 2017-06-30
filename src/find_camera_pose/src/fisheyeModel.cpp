#include <iostream>

#include "find_camera_pose/fisheyeModel.h"

using namespace std;

/** 
 * \brief Apply distortion to input point (from the normalised plane)
 *        and calculate jacobian
 *
 * \param mx_u undistorted x coordinate of point on the normalised plane
 * \param my_u undistorted y coordinate of point on the normalised plane
 * \param dx return value, to obtain the distorted point : mx_d = mx_u+dx_u
 * \param dy return value, to obtain the distorted point : my_d = my_u+dy_u
 */
void FisheyeModel::distortion(const double mx_u, const double my_u, 
                double &dx_u, double &dy_u, 
                double *dxdmx, double *dydmx,
                double *dxdmy, double *dydmy) {
  double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

  mx2_u = mx_u * mx_u;
  my2_u = my_u * my_u;
  mxy_u = mx_u * my_u;
  rho2_u = mx2_u + my2_u;
  rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
  dx_u = mx_u * rad_dist_u + 2 * p1 * mxy_u + p2 * (rho2_u + 2 * mx2_u);
  dy_u = my_u * rad_dist_u + 2 * p2 * mxy_u + p1 * (rho2_u + 2 * my2_u);

  *dxdmx = 1 + rad_dist_u + k1 * 2 * mx2_u + k2 * rho2_u * 4 * mx2_u
      + 2 * p1 * my_u + 6 * p2 * mx_u;
  *dydmx = k1 * 2 * mx_u * my_u + k2 * 4 * rho2_u * mx_u * my_u
      + p1 * 2 * mx_u + 2 * p2 * my_u;
  *dxdmy = *dydmx;
  *dydmy = 1 + rad_dist_u + k1 * 2 * my2_u + k2 * rho2_u * 4 * my2_u
      + 6 * p1 * my_u + 2 * p2 * mx_u;
}


// Use Gauss-Newton to undistort.
void FisheyeModel::undistortGN(const double u_d, const double v_d, double &u,
                                     double &v) {
  u = u_d;
  v = v_d;

  double ubar = u_d, vbar = v_d;
  double hat_u_d, hat_v_d;

  double ERROR = 1e-6;
  Eigen::Vector2d e;
  Eigen::Matrix2d F;
  do{
    distortion(ubar, vbar, hat_u_d, hat_v_d, &F(0, 0), &F(1, 0), &F(0, 1),
               &F(1, 1));

    e << u_d - ubar - hat_u_d, v_d - vbar - hat_v_d;
    Eigen::Vector2d du = (F.transpose() * F).inverse() * F.transpose() * e;

    ubar += du[0];
    vbar += du[1];

  }while(e.dot(e) > ERROR); 

  u = ubar;
  v = vbar;
}

// assume the undistorted image size is the same as distorted image
void FisheyeModel::undistortImage(const cv::Mat distorted, cv::Mat undistorted){
    size_t nn = 0;
    for (int y=0; y<undistorted.rows; y++){
      for (int x=0; x<undistorted.cols; x++){
        double ux, uy;
        undistortGN(x, y, ux, uy);
        if (uy >=0 && uy < undistorted.rows && ux >=0 && ux < undistorted.cols){
          ux = int(ux);
          uy = int(uy);
          if (++nn < 20) cout << uy << " " << ux << " " << y << " " <<x << endl;
          undistorted.at<double>(uy, ux) = distorted.at<double>(uy, ux);    
        }
      }
    }
}

