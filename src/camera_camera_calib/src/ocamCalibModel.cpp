#include <math.h>
#include <gsl/gsl_poly.h>
#include <stdlib.h>
#include <stdio.h>

#include "camera_camera_calib/ocamCalibModel.h" 

// inline double pow(double rho, size_t n){
//       double res = 1;
//       for (size_t i=0; i<n; i++){
//           res *= rho;
//       }
//       return res;
// }

// inline bool equal2zero(double num, double epsilon=1e-10){
//     return fabs(num) < epsilon;
// }

// void OCamCalibModel::setParameter(int width, int height,
//               double u0, double v0, 
//               std::vector<double> ss,
//               std::vector<double> ss_inv,
//               double c, double d, double e){ // intrinsics: 3 by 3 distortionCoeff: 4 by 1
//         m_u0 = u0;
//         m_v0 = v0;

//         for (size_t i=0; i<ss.size(); i++)
//             m_ss.push_back(ss[i]);

//         for (size_t i=0; i<ss_inv.size(); i++)
//             m_ss_inv.push_back(ss[i]);

//         m_c = c;
//         m_d = d;
//         m_e = e;
// }

// /*
//  * Project a image point onto the unit sphere
//  * input:   Ms: image points
//  * output:  Ps: object points in camera frame 
//  * Adapted from camera model of OCamCalib
//  */
// bool OCamCalibModel::cam2world(const cv::Point2f &Ms,
//                           cv::Point3f &Ps)const{
    
//     Eigen::Matrix2d A;
//     A << m_c, m_d, m_e, 1;
//     Eigen::Matrix2d A_inv = A.inverse();
//     Eigen::Vector2d C;
//     C << Ms.x - m_u0, Ms.y - m_v0;

//     Eigen::Vector2d m = A_inv * C;
//     double rho = sqrt( m(0)*m(0) + m(1)*m(1) );

//     double f = 0;
   
//     for (size_t i=0; i<m_ss.size(); i++){
//         f += m_ss[i] * pow(rho, i); 
//     }
   
//     Ps.x = m(0) / f;
//     Ps.y = m(1) / f;
//     Ps.z = 1;

//     return true;
    
// }

// bool OCamCalibModel::world2cam(const cv::Point3f &Ps,
//                           cv::Point2f &Ms)const{
//     cv::Point2f Ms_ideal;
//     bool bfind = omni3d2pixel(Ps, Ms_ideal);

//     if (!bfind) return false;
 
//     Ms.x = Ms_ideal.x * m_c + Ms_ideal.y * m_d + m_u0;
//     Ms.y = Ms_ideal.x * m_e + Ms_ideal.y       + m_v0;

//     return true;
// }

// bool OCamCalibModel::omni3d2pixel(const cv::Point3f &Ps,
//                           cv::Point2f &Ms){
//     double m = Ps.z / sqrt(Ps.x * Ps.x + Ps.y * Ps.y)  ; 
    
//     std::vector<double> poly_coeff = m_ss_inv;

//     std::vector<double> poly_coeff_copy = poly_coeff;


//     size_t n = poly_coeff.size();
//     poly_coeff_copy[n-2] = poly_coeff[n-2] - m;

//     int i;
//     /* coefficients of P(x) =  -1 + x^5  */
//     double* a = new double(n);  
//     double* z = new double((n-1)*2);
//     for (size_t i=0; i<n; i++)
//         a[i] = poly_coeff[i];

//     gsl_poly_complex_workspace * w 
//         = gsl_poly_complex_workspace_alloc (n);
    
//     gsl_poly_complex_solve (a, n, w, z);

//     gsl_poly_complex_workspace_free (w);

//     std::vector<double> res;
//     for (size_t i = 0; i < n-1; i++)
//     {
//         std::cout << i << " " << z[2*i] << " " << z[2*i+1] << std::endl;
//         if ( z[2*i] > 0 && equal2zero(z[2*i+1]) )
//             res.push_back(z[2*i]);
//     }

//     if (res.size() == 0)
//         return false;
    
//     auto prho = std::min_element(std::begin(res), std::end(res));
//     rho = *prho;

//     Ms.x = Ps.x / sqrt(Ps.x * Ps.x + Ps.y * Ps.y) * rho;
//     Ms.y = Ps.y / sqrt(Ps.x * Ps.x + Ps.y * Ps.y) * rho;
//     return true;
// }

// bool OCamCalibModel::estimateTransformation(std::vector<cv::Point2f> Ms,
//                                 std::vector<cv::Point3f> Ps,
//                                 Eigen::Matrix4d &  out_T_t_c){
//     return true;
// }


/*
 * directly from OCamCalib
 */
/*------------------------------------------------------------------------------
   Example code that shows the use of the 'cam2world" and 'world2cam" functions
   Shows also how to undistort images into perspective or panoramic images
   Copyright (C) 2008 DAVIDE SCARAMUZZA, ETH Zurich  
   Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org
------------------------------------------------------------------------------*/
#define CMV_MAX_BUF 1024  

inline double pow(double rho, size_t n){
      double res = 1;
      for (size_t i=0; i<n; i++){
          res *= rho;
      }
      return res;
}

//------------------------------------------------------------------------------
int OCamCalibModel::get_ocam_model(char *filename)
{
    FILE *f;
    char buf[CMV_MAX_BUF];
    int i;
    //Open filename
    if(!(f=fopen(filename,"r")))
    {
     printf("File %s cannot be opened\n", filename);          
     return -1;
    }

    //Read polynomial coefficients
    fgets(buf,CMV_MAX_BUF,f);
    fscanf(f,"\n");
    fscanf(f,"%d", &m_len_pol);
    for (i = 0; i < m_len_pol; i++)
    {
       m_pol.push_back(-1);
       fscanf(f," %lf",&m_pol[i]);
    }

    //Read inverse polynomial coefficients
    fscanf(f,"\n");
    fgets(buf,CMV_MAX_BUF,f);
    fscanf(f,"\n");
    fscanf(f,"%d", &m_len_pol_inv);
    for (i = 0; i < m_len_pol_inv; i++)
    {
       m_pol_inv.push_back(-1);
       fscanf(f," %lf",&m_pol_inv[i]);
    }

    //Read center coordinates
    fscanf(f,"\n");
    fgets(buf,CMV_MAX_BUF,f);
    fscanf(f,"\n");
    fscanf(f,"%lf %lf\n", &m_xc, &m_yc);

    //Read affine coefficients
    fgets(buf,CMV_MAX_BUF,f);
    fscanf(f,"\n");
    fscanf(f,"%lf %lf %lf\n", &m_c,&m_d, &m_e);

    //Read image size
    fgets(buf,CMV_MAX_BUF,f);
    fscanf(f,"\n");
    fscanf(f,"%d %d", &m_height, &m_width);

    fclose(f);
    return 0;
}

//------------------------------------------------------------------------------
void OCamCalibModel::cam2world(double point3D[3], double point2D[2])
{
    excoordinate2D(point2D);

    double invdet  = 1/(m_c-m_d*m_e); // 1/det(A), where A = [c,d;e,1] as in the Matlab file

    double xp = invdet*(    (point2D[0] - m_xc) - m_d*(point2D[1] - m_yc) );
    double yp = invdet*( -m_e*(point2D[0] - m_xc) + m_c*(point2D[1] - m_yc) );

    double r   = sqrt(  xp*xp + yp*yp ); //distance [pixels] of  the point from the image center
    double zp  = m_pol[0];
    double r_i = 1;

    for (size_t i = 1; i < m_len_pol; i++)
    {
     r_i *= r;
     zp  += r_i * m_pol[i];
    }

    //normalize to unit norm
    double invnorm = 1/sqrt( xp*xp + yp*yp + zp*zp );

    point3D[0] = invnorm*xp;
    point3D[1] = invnorm*yp; 
    point3D[2] = invnorm*zp;

    excoordinate3D(point3D);
}

// template class OCamCalibModel::world2cam<float>;
// template class OCamCalibModel::world2cam<double>;

/*
 * Project a image point onto the unit sphere
 * input:   Ms: image points
 * output:  Ps: object points in camera frame 
 * Adapted from camera model of OCamCalib
 */
bool OCamCalibModel::cam2world_unitfocal(cv::Point2f &Ms,
                          cv::Point3f &Ps)const{
    // change coordinate system to OCamCalib Model
    excoordinate2D(Ms);
    // Eigen::Matrix2d A;
    // A << m_c, m_d, m_e, 1;
    // Eigen::Matrix2d A_inv = A.inverse();
    // Eigen::Vector2d C;
    // C << Ms.x - m_xc, Ms.y - m_yc;

    // Eigen::Vector2d m = A_inv * C;
    // double rho = sqrt( m(0)*m(0) + m(1)*m(1) );

    // double f = 0;
   
    // for (size_t i=0; i<m_pol.size(); i++){
    //     f += m_pol[i] * pow(rho, i); 
    // }
   
    // Ps.x = m(0) / f;
    // Ps.y = m(1) / f;
    // Ps.z = 1;

    bool pt_behind_cam;

    double invdet  = 1/(m_c-m_d*m_e); // 1/det(A), where A = [c,d;e,1] as in the Matlab file

    double xp = invdet*(    (Ms.x - m_xc) - m_d*(Ms.y - m_yc) );
    double yp = invdet*( -m_e*(Ms.x - m_xc) + m_c*(Ms.y - m_yc) );

    double r   = sqrt(  xp*xp + yp*yp ); //distance [pixels] of  the point from the image center
    double zp  = m_pol[0];
    double r_i = 1;
    int i;

    for (i = 1; i < m_pol.size(); i++)
    {
     r_i *= r;
     zp  += r_i*m_pol[i];
    }

    //normalize to unit norm
    double invnorm = 1/sqrt( xp*xp + yp*yp + zp*zp );

    Ps.x = invnorm*xp;
    Ps.y = invnorm*yp; 
    Ps.z = invnorm*zp;

    if (Ps.z > 0 ){
        Ps.x /= Ps.z;
        Ps.y /= Ps.z; 
        pt_behind_cam = true;
    }
    else{
        Ps.x /= -Ps.z;
        Ps.y /= -Ps.z;
        pt_behind_cam  = false;
    }

    // change back to OmniModel
    excoordinate3D(Ps);

    return pt_behind_cam;
}

bool OCamCalibModel::findCamPose( std::vector<cv::Point2f> Ms, 
                                  std::vector<cv::Point3f> Ps,
                                  Eigen::Matrix4d &  out_T_t_c) const{
    std::vector<cv::Point2f> Ms_behind, Ms_front;
    std::vector<cv::Point3f> Ps_behind, Ps_front;
    for (size_t i = 0; i < Ms.size(); ++i) {
        cv::Point3f undistortPt;

        std::cout << "-----" << std::endl;
        std::cout << Ms[i].x - m_xc << " " << Ms[i].y - m_yc << std::endl;
        bool pt_behind_cam = cam2world_unitfocal(Ms[i], undistortPt);
        Ms[i].x = undistortPt.x;
        Ms[i].y = undistortPt.y;

        if (pt_behind_cam){
            Ms_behind.push_back(Ms[i]);
            Ps_behind.push_back(Ps[i]);
        }
        else{
            Ms_front.push_back(Ms[i]);
            Ps_front.push_back(Ps[i]);
        }
    }

  std::vector<double> distCoeffs(4, 0.0);

  cv::Mat rvec(3, 1, CV_64F);
  cv::Mat tvec(3, 1, CV_64F);

  if (Ps_front.size() < 4) {
  // SM_DEBUG_STREAM(
  // "At least 4 points are needed for calling PnP. Found " << Ps.size());
    // std::cout << Ps.size() << " points found!" << endl;
    // std::cout << "At least 4 points are needed for calling PnP. Found" << std::endl;
    return false;
  }

  // cv::solvePnP(Ps, Ms, cv::Mat::eye(3, 3, CV_64F), distCoeffs, rvec, tvec);
  // cv::solvePnPRansac(Ps, Ms, cv::Mat::eye(3, 3, CV_64F), distCoeffs, rvec, tvec);
  std::cout << Ms_behind.size() << " points behind camera." << std::endl;
  std::cout << Ms_front.size() << " points in front of camera." << std::endl;

  if (Ms_behind.size() > Ms_front.size()){
      cv::solvePnPRansac(Ps_behind, Ms_behind, cv::Mat::eye(3, 3, CV_64F), distCoeffs, rvec, tvec);
      // tvec.at<double>(2, 0) = -tvec.at<double>(2, 0);
  }
  else{
      cv::solvePnPRansac(Ps_front, Ms_front, cv::Mat::eye(3, 3, CV_64F), distCoeffs, rvec, tvec);
  }

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

  if (Ms_behind.size() > Ms_front.size()){
      Eigen::Matrix4d reflection;
      reflection << 1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, -1, 0,
                    0, 0, 0, 1;
      T_camera_model = reflection * T_camera_model;
  }
  out_T_t_c = T_camera_model; // object pose in camera frame
  return true;
}
