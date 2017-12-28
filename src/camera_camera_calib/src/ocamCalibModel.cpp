#include <math.h>

//#include <gsl/gsl_poly.h>
#include <stdlib.h>
#include <stdio.h>

#include <opengv/absolute_pose/methods.hpp>
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/math/cayley.hpp>


// #include <gsl/gsl_poly.h>


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

// void OCamCalibModel::setParameter(int width, int height,W
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
                          cv::Point3f &Ps, bool &isback)const{
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

    //if (Ps.z > 0 && Ps.z < 0.6) return false;

    if (Ps.z > 0 ){
        Ps.x /= Ps.z;
        Ps.y /= Ps.z; 
        isback = true;
    }
    else{
        Ps.x /= -Ps.z;
        Ps.y /= -Ps.z;
        isback  = false;
    }

    // change back to OmniModel
    excoordinate3D(Ps);

    return true;
}

bool OCamCalibModel::findCamPose( std::vector<cv::Point2f> Ms, 
                                  std::vector<cv::Point3f> Ps,
                                  Eigen::Matrix4d &  out_T_t_c) const{
    std::vector<cv::Point2f> Ms_back, Ms_front;
    std::vector<cv::Point3f> Ps_behind, Ps_front;
    for (size_t i = 0; i < Ms.size(); ++i) {
        cv::Point3f undistortPt;

        bool isback;
        bool valid = cam2world_unitfocal(Ms[i], undistortPt, isback);
        Ms[i].x = undistortPt.x;
        Ms[i].y = undistortPt.y;
        if (!valid) continue;
        if (isback){
            Ms_back.push_back(Ms[i]);
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

  if (Ps_front.size() < 4 && Ps_behind.size() < 4) {
  // SM_DEBUG_STREAM(
  // "At least 4 points are needed for calling PnP. Found " << Ps.size());
    // std::cout << Ps.size() << " points found!" << endl;
    // std::cout << "At least 4 points are needed for calling PnP. Found" << std::endl;
    return false;
  }

  // cv::solvePnP(Ps, Ms, cv::Mat::eye(3, 3, CV_64F), distCoeffs, rvec, tvec);
  // cv::solvePnPRansac(Ps, Ms, cv::Mat::eye(3, 3, CV_64F), distCoeffs, rvec, tvec);

  if (Ms_back.size() > Ms_front.size()){
      cv::solvePnPRansac(Ps_behind, Ms_back, cv::Mat::eye(3, 3, CV_64F), distCoeffs, rvec, tvec);
      // cv::solvePnP(Ps_behind, Ms_back, cv::Mat::eye(3, 3, CV_64F), distCoeffs, rvec, tvec);
  }
  else{
      cv::solvePnPRansac(Ps_front, Ms_front, cv::Mat::eye(3, 3, CV_64F), distCoeffs, rvec, tvec);
      // cv::solvePnP(Ps_front, Ms_front, cv::Mat::eye(3, 3, CV_64F), distCoeffs, rvec, tvec);
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

  if (Ms_back.size() > Ms_front.size()){
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

bool OCamCalibModel::findCamPose( std::vector<cv::Point2f> Ms, 
                                  std::vector<cv::Point3f> Ps,
                                  cv::Mat &rvec,
                                  cv::Mat &tvec) const{
    std::vector<cv::Point2f> Ms_back, Ms_front;
    std::vector<cv::Point3f> Ps_behind, Ps_front;
    for (size_t i = 0; i < Ms.size(); ++i) {
        cv::Point3f undistortPt;

        bool isback;
        bool valid = cam2world_unitfocal(Ms[i], undistortPt, isback);
        Ms[i].x = undistortPt.x;
        Ms[i].y = undistortPt.y;
        if (!valid) continue;
        if (isback){
            Ms_back.push_back(Ms[i]);
            Ps_behind.push_back(Ps[i]);
        }
        else{
            Ms_front.push_back(Ms[i]);
            Ps_front.push_back(Ps[i]);
        }
    }

  std::vector<double> distCoeffs(4, 0.0);

  if (Ps_front.size() < 4 && Ps_behind.size() < 4) {
  // SM_DEBUG_STREAM(
  // "At least 4 points are needed for calling PnP. Found " << Ps.size());
    // std::cout << Ps.size() << " points found!" << endl;
    // std::cout << "At least 4 points are needed for calling PnP. Found" << std::endl;
    return false;
  }

  // cv::solvePnP(Ps, Ms, cv::Mat::eye(3, 3, CV_64F), distCoeffs, rvec, tvec);
  // cv::solvePnPRansac(Ps, Ms, cv::Mat::eye(3, 3, CV_64F), distCoeffs, rvec, tvec);

  if (Ms_back.size() > Ms_front.size()){
      cv::solvePnPRansac(Ps_behind, Ms_back, cv::Mat::eye(3, 3, CV_64F), distCoeffs, rvec, tvec);
      // cv::solvePnP(Ps_behind, Ms_back, cv::Mat::eye(3, 3, CV_64F), distCoeffs, rvec, tvec);
  }
  else{
      cv::solvePnPRansac(Ps_front, Ms_front, cv::Mat::eye(3, 3, CV_64F), distCoeffs, rvec, tvec);
      // cv::solvePnP(Ps_front, Ms_front, cv::Mat::eye(3, 3, CV_64F), distCoeffs, rvec, tvec);
  }

 
  // std::cout << "before" << std::endl;
  // std::cout << rvec.at<double>(0,0) << " " << rvec.at<double>(1,0) << " " << rvec.at<double>(2,0);
  // std::cout<< std::endl;
  // std::cout << tvec.at<double>(0,0) << " " << tvec.at<double>(1,0) << " " << tvec.at<double>(2,0);
  // std::cout<< std::endl;

  // Eigen::Matrix4d T_camera_model;
  // transformVec2Mat(rvec, tvec, T_camera_model);
  // if (Ms_back.size() > Ms_front.size()){
  //     std::cout << "back" << std::endl; 
  //     std::cout << T_camera_model << std::endl;

  //     Eigen::Matrix4d reflection;
  //     reflection << 1, 0, 0, 0,
  //                   0, 1, 0, 0,
  //                   0, 0, -1, 0,
  //                   0, 0, 0, 1;
  //     T_camera_model = reflection * T_camera_model;
  //     std::cout<< T_camera_model << std::endl;
  // }

  // transformMat2Vec(T_camera_model, rvec, tvec);
  // std::cout << "after" << std::endl;
  // std::cout << rvec.at<double>(0,0) << " " << rvec.at<double>(1,0) << " " << rvec.at<double>(2,0);
  // std::cout<< std::endl;
  // std::cout << tvec.at<double>(0,0) << " " << tvec.at<double>(1,0) << " " << tvec.at<double>(2,0);
  // std::cout<< std::endl;


  // transformVec2Mat(rvec, tvec, T_camera_model);
  // std::cout << "after after" << std::endl;
  // std::cout << T_camera_model;
  // std::cout<< std::endl;
  

  return true;
}


/*
 * Analytical solution given in Scaramuzza, 2006
 * 
 * Input: 
 *      scene points Ps (in target board frame)
 *      image points Ms (in image frame, pixels, 
                        origin is at the center of image)
 * Output:
 *      rvec: rotation vector
 *      tvec: translation vector
 *
 */
bool OCamCalibModel::solveCamPose( std::vector<cv::Point2f> Ms, 
                                  std::vector<cv::Point3f> Ps,
                                  cv::Mat &rvec,
                                  cv::Mat &tvec) const{

    for (size_t i = 0; i < Ms.size(); ++i) {
        cv::Point3f undistortPt;

        bool isback;
        bool valid = cam2world_unitfocal(Ms[i], undistortPt, isback);
        Ms[i].x = undistortPt.x;
        Ms[i].y = undistortPt.y;
    }

    std::vector<Eigen::Matrix<float, 3, 4> > Rt_set;   // Rt = [r1 r2 t]

    bool flag = findExtrinsic(Ms, Ps, Rt_set); 

    for (size_t i=0; i<Rt_set.size(); i++){
        std::cout << Rt_set[i] << std::endl;
    }
    
    // Eigen::Vector3f r3 = r1.cross(r2);

    // Eigen::Matrix3f intermediate;
    // intermediate.block<3,1>(0,0)  = r1;
    // intermediate.block<3,1>(0,1) = r2;
    // intermediate.block<3,1>(0,2) = r3;
    // Eigen::JacobiSVD<Eigen::Matrix3f> svd2(intermediate, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // std::cout << svd2.matrixU() * svd2.matrixV().transpose() << std::endl; 
    

    return true;
}

bool OCamCalibModel::findExtrinsic(std::vector<cv::Point2f> Ms, 
                                  std::vector<cv::Point3f> Ps,
                                  std::vector<Eigen::Matrix<float, 3, 4> > &Rt_set) const{
    Eigen::MatrixXf A;
    std::vector<float> entries;
    int rows = Ms.size(), cols = 6;
    for (size_t i=0; i<rows; i++) {
        entries.push_back(-Ms[i].y * Ps[i].x); 
        entries.push_back(-Ms[i].y * Ps[i].y); 
        entries.push_back( Ms[i].x * Ps[i].x); 
        entries.push_back( Ms[i].x * Ps[i].y); 
        entries.push_back(-Ms[i].y); 
        entries.push_back( Ms[i].x); 
    }

    A = Eigen::Map< Eigen::Matrix<float, 
              Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >
              (&entries[0], rows, cols);

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeFullV);
    // std::cout << "Its left singular values are:" << std::endl
    //           << svd.singularValues() << std::endl;         
    // std::cout << "Its left singular vectors are the"
    //              " columns of the thin U matrix:" 
    //           << std::endl << svd.matrixU() << std::endl;
    // std::cout << "Its right singular vectors are the"
    //           << " columns of the thin V matrix:" 
    //           << std::endl << svd.matrixV() << std::endl;
    
    /* solve Ax = 0
     *    use SVD decomposition
     *    the null space of A is the eigenvector 
     *        corresponding to the smallest eigenvalue 
     */
    Eigen::VectorXf x = svd.matrixV().block<6, 1>(0, 5);

    float alpha = x(0, 0) * x(1, 0) + x(2, 0) * x(3, 0);  // r11 * r12 + r21 * r22
    float gamma = x(0, 0) * x(0, 0) + x(2, 0) * x(2, 0);  // r11^2 + r21^2
    float beta = gamma
              - (x(1, 0) * x(1, 0) + x(3, 0) * x(3, 0));  // r11^2 + r21^2 - (r12^2+r22^2)

    float r31_s, r31;        // r31_s = r31 ^ 2
    float r32_s, r32;   // r32_s = r32 ^ 2
    float scale_abs;

    if (std::abs(alpha) < 1e-5){
        r31   = 0;
        r32_s = beta;   // r32 can be positive or negative
        
        // determine the sign of the scale factor
        scale_abs =  std::sqrt(1.0 / gamma); 
        Eigen::Vector2f nRR1, nRR2;
        nRR1 << scale_abs*x(4) - Ms[0].x, scale_abs*x(5) - Ms[0].y; 
        nRR2 << -scale_abs*x(4) - Ms[0].x, -scale_abs*x(5) - Ms[0].y; 
        int sign = nRR1.norm() < nRR2.norm() ? 1 : -1;
        for (size_t i=0; i<2; i++){
            Eigen::Matrix<float, 3, 4> Rt;
            r32 = std::pow(-1, i) * std::sqrt(r32_s);
            
            Rt << x(0), x(1), 0, x(4),
                  x(2), x(3), 0, x(5),
                  r31 ,  r32, 0,    0;
            Rt = Rt * scale_abs * sign;
            
            Rt.block<3,1>(0, 2) =  Eigen::Vector3f(Rt(0,0), Rt(1,0), Rt(2,0)).cross(
                              Eigen::Vector3f(Rt(0,1), Rt(1,1), Rt(2,1)));
            Rt_set.push_back(Rt); 
        }
    }else{ 
        for (size_t i=0; i<2; i++){
            r31_s = (-beta + std::pow(-1, i) *std::sqrt(beta*beta + 4 * alpha)) / 2.0; 
            if (r31_s < 0) continue;

            r31 = std::sqrt(r31_s);
            for (size_t j=0; j<2; j++){  
                r31 = std::pow(-1, j) * r31;
                r32 = -alpha / r31;  

                // determine the sign of scale factor
                scale_abs =  std::sqrt(1.0 / (gamma + r31_s)); 
                Eigen::Vector2f nRR1, nRR2;
                nRR1 << scale_abs*x(4) - Ms[0].x,  scale_abs*x(5) - Ms[0].y; 
                nRR2 << -scale_abs*x(4) - Ms[0].x, -scale_abs*x(5) - Ms[0].y; 
                int sign = nRR1.norm() < nRR2.norm() ? 1 : -1;

                Eigen::Matrix<float, 3, 4> Rt;
                Rt << x(0), x(1), 0, x(4),
                      x(2), x(3), 0, x(5),
                      r31 ,  r32, 0,    0;
                Rt = Rt * scale_abs * sign;
                Rt.block<3,1>(0, 2) =  Eigen::Vector3f(Rt(0,0), Rt(1,0), Rt(2,0)).cross(
                              Eigen::Vector3f(Rt(0,1), Rt(1,1), Rt(2,1)));
                Rt_set.push_back(Rt); 
            }
        }
    }

    return true;
}

// -----------------------------------------------------------------------------
cv::Point3f OCamCalibModel::pointTransform(const cv::Point3f& p0, const Eigen::Matrix4d& transform){
    Eigen::Vector4d eigen_p0;
    eigen_p0 << p0.x, p0.y, p0.z, 1;
    Eigen::Vector4d eigen_p1 = transform * eigen_p0;
    return cv::Point3f(eigen_p1(0), eigen_p1(1), eigen_p1(2));
}

cv::Point2f OCamCalibModel::targetPoint2ImagePixel(const cv::Point3f& p0, const Eigen::Matrix4d& target_pose){
    cv::Point3f p1 = pointTransform(p0, target_pose);

    double Ps[3] = {p1.x, p1.y, p1.z};
    double Ms[2];
    world2cam(Ms, Ps);

    return cv::Point2f(Ms[0], Ms[1]);
}

void OCamCalibModel::transformVec2Mat(const cv::Mat& rvec, const cv::Mat& tvec, 
                                      Eigen::Matrix4d& T_camera_model) const{
    cv::Mat C_camera_model = cv::Mat::eye(3, 3, CV_64F);
    T_camera_model = Eigen::Matrix4d::Identity();
    cv::Rodrigues(rvec, C_camera_model);
    for (int r = 0; r < 3; ++r) {
        T_camera_model(r, 3) = tvec.at<double>(r, 0);
        for (int c = 0; c < 3; ++c) {
            T_camera_model(r, c) = C_camera_model.at<double>(r, c);
        }
    }
}


void OCamCalibModel::transformMat2Vec(const Eigen::Matrix4d& T_camera_model, 
                                                 cv::Mat& rvec, cv::Mat& tvec) const{
    cv::Mat R(3, 3, CV_64F);
    for (size_t i=0; i<3; i++){
        tvec.at<double>(i, 0) =  T_camera_model(i, 3);
        for (size_t j=0; j<3; j++){
            R.at<double>(i, j) = T_camera_model(i, j);
        }
    }
    cv::Rodrigues(R, rvec);
    // rvec = rotationMatrixToEulerAngles(R);

    
    
}


// void OCamCalibModel::solveMLPnP(std::vector<cv::Point3f> Ps, 
//                                 std::vector<cv::Point2f> Ms, 
//                                 cv::Mat, 
//                                 std::vector<double> distCoeffs, 
//                                 cv::Mat rvec, 
//                                 cv::Mat tvec)
// {
//     bearingVectors_t bearingVectors;
//     points_t points;

//     for (size_t i=0; i<Ps.size(); i++){
//         bearingVectors
//     }

//     absolute_pose::CentralAbsoluteAdapter adapter(
//         bearingVectors,
//         points);
//     transformation_t mlpnp_transformation;
//     mlpnp_transformation = absolute_pose::mlpnp(adapter);

// }


// void OCamCalibModel::pointCloudPose(const std::vector<cv::Point3f> &point_cloud_1, const std::vector<cv::Point3f> &point_cloud_2, cv::Mat &R, cv::Mat &t)
// {
//     float p1_x=0, p1_y=0, p1_z=0;
//     float p2_x=0, p2_y=0, p2_z=0;
    
//     const int point_num = index.size();
//     int p_index;
//     for (int i=0; i<point_num; i++){
//         p_index = index[i];
//         p1_x += point_cloud_1[p_index].x;
//         p1_y += point_cloud_1[p_index].y;
//         p1_z += point_cloud_1[p_index].z;
        
//         p2_x += point_cloud_2[p_index].x;
//         p2_y += point_cloud_2[p_index].y;
//         p2_z += point_cloud_2[p_index].z;
//     }
    
//     // centroids of two point clouds
//     p1_x /= point_num;
//     p1_y /= point_num;
//     p1_z /= point_num;
    
//     p2_x /= point_num;
//     p2_y /= point_num;
//     p2_z /= point_num;
    
//     Mat H = cv::Mat::zeros(3, 3, CV_32F);
//     for (int i=0; i<point_num; i++){
//         p_index = index[i];
        
//         Point3f p1 = point_cloud_1[p_index] - Point3f(p1_x, p1_y, p1_z);
//         Point3f p2 = point_cloud_2[p_index] - Point3f(p2_x, p2_y, p2_z);
        
//         H.at<float>(0,0) += p1.x * p2.x;
//         H.at<float>(0,1) += p1.x * p2.y;
//         H.at<float>(0,2) += p1.x * p2.z;
        
//         H.at<float>(1,0) += p1.y * p2.x;
//         H.at<float>(1,1) += p1.y * p2.y;
//         H.at<float>(1,2) += p1.y * p2.z;
        
//         H.at<float>(2,0) += p1.z * p2.x;
//         H.at<float>(2,1) += p1.z * p2.y;
//         H.at<float>(2,2) += p1.z * p2.z;
        
//     }
    
//     Mat e, U, V, U_transpose;
//     cv::SVDecomp(H, e, U, V, cv::SVD::FULL_UV);
//     transpose(U, U_transpose);
//     R = V * U_transpose;
    
//     t.at<float>(0) = p2_x - R.at<float>(0,0)*p1_x + R.at<float>(0,1)*p1_y + R.at<float>(0,2)*p1_z;
//     t.at<float>(1) = p2_y - R.at<float>(1,0)*p1_x + R.at<float>(1,1)*p1_y + R.at<float>(1,2)*p1_z;
//     t.at<float>(2) = p2_z - R.at<float>(2,0)*p1_x + R.at<float>(2,1)*p1_y + R.at<float>(2,2)*p1_z;
    
// }


