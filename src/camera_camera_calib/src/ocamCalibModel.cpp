#include <math.h>

//#include <gsl/gsl_poly.h>
#include <stdlib.h>
#include <stdio.h>

#include <opengv/absolute_pose/methods.hpp>
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/math/cayley.hpp>


#include "camera_camera_calib/ocamCalibModel.h" 

/*
 * some are directly from OCamCalib
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



/* *******************************************************************
 * 
 * get_ocam_model
 *
 * *******************************************************************/
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




/* *******************************************************************
 * 
 * cam2world
 *
 * *******************************************************************/
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



/* *******************************************************************
 *
 * Project a image point onto the unit sphere
 * input:   Ms: image points
 * output:  Ps: object points in camera frame 
 * Adapted from camera model of OCamCalib
 *
 * *******************************************************************
 */
template <typename T>
bool OCamCalibModel::cam2world_unitfocal(cv::Point_<T> &Ms, 
                                        cv::Point3_<T> &Ps, 
                                        bool &isback)const{
    // change coordinate system to OCamCalib Model
    excoordinate2D(Ms);

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
template bool OCamCalibModel::cam2world_unitfocal<float>(
                                  cv::Point_<float> &Ms,
                                  cv::Point3_<float> &Ps, 
                                  bool &isback)const;
template bool OCamCalibModel::cam2world_unitfocal<double>(
                                  cv::Point_<double> &Ms,
                                  cv::Point3_<double> &Ps, 
                                  bool &isback)const;





/* *******************************************************************
 * 
 * findCamPose
 *
 * *******************************************************************/
template <typename T>
bool OCamCalibModel::findCamPose( std::vector<cv::Point_<T>> Ms, 
                                  std::vector<cv::Point3_<T>> Ps,
                                  Eigen::Matrix4d &  out_T_t_c) const{
    
    cv::Mat rvec(3, 1, CV_64F);
    cv::Mat tvec(3, 1, CV_64F);
    Eigen::Matrix4d T_camera_model;
    bool isback;  // if the target is likely behind the camera

    pnpPose(Ms, Ps, rvec, tvec, isback);

    transformVec2Mat(rvec, tvec, T_camera_model);

    if (isback){
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
template bool OCamCalibModel::findCamPose<float>( 
                                  std::vector<cv::Point_<float>> Ms, 
                                  std::vector<cv::Point3_<float>> Ps,
                                  Eigen::Matrix4d &  out_T_t_c) const;
template bool OCamCalibModel::findCamPose<double>( 
                                  std::vector<cv::Point_<double>> Ms, 
                                  std::vector<cv::Point3_<double>> Ps,
                                  Eigen::Matrix4d &  out_T_t_c) const;


/* *******************************************************************
 * 
 * findCamPose
 *
 * *******************************************************************/
template <typename T>
bool OCamCalibModel::findCamPose( std::vector<cv::Point_<T>> Ms, 
                                  std::vector<cv::Point3_<T>> Ps,
                                  cv::Mat &rvec,
                                  cv::Mat &tvec) const{
    bool isback;
    pnpPose(Ms, Ps, rvec, tvec, isback);

    
    if (isback){
        rvec.at<double>(0,0) = -rvec.at<double>(0,0);
        rvec.at<double>(1,0) = -rvec.at<double>(1,0);
        tvec.at<double>(2,0) = -tvec.at<double>(2,0);
   
    }
  

    return true;
}
template bool OCamCalibModel::findCamPose<float>( 
                                  std::vector<cv::Point_<float>> Ms, 
                                  std::vector<cv::Point3_<float>> Ps,
                                  cv::Mat &rvec,
                                  cv::Mat &tvec) const;
template bool OCamCalibModel::findCamPose<double>( 
                                  std::vector<cv::Point_<double>> Ms, 
                                  std::vector<cv::Point3_<double>> Ps,
                                  cv::Mat &rvec,
                                  cv::Mat &tvec) const;


/* *******************************************************************
 * 
 * pnpPose
 *
 * private function, find transform (R, t) between 
 *    camera frame (in meters) and target frame (in meters) 
 *
 * *******************************************************************/
template <typename T>
bool OCamCalibModel::pnpPose( std::vector<cv::Point_<T>> Ms, 
                                  std::vector<cv::Point3_<T>> Ps,
                                  cv::Mat &rvec,
                                  cv::Mat &tvec,
                                  bool &isBoardBehindCamera) const{
    if (Ps.size() <= 0) return false;

    std::vector<cv::Point_<T> > Ms_back, Ms_front;
    std::vector<cv::Point3_<T> > Ps_back, Ps_front;
    for (size_t i = 0; i < Ms.size(); ++i) {
        cv::Point3_<T> undistortPt;

        bool isback;
        bool valid = cam2world_unitfocal(Ms[i], undistortPt, isback);
        Ms[i].x = undistortPt.x;
        Ms[i].y = undistortPt.y;
        if (!valid) continue;
        if (isback){
            Ms_back.push_back(Ms[i]);
            Ps_back.push_back(Ps[i]);
        }
        else{
            Ms_front.push_back(Ms[i]);
            Ps_front.push_back(Ps[i]);
        }
    }

    std::vector<double> distCoeffs(4, 0.0);

    
    std::cout << Ps_front.size() << " " << Ps_back.size() << std::endl;
    std::cout << Ps[0].x << " " << Ps[1].y << std::endl;
  
    if (Ps_front.size() < 4 && Ps_back.size() < 4) {
        // At least 4 points are needed for calling PnP
        return false;
    }

    if (Ms_back.size() > Ms_front.size()){
        isBoardBehindCamera = true;
        cv::solvePnPRansac(Ps_back, Ms_back, 
                           cv::Mat::eye(3, 3, CV_64F), 
                           distCoeffs, rvec, tvec);
        // cv::solvePnP is not as accurate as solvePnPRansac
    }
    else{
        isBoardBehindCamera = false;
        cv::solvePnPRansac(Ps_front, Ms_front, 
                           cv::Mat::eye(3, 3, CV_64F), 
                           distCoeffs, rvec, tvec);
    }

    

    return true;
}

template bool OCamCalibModel::pnpPose<float>( 
                                  std::vector<cv::Point_<float>> Ms, 
                                  std::vector<cv::Point3_<float>> Ps,
                                  cv::Mat &rvec,
                                  cv::Mat &tvec,
                                  bool &isback) const;
template bool OCamCalibModel::pnpPose<double>( 
                                  std::vector<cv::Point_<double>> Ms, 
                                  std::vector<cv::Point3_<double>> Ps,
                                  cv::Mat &rvec,
                                  cv::Mat &tvec,
                                  bool &isback) const;

/* *******************************************************************
 *
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
 * *******************************************************************/
template <typename T>
bool OCamCalibModel::solveAnalyticalSol( 
                        const std::vector<cv::Point_<T> > Ms_abs, 
                        const std::vector<cv::Point3_<T> > Ps,
                        const T xc, const T yc,
                        cv::Mat &rvec,
                        cv::Mat &tvec) const{

    // for (size_t i = 0; i < Ms.size(); ++i) {
    //     cv::Point3f undistortPt;

    //     bool isback;
    //     bool valid = cam2world_unitfocal(Ms[i], undistortPt, isback);
    //     Ms[i].x = undistortPt.x;
    //     Ms[i].y = undistortPt.y;
    // }

    int taylor_order = 5;
    std::vector<cv::Point_<T> > Ms = Ms_abs;
    for (size_t i=0; i < Ms.size(); ++i){
        Ms[i].x = Ms_abs[i].x - xc; //512.560101; //532.425687;
        Ms[i].y = Ms_abs[i].y - yc; //523.645938; //517.382409;
    }

    std::vector<Eigen::Matrix<T, 3, 4> > Rt_set;   // Rt = [r1 r2 t]

    bool flag = findExtrinsic(Ms, Ps, Rt_set); 

    for (size_t i=0; i<Rt_set.size(); i++){
        std::cout << Rt_set[i] << std::endl;
    }
    
    size_t num_pt = Ms.size();
    for (size_t i=0; i<Rt_set.size(); i++){
        std::vector<Eigen::Matrix<T, 3, 4> > pose_set;
        pose_set.push_back(Rt_set[i]);

        std::vector<std::vector<cv::Point_<T> > > Ms_set;
        std::vector<std::vector<cv::Point3_<T> > > Ps_set;

        Ms_set.push_back(Ms);
        Ps_set.push_back(Ps);
        findIntrinsic(Ms_set, Ps_set, pose_set, xc, yc, taylor_order, num_pt);
    }
    

    return true;
}
template bool OCamCalibModel::solveAnalyticalSol<float>(
                                  std::vector<cv::Point_<float> > Ms, 
                                  std::vector<cv::Point3_<float> > Ps,
                                  const float xc, const float yc,
                                  cv::Mat &rvec,
                                  cv::Mat &tvec) const;

template bool OCamCalibModel::solveAnalyticalSol<double>(
                                  std::vector<cv::Point_<double> > Ms, 
                                  std::vector<cv::Point3_<double> > Ps,
                                  const double xc, const double yc,
                                  cv::Mat &rvec,
                                  cv::Mat &tvec) const;


/* *******************************************************************
 * 
 * findExtrinsic
 *
 * *******************************************************************/
template <typename T>
bool OCamCalibModel::findExtrinsic(
              std::vector<cv::Point_<T> > Ms, 
              std::vector<cv::Point3_<T> > Ps,
              std::vector<Eigen::Matrix<T, 3, 4> > &Rt_set) const{
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> A;
    std::vector<T> entries;
    int rows = Ms.size(), cols = 6;
    for (size_t i=0; i<rows; i++) {
        entries.push_back(-Ms[i].y * Ps[i].x); 
        entries.push_back(-Ms[i].y * Ps[i].y); 
        entries.push_back( Ms[i].x * Ps[i].x); 
        entries.push_back( Ms[i].x * Ps[i].y); 
        entries.push_back(-Ms[i].y); 
        entries.push_back( Ms[i].x); 
    }

    A = Eigen::Map< Eigen::Matrix<T, 
              Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >
              (&entries[0], rows, cols);

    Eigen::JacobiSVD<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> > svd(
                                A, Eigen::ComputeThinU | Eigen::ComputeFullV);


    double cond = svd.singularValues()(0) 
    / svd.singularValues()(svd.singularValues().size()-1);
    std::cout << "Condition number of matrix A: ";
    std::cout << cond << std::endl;
    
    // /* solve Ax = 0
    //  *    use SVD decomposition
    //  *    the null space of A is the eigenvector 
    //  *        corresponding to the smallest eigenvalue 
    //  */
    
    Eigen::Matrix<T, 6, 1> x = svd.matrixV().template block<6,1>(0,5);
    
    T alpha = x(0) * x(1) + x(2) * x(3);  // r11 * r12 + r21 * r22
    T gamma = x(0) * x(0) + x(2) * x(2);  // r11^2 + r21^2
    T beta = gamma
              - (x(1) * x(1) + x(3) * x(3));  // r11^2 + r21^2 - (r12^2+r22^2)

    T r31_s, r31;        // r31_s = r31 ^ 2
    T r32_s, r32;   // r32_s = r32 ^ 2
    T scale_abs;

    T sqrt_det = std::sqrt(beta*beta + 4 * alpha * alpha);
    if (std::abs(alpha) < 1e-9){
        r31   = 0;
        r32_s = beta;   // r32 can be positive or negative
   
        // determine the sign of the scale factor
        scale_abs =  std::sqrt(1.0 / gamma); 
        Eigen::Vector2f nRR1, nRR2;
        nRR1 << scale_abs*x(4) - Ms[0].x, scale_abs*x(5) - Ms[0].y; 
        nRR2 << -scale_abs*x(4) - Ms[0].x, -scale_abs*x(5) - Ms[0].y; 
        int sign = nRR1.norm() < nRR2.norm() ? 1 : -1;

        for (size_t j=0; j<2; j++){
            Eigen::Matrix<T, 3, 4> Rt;
            r32 = std::pow(-1, j) * std::sqrt(r32_s);
            
            Rt << x(0), x(1), 0, x(4),
                  x(2), x(3), 0, x(5),
                  r31 ,  r32, 0,    0;
            Rt = Rt * scale_abs * sign;
            
            Rt.template block<3,1>(0, 2) =  
                      Eigen::Matrix<T, 3, 1>(Rt(0,0), Rt(1,0), Rt(2,0)).cross(
                      Eigen::Matrix<T, 3, 1>(Rt(0,1), Rt(1,1), Rt(2,1)));
            Rt_set.push_back(Rt); 
        }
        
    }else{ 
        for (size_t i=0; i<2; i++){
            r31_s = (-beta + std::pow(-1, i) * sqrt_det) / 2.0; 
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

                Eigen::Matrix<T, 3, 4> Rt;
                Rt << x(0), x(1), 0, x(4),
                      x(2), x(3), 0, x(5),
                      r31 ,  r32, 0,    0;
                Rt = Rt * scale_abs * sign;
                Rt.template block<3,1>(0, 2) =  
                      Eigen::Matrix<T, 3, 1>(Rt(0,0), Rt(1,0), Rt(2,0)).cross(
                      Eigen::Matrix<T, 3, 1>(Rt(0,1), Rt(1,1), Rt(2,1)));
                Rt_set.push_back(Rt); 
            }
        }
    }

    return true;
}
template bool OCamCalibModel::findExtrinsic<float>(
              std::vector<cv::Point_<float> > Ms, 
              std::vector<cv::Point3_<float> > Ps,
              std::vector<Eigen::Matrix<float, 3, 4> > &Rt_set) const;
template bool OCamCalibModel::findExtrinsic<double>(
              std::vector<cv::Point_<double> > Ms, 
              std::vector<cv::Point3_<double> > Ps,
              std::vector<Eigen::Matrix<double, 3, 4> > &Rt_set) const;



/* *******************************************************************
 *
 * Solve camera intrinsic parameter analytically
 * 
 * Input: 
 *      scene points Ps (in target board frame)
 *      image points Ms (in image frame, pixels, 
 *                      origin is at the center of image)
 *
 *
 *      partial extrinsic parameters obtained in findExtrinsic
 *       
 * Output:
 *      
 *        
 *
 * *******************************************************************/
template <typename T>
bool OCamCalibModel::findIntrinsic(
              const std::vector<std::vector<cv::Point_<T> > > Ms, 
              const std::vector<std::vector<cv::Point3_<T> > > Ps,
              std::vector<Eigen::Matrix<T, 3, 4> > &Rt_set,
              const T xc, const T yc, 
              const int taylor_order,
              const size_t num_pt) const{
    // here, Ms is the image points, and origin is at the center
    std::vector<T> entries;
    std::vector<T> right_col;
    size_t num_img=Rt_set.size();
    for (size_t i=0; i<Ms.size(); i++){
        T R11=Rt_set[i](0,0);
        T R21=Rt_set[i](1,0);
        T R31=Rt_set[i](2,0);
        T R12=Rt_set[i](0,1);
        T R22=Rt_set[i](1,1);
        T R32=Rt_set[i](2,1);
        T T1=Rt_set[i](0,3);
        T T2=Rt_set[i](1,3);

        for (size_t j=0; j<Ms[i].size(); j++){
          

          T Xt = Ps[i][j].x;
          T Yt = Ps[i][j].y;
          T Xpt = Ms[i][j].x;
          T Ypt = Ms[i][j].y;
          T MA= R21 * Xt + R22 * Yt + T2;
          T MB= Ypt * ( R31 * Xt + R32 * Yt );
          T MC= R11 * Xt + R12 * Yt + T1;
          T MD= Xpt * ( R31 * Xt + R32 * Yt ); 

          T rho_sq = Ms[i][j].x * Ms[i][j].x + Ms[i][j].y * Ms[i][j].y;
          T rho = std::sqrt(rho_sq); entries.push_back(MA);
          T tmp = MA * rho_sq; entries.push_back(tmp);
          // first row
          for (size_t k=2; k<taylor_order; k++){
              tmp *= rho;
              entries.push_back(tmp);
          }

          for (size_t k=0; k<i; k++){
              entries.push_back(0);
          }

          entries.push_back(-Ms[i][j].y);

          for (size_t k=taylor_order+i+1; k<taylor_order+num_img; k++){
              entries.push_back(0);
          }

          // second row
          entries.push_back(MC);
          tmp = MC * rho_sq; entries.push_back(tmp);
          for (size_t k=2; k<taylor_order; k++){
              tmp *= rho;
              entries.push_back(tmp);
          }

          for (size_t k=0; k<i; k++){
              entries.push_back(0);
          }

          entries.push_back(-Ms[i][j].x);

          for (size_t k=taylor_order+i+1; k<taylor_order+num_img; k++){
              entries.push_back(0);
          }

          right_col.push_back(MB);
          right_col.push_back(MD);

        }
    }
   
    size_t rows = 2*num_pt, cols = num_img + taylor_order;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> A;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> b;
    A = Eigen::Map< Eigen::Matrix<T, 
              Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >
              (&entries[0], rows, cols);
    b = Eigen::Map< Eigen::Matrix<T, 
              Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >
              (&right_col[0], rows, 1);

    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> x;
    x = A.colPivHouseholderQr().solve(b);

    std::cout << A;
    std::cout << b;
    std::cout << x << std::endl;

    return 0;    
}
template bool OCamCalibModel::findIntrinsic<float>(
              std::vector<std::vector<cv::Point_<float> > > Ms, 
              std::vector<std::vector<cv::Point3_<float> > > Ps,
              std::vector<Eigen::Matrix<float, 3, 4> > &Rt_set,
              const float xc, const float yc, 
              const int taylor_order,
              const size_t num_pt) const;
template bool OCamCalibModel::findIntrinsic<double>(
              std::vector<std::vector<cv::Point_<double> > > Ms, 
              std::vector<std::vector<cv::Point3_<double> > > Ps,
              std::vector<Eigen::Matrix<double, 3, 4> > &Rt_set,
              const double xc, const double yc, 
              const int taylor_order,
              const size_t num_pt) const;




/* *******************************************************************
 *
 * pointTransform
 *
 * *******************************************************************/
cv::Point3f OCamCalibModel::pointTransform(const cv::Point3f& p0, const Eigen::Matrix4d& transform){
    Eigen::Vector4d eigen_p0;
    eigen_p0 << p0.x, p0.y, p0.z, 1;
    Eigen::Vector4d eigen_p1 = transform * eigen_p0;
    return cv::Point3f(eigen_p1(0), eigen_p1(1), eigen_p1(2));
}


/* *******************************************************************
 *
 * targetPoint2ImagePixel
 *
 * *******************************************************************/
cv::Point2f OCamCalibModel::targetPoint2ImagePixel(const cv::Point3f& p0, const Eigen::Matrix4d& target_pose){
    cv::Point3f p1 = pointTransform(p0, target_pose);

    double Ps[3] = {p1.x, p1.y, p1.z};
    double Ms[2];
    world2cam(Ms, Ps);

    return cv::Point2f(Ms[0], Ms[1]);
}


/* *******************************************************************
 *
 * transformVec2Mat
 *
 * *******************************************************************/
void OCamCalibModel::transformVec2Mat(const cv::Mat& rvec, 
                                      const cv::Mat& tvec, 
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


/* *******************************************************************
 *
 * transformMat2Vec
 *
 * *******************************************************************/
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

