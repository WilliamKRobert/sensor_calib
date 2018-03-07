#ifndef OCAMCALIBMODEL_H
#define OCAMCALIBMODEL_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "camera_camera_calib/utility.h"

#define CMV_MAX_BUF 1024
#define MAX_POL_LENGTH 64

class OCamCalibModel
{
public:
    // OCamCalibModel( int width, int height,
    //           double xc, double yc, 
    //           std::vector<double> pol,
    //           std::vector<double> pol_inv,
    //           double c, double d, double e);

    // OCamCalibModel( int width, int height,
    //           std::vector<double> pol,
    //           double xc, double yc, 
    //           double c, double d, double e);

    void setParameter(int width, int height,
              double xc, double yc, 
              std::vector<double> pol,
              std::vector<double> pol_inv,
              double c, double d, double e);

    bool omni3d2pixel(const cv::Point3f &Ps,
                          cv::Point2f &Ms);

    bool estimateTransformation(std::vector<cv::Point2f> Ms,
                                std::vector<cv::Point3f> Ps,
                                Eigen::Matrix4d &  out_T_t_c);
    
    // obtained directly from OCamCalib
    bool get_ocam_model(char *filename);


    bool cam2world(const cv::Point2f &Ms,
                          cv::Point3f &Ps)const;
    void cam2world(double point3D[3], double point2D[2]);

    template <typename T>
    bool cam2world_unitfocal(cv::Point_<T> &Ms, 
      cv::Point3_<T> &Ps, bool &isback)const;

    template <typename T>
    bool findCamPose(std::vector<cv::Point_<T> > Ms,
                    std::vector<cv::Point3_<T> > Ps,
                    Eigen::Matrix4d &  out_T_t_c) const;

    template <typename T>
    bool findCamPose(std::vector<cv::Point_<T> > Ms, 
                    std::vector<cv::Point3_<T> > Ps,
                    cv::Mat &rvec,
                    cv::Mat &tvec) const;

    template <typename T>
    bool solveAnalyticalSol( std::vector<cv::Point_<T> > Ms, 
                      std::vector<cv::Point3_<T> > Ps,
                      const cv::Point_<T> center,
                      Eigen::Matrix<T, 4, 4> &pose) const;



    template <typename T>
    bool findIntrinsic(
              const std::vector<std::vector<cv::Point_<T> > > Ms, 
              const std::vector<std::vector<cv::Point3_<T> > > Ps,
              std::vector<Eigen::Matrix<T, 4, 4> > &Rt_set,
              const int taylor_order,
              const size_t num_pt,
              std::vector<double> &poly) const;
    
    template <typename T>
    bool findAnalyticalExtrinsics( 
                        const std::vector<cv::Point_<T> > Ms, 
                        const std::vector<cv::Point3_<T> > Ps,
                        Eigen::Matrix<T, 4, 4> &pose) const;

private:
    template <typename T>
    bool findExtrinsic(std::vector<cv::Point_<T> > Ms, 
                      std::vector<cv::Point3_<T> > Ps,
                      std::vector<Eigen::Matrix<T, 4, 4> > &Rt_set) const;
public:
    //------------------------------------------------------------------------------
    // void world2cam(T point2D[2], T point3D[3])const;
    bool world2cam(const cv::Point3f &Ps,
                          cv::Point2f &Ms)const;

    template <typename T>
    void world2cam_naive(T point2D[2], T point3D[3])const;
   
    template <typename T>
    void triangulate(const Eigen::Matrix<T, 3, 1> & point1, 
                     const Eigen::Matrix<T, 3, 1> & ray1,
                     const Eigen::Matrix<T, 3, 1> & point2, 
                     const Eigen::Matrix<T, 3, 1> & ray2,
                     Eigen::Matrix<T, 3, 1> & outTriangulatedPoint, 
                     T & outGap,
                     T & outS1, 
                     T & outS2, 
                     Eigen::Matrix<T, 3, 1> &xm, 
                     Eigen::Matrix<T, 3, 1> &xn) const;

    cv::Point3f pointTransform(const cv::Point3f& p0, const Eigen::Matrix4d& transform);
    cv::Point2f targetPoint2ImagePixel(const cv::Point3f& p0, const Eigen::Matrix4d& target_pose);
    void transformVec2Mat(const cv::Mat& rvec, const cv::Mat& tvec, 
                          Eigen::Matrix4d& T_camera_model) const;
    void transformMat2Vec(const Eigen::Matrix4d& T_camera_model, 
                          cv::Mat& rvec, cv::Mat& tvec) const;

    double getImageWidth(){ assert(m_is_initialized); return m_width; }
    double getImageHeight() { assert(m_is_initialized); return m_height; }

    template <typename T>
    T findRho(T tantheta) const;

    void findInvPoly(const unsigned int order,
                  std::vector<double> &pol);
private:
    // The coordinate system in OCam is different from that in 
    // OpenCV
    template <typename T>
    void excoordinate2D(cv::Point_<T>& pt)const{
       float temp = pt.x;
       pt.x = pt.y;
       pt.y = temp;
    }

    template <typename T>
    void excoordinate3D(cv::Point3_<T>& pt)const{
        float temp = pt.x;
        pt.x = pt.y;
        pt.y = temp;
        pt.z = -pt.z;
    }

    template <typename T>
    void excoordinate2D(T point2D[2])const{
        T temp = point2D[0];
        point2D[0] = point2D[1];
        point2D[1] = temp;
    }

    template <typename T>
    void excoordinate3D(T point3D[3])const{
        T temp = point3D[0];
        point3D[0] = point3D[1];
        point3D[1] = temp;
        point3D[2] = -point3D[2];
    }

    template <typename T>
    bool pnpPose( std::vector<cv::Point_<T>> Ms, 
                                  std::vector<cv::Point3_<T>> Ps,
                                  cv::Mat &rvec,
                                  cv::Mat &tvec,
                                  bool &isback) const;

    bool m_is_initialized;
    double m_xc, m_yc; //camera paramter

    std::vector<double> m_pol; // polynomial coefficients of function F, from low degree to high degree
                              // F(r) = a0 + a1*r + a2*r^2 + a3*r^3 + a4*r^4    
    int m_len_pol;
    std::vector<double> m_pol_inv;
    int m_len_pol_inv;

    double m_c, m_d, m_e;
    int m_width, m_height;


public:
    double get_u0(){ return m_xc; }
    double get_v0(){ return m_yc; }
    double get_c(){ return m_c; }
    double get_d(){ return m_d; }
    double get_e(){ return m_e; }

    void get_coeff(std::vector<double>& coeff){
        coeff = m_pol;
    }
    void get_coeff_inv(std::vector<double>& coeff_inv){
        coeff_inv = m_pol_inv;
    }

    OCamCalibModel(){m_is_initialized = false;}

    OCamCalibModel( int width, int height,
        std::vector<double> pol,
        std::vector<double> pol_inv,
        double xc, double yc, 
        double c, double d, double e)
    { 
        m_width = width;
        m_height = height;

        m_xc = xc;
        m_yc = yc;

        for (size_t i=0; i<pol.size(); i++)
            m_pol.push_back(pol[i]);

        for (size_t i=0; i<pol_inv.size(); i++)
            m_pol_inv.push_back(pol_inv[i]);

        m_len_pol = m_pol.size();
        m_len_pol_inv = m_pol_inv.size();

        m_c = c;
        m_d = d;
        m_e = e;

        m_is_initialized = true;
    }



    OCamCalibModel( int width, int height,
        std::vector<double> pol_inv,
        double xc, double yc, 
        double c, double d, double e)
    { 
        m_width = width;
        m_height = height;

        m_xc = xc;
        m_yc = yc;

        for (size_t i=0; i<pol_inv.size(); i++){
            m_pol_inv.push_back(pol_inv[i]);
        }

        m_len_pol_inv = m_pol_inv.size();

        m_c = c;
        m_d = d;
        m_e = e;

        m_is_initialized = true;
    }


/* *******************************************************************
 *
 * world2cam
 *
 * *******************************************************************/
template <typename T>
void world2cam(T point2D[2], T point3D[3])const
{
    assert(m_is_initialized);
    assert(m_pol_inv.size() != 0);
    excoordinate3D(point3D);

    T norm        = sqrt(point3D[0]*point3D[0] + point3D[1]*point3D[1]);
    T theta       = atan2(point3D[2], norm);
    T t, t_i;
    T rho, x, y;
    T invnorm;

    if (norm != T(0)) 
    {
      invnorm = T(1) / norm;
      t  = theta;
      rho = T(m_pol_inv[0]);
      t_i = T(1);

      for (size_t i = 1; i < m_len_pol_inv; i++)
      {
        t_i *= t;
        rho += t_i*m_pol_inv[i];
      }

      x = point3D[0]*invnorm*rho;
      y = point3D[1]*invnorm*rho;

      point2D[0] = x*m_c + y*m_d + m_xc;
      point2D[1] = x*m_e + y   + m_yc;
    }
    else
    {
      point2D[0] = T(m_xc);
      point2D[1] = T(m_yc);
    }

    excoordinate2D(point2D);
}

};

template<typename T>
void OCamCalibModel::world2cam_naive(T point2D[2], T point3D[3])const
{
    assert(m_is_initialized);

    T norm        = sqrt(point3D[0]*point3D[0] + point3D[1]*point3D[1]);
    T tantheta;
    T rho, x, y;
    T invnorm;

    if (norm != T(0)) 
    {
      invnorm = T(1) / norm;
      tantheta = point3D[2] * invnorm;
      rho = findRho(tantheta);
    
      x = point3D[0]*invnorm*rho;
      y = point3D[1]*invnorm*rho;

      point2D[0] = x*m_c + y*m_d + m_xc;
      point2D[1] = x*m_e + y   + m_yc;
    }
    else
    {
      point2D[0] = T(m_xc);
      point2D[1] = T(m_yc);
    }

    //excoordinate2D(point2D);
}

template <typename T>
T OCamCalibModel::findRho(T tantheta) const{
    T lower = T(0.0);
    T upper = T(0.5 * m_width);
    T tol = T(1e-9);
    unsigned int max_iter = 1000;
    if (tantheta < T(-0.8)) {
        std::cout << tantheta << " " << "Error" << std::endl;
        return T(550);
    }
    T rho = brentsFunc(projectionFunc, m_pol, tantheta, lower, upper, tol, max_iter);
    //std::cout << tantheta << " " << rho << std::endl;
    // TODO: check if rho is valid or not
    return rho;
}


/* *******************************************************************
 *
 * triangulate
 *
 * *******************************************************************/
template <typename T> 
void OCamCalibModel::triangulate(const Eigen::Matrix<T, 3, 1> & point1, 
                                 const Eigen::Matrix<T, 3, 1> & ray1,
                                 const Eigen::Matrix<T, 3, 1> & point2, 
                                 const Eigen::Matrix<T, 3, 1> & ray2,
                                 Eigen::Matrix<T, 3, 1> & outTriangulatedPoint, 
                                 T & outGap,
                                 T & outS1, 
                                 T & outS2, 
                                 Eigen::Matrix<T, 3, 1> &xm, 
                                 Eigen::Matrix<T, 3, 1> &xn) const{

    Eigen::Matrix<T, 3, 1> t12 = point2 - point1;

    Eigen::Matrix<T, 2, 1> b;
    b(0, 0) = t12.dot(ray1);
    b(1, 0) = t12.dot(ray2);
    Eigen::Matrix<T, 2, 2> A;
    A(0, 0) = ray1.dot(ray1);
    A(1, 0) = ray1.dot(ray2);
    A(0, 1) = -A(1, 0);
    A(1, 1) = -ray2.dot(ray2);
    Eigen::Matrix<T, 2, 1> lambda = A.inverse() * b;
    xm = point1 + lambda[0] * ray1;
    xn = point2 + lambda[1] * ray2;
    t12 = (xm - xn);

    outGap = t12.norm();
    outTriangulatedPoint = xn + T(0.5) * t12;
    outS1 = lambda(0, 0);
    outS2 = lambda(1, 0);

}
#endif
