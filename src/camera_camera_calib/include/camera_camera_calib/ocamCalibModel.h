#ifndef OCAMCALIBMODEL_H
#define OCAMCALIBMODEL_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#define CMV_MAX_BUF 1024
#define MAX_POL_LENGTH 64

class OCamCalibModel
{
public:
    OCamCalibModel(){}

    OCamCalibModel( int width, int height,
              double xc, double yc, 
              std::vector<double> pol,
              std::vector<double> pol_inv,
              double c, double d, double e){ 
        m_width = width;
        m_height = height;

        m_xc = xc;
        m_yc = yc;

        for (size_t i=0; i<pol.size(); i++)
            m_pol.push_back(pol[i]);

        for (size_t i=0; i<pol_inv.size(); i++)
            m_pol_inv.push_back(pol_inv[i]);

        m_c = c;
        m_d = d;
        m_e = e;
    }

    void setParameter(int width, int height,
              double xc, double yc, 
              std::vector<double> pol,
              std::vector<double> pol_inv,
              double c, double d, double e);


    bool cam2world(const cv::Point2f &Ms,
                          cv::Point3f &Ps)const;

    bool world2cam(const cv::Point3f &Ps,
                          cv::Point2f &Ms)const;

    bool omni3d2pixel(const cv::Point3f &Ps,
                          cv::Point2f &Ms);

    bool estimateTransformation(std::vector<cv::Point2f> Ms,
                                std::vector<cv::Point3f> Ps,
                                Eigen::Matrix4d &  out_T_t_c);

    // obtained directly from OCamCalib
    int get_ocam_model(char *filename);
    void cam2world(double point3D[3], double point2D[2]);
    bool cam2world_unitfocal(cv::Point2f &Ms,
                          cv::Point3f &Ps)const;


    bool findCamPose(std::vector<cv::Point2f> Ms,
                                std::vector<cv::Point3f> Ps,
                                Eigen::Matrix4d &  out_T_t_c) const;
    //------------------------------------------------------------------------------
    template <typename T>
    void world2cam(T point2D[2], T point3D[3])const
    {
        excoordinate3D(point3D);

        T norm        = sqrt(point3D[0]*point3D[0] + point3D[1]*point3D[1]);
        T theta       = atan(point3D[2] / norm);
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

private:

      void excoordinate2D(cv::Point2f& pt)const{
         float temp = pt.x;
         pt.x = pt.y;
         pt.y = temp;
      }

      void excoordinate3D(cv::Point3f& pt)const{
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


};

#endif