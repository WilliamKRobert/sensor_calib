#ifndef FISHEYEMODEL_H
#define FISHEYEMODEL_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

class OminiModel
{
public:
	ominiModel(cv::Mat intrinsics, cv::Mat distortionCoeff, double mirror){ // intrinsics: 3 by 3 distortionCoeff: 4 by 1
		u0 = intrinsics.at<double>(0, 2);
		v0 = intrinsics.at<double>(1, 2);
		gamma1 = intrinsics.at<double>(0, 0);
		gamma2 = intrinsics.at<double>(1, 1);
		xi = mirror;
		k1 = distortionCoeff.at<double>(0, 0);
		k2 = distortionCoeff.at<double>(1, 0);
		p1 = distortionCoeff.at<double>(2, 0);
		p2 = distortionCoeff.at<double>(3, 0);
	}

	void distortion(const double mx_u, const double my_u, 
                double *dx_u, double *dy_u, 
                double *dxdmx, double *dydmx,
                double *dxdmy, double *dydmy);

	void undistortGN(const double u_d, const double v_d, double *u,
                                     double *v);

	void undistortImage(cv::Mat distorted, cv::Mat undistorted);

	bool estimateTransformation(const int &obs, cv::Mat &transformation);
private:
	double u0, v0, gamma1, gamma2, xi; //camera paramter
	double k1, k2, p1, p2; // distortion parameter

};
#endif