/*
 * This file is partially adapted from AprilTags C++ Library
 */

#ifndef APRILTAGSDETECTOR_H
#define APRILTAGSDETECTOR_H

#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag25h7.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag36h9.h"
#include "AprilTags/Tag36h11.h"

#include "camera_camera_calib/omniModel.h"

class AprilTagsDetector
{
public:
	// default constructor
	AprilTagsDetector() :
    // default settings, most can be modified through command line options (see below)
    m_tagDetector(NULL),
    m_tagCodes(AprilTags::tagCodes36h11),

    m_draw(true),
    m_timing(false),

    m_width(640),
    m_height(480),
    m_tagSize(0.166),
    m_fx(600),
    m_fy(600),
    m_px(m_width/2),
    m_py(m_height/2),

    m_exposure(-1),
    m_gain(-1),
    m_brightness(-1),

    m_deviceId(0),
    m_windowName(string("apriltags_detection"))
 	{}

 	AprilTagsDetector(cv::Mat intrinsics, cv::Mat distortionCoeff, double mirror, double width, double height, double tagSize) :
    // default settings, most can be modified through command line options (see below)
    m_tagDetector(NULL),
    m_tagCodes(AprilTags::tagCodes36h11),

    m_draw(true),
    m_timing(false),

    m_width(width),
    m_height(height0),
    m_tagSize(tagSize),

    m_deviceId(0),
    m_windowName(string("apriltags_detection"))
 	{
 		m_px = intrinsics.at<double>(0, 2);
		m_py = intrinsics.at<double>(1, 2);
		m_fx = intrinsics.at<double>(0, 0);
		m_fy = intrinsics.at<double>(1, 1);
		m_xi = mirror;
		m_k1 = distortionCoeff.at<double>(0, 0);
		m_k2 = distortionCoeff.at<double>(1, 0);
		m_p1 = distortionCoeff.at<double>(2, 0);
		m_p2 = distortionCoeff.at<double>(3, 0);
		m_fov_parameter = m_xi <= 1 ? xi : 1.0/xi;

		camModel.setParrameter(intrinsics, distortionCoeff, mirror);
 	}
	bool AprilTagsDetector::findTagPose(cv::Mat& img, Eigen::Vector3d& translation, Eigen::Matrix3d& rotation);

private:
	inline double standardRad(double t) const;
	void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) const;

	void setTagCodes(string s);
	void setup();
	void printDetection(AprilTags::TagDetection& detection) const;
	void processImage(cv::Mat& image, cv::Mat& image_gray);

	AprilTags::TagDetector* m_tagDetector;
	AprilTags::TagCodes m_tagCodes;

	bool m_draw; // draw image and April tag detections?
	bool m_timing; // print timing information for each tag extraction call

	int m_width; // image size in pixels
	int m_height;
	double m_tagSize; // April tag side length in meters of square black frame
	double m_fx; // camera focal length in pixels
	double m_fy;
	double m_px; // camera principal point
	double m_py;
	double m_xi;

	double m_k1, m_k2, m_p1, m_p2; // camera distortion parameters
	double m_fov_parameter;

	int m_deviceId; // camera id (in case of multiple cameras)

	int m_exposure;
	int m_gain;
	int m_brightness;

	string m_windowName; // name of image drawing window

	OmniModel camModel;

};

#endif