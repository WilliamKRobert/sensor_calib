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

    AprilTagsDetector(OmniModel &cam,
                    //cv::Mat intrinsics, cv::Mat distortionCoeff, double mirror, 
 	 				  double width, double height, 
 					  int tagRows, int tagCols,
 					  double tagSize, double tagSpacing,
                      std::string windowName) :
    // default settings, most can be modified through command line options (see below)
    m_tagDetector(NULL),
    m_tagCodes(AprilTags::tagCodes36h11),

    m_draw(true),
    m_timing(false),

    m_width(width),
    m_height(height),
    m_tagRows(tagRows),
    m_tagCols(tagCols),
    m_tagSize(tagSize),
    m_tagSpacing(tagSpacing),

    m_deviceId(0),
    m_windowName(windowName),
    m_verbose(false)
 	{
 		m_px = cam.m_u0;
		m_py = cam.m_v0;
		m_fx = cam.m_fu;
		m_fy = cam.m_fv;
		m_xi = cam.m_xi;
		m_k1 = cam.m_k1;
		m_k2 = cam.m_k2;
		m_p1 = cam.m_p1;
		m_p2 = cam.m_p2;
		m_fov_parameter = cam.m_fov_parameter;

		camModel = cam;

        setup();
 	}

	bool getDetections(cv::Mat& img, 
                       std::vector<AprilTags::TagDetection> &detections, 
                       std::vector<cv::Point3f> &objPts,
                       std::vector<cv::Point2f> &imgPts,
                       std::vector<std::pair<bool, int> >& tagid_found);
    bool findCamPose( const std::vector<cv::Point3f> objPts,
                      const std::vector<cv::Point2f> imgPts,
                      Eigen::Matrix4d& pose);

    OmniModel camModel;

private:
	inline double standardRad(double t) const;
	void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) const;

	void setTagCodes(string s);
	void setup();
	void printDetection(AprilTags::TagDetection& detection) const;
	void processImage(cv::Mat& image, std::vector<AprilTags::TagDetection>& detections);

	AprilTags::TagDetector* m_tagDetector;
	AprilTags::TagCodes m_tagCodes;

	bool m_draw; // draw image and April tag detections?
    bool m_verbose; // print detections
	bool m_timing; // print timing information for each tag extraction call

	int m_width; // image size in pixels
	int m_height;

	int m_tagRows;
	int m_tagCols;
	double m_tagSize; // April tag side length in meters of square black frame
	double m_tagSpacing; // in percentage
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


	#ifndef PI
	static const double PI = 3.14159265358979323846;
	#endif
	static const double TWOPI = 2.0 * 3.14159265358979323846;

};

#endif