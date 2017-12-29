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

 	// for ocam camera
    AprilTagsDetector(int tagRows, int tagCols,
 					  double tagSize, double tagSpacing,
                      std::string windowName) :
    // default settings, most can be modified through command line options (see below)
    m_tagDetector(NULL),
    m_tagCodes(AprilTags::tagCodes36h11),

    m_draw(true),
    m_timing(false),

    m_tagRows(tagRows),
    m_tagCols(tagCols),
    m_tagSize(tagSize),
    m_tagSpacing(tagSpacing),

    m_deviceId(0),
    m_windowName(windowName),
    m_verbose(false)
    
 	{
        setup();
        m_width = 640;
	    m_height = 480;
	    m_fx = 600;
	    m_fy = 600;
	    m_px = m_width/2;
	    m_py = m_height/2;
 	}

    template <typename _Tp2, typename _Tp3>
	bool getDetections(cv::Mat& img, 
                       std::vector<AprilTags::TagDetection> &detections, 
                       std::vector<_Tp3> &objPts,
                       std::vector<_Tp2> &imgPts,
                       std::vector<std::pair<bool, int> >& tagid_found);


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

	int m_tagRows;
	int m_tagCols;
	double m_tagSize; // April tag side length in meters of square black frame
	double m_tagSpacing; // in percentage


	int m_width;
    int m_height;
    double m_fx;
    double m_fy;
    double m_px;
    double m_py;

	int m_deviceId; // camera id (in case of multiple cameras)

	int m_exposure;
	int m_gain;
	int m_brightness;

	string m_windowName; // name of image drawing windowName


	#ifndef PI
	static constexpr double PI = 3.14159265358979323846;
	#endif
	static constexpr double TWOPI = 2.0 * 3.14159265358979323846;

};

#endif