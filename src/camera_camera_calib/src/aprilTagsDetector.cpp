#include <iostream>
#include <sys/time.h>

using namespace std;

#include "camera_camera_calib/omniModel.h"
#include "camera_camera_calib/aprilTagsDetector.h"

double tic(){
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}

inline double AprilTagsDetector::standardRad(double t) const{
  if (t >= 0.) {
    t = fmod(t+PI, TWOPI) - PI;
  } else {
    t = fmod(t-PI, -TWOPI) + PI;
  }
  return t;
}

/**
 * Convert rotation matrix to Euler angles
 */
void AprilTagsDetector::wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) const {
    yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
    double c = cos(yaw);
    double s = sin(yaw);
    pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
    roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}

// changing the tag family
void AprilTagsDetector::setTagCodes(string s) {
	if (s=="16h5") {
	  m_tagCodes = AprilTags::tagCodes16h5;
	} else if (s=="25h7") {
	  m_tagCodes = AprilTags::tagCodes25h7;
	} else if (s=="25h9") {
	  m_tagCodes = AprilTags::tagCodes25h9;
	} else if (s=="36h9") {
	  m_tagCodes = AprilTags::tagCodes36h9;
	} else if (s=="36h11") {
	  m_tagCodes = AprilTags::tagCodes36h11;
	} else {
	  cout << "Invalid tag family specified" << endl;
	  exit(1);
	}
}


void AprilTagsDetector::setup() {
    m_tagDetector = new AprilTags::TagDetector(m_tagCodes);

    // prepare window for drawing the camera images
    if (m_draw) {
      cv::namedWindow(m_windowName, 1);
    }
}

void AprilTagsDetector::printDetection(AprilTags::TagDetection& detection) const {
    cout << "  Id: " << detection.id
         << " (Hamming: " << detection.hammingDistance << ")";

    // recovering the relative pose of a tag:

    // NOTE: for this to be accurate, it is necessary to use the
    // actual camera parameters here as well as the actual tag size
    // (m_fx, m_fy, m_px, m_py, m_tagSize)

    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
                                             translation, rotation);

    Eigen::Matrix3d F;
    F <<
      1, 0,  0,
      0,  -1,  0,
      0,  0,  1;
    Eigen::Matrix3d fixed_rot = F*rotation;
    double yaw, pitch, roll;
    wRo_to_euler(fixed_rot, yaw, pitch, roll);

    cout << "  distance=" << translation.norm()
         << "m, x=" << translation(0)
         << ", y=" << translation(1)
         << ", z=" << translation(2)
         << ", yaw=" << yaw
         << ", pitch=" << pitch
         << ", roll=" << roll
         << endl;

    // Also note that for SLAM/multi-view application it is better to
    // use reprojection error of corner points, because the noise in
    // this relative pose is very non-Gaussian; see iSAM source code
    // for suitable factors.
  }

void AprilTagsDetector::processImage(cv::Mat& image, vector<AprilTags::TagDetection>& detections) {
    // alternative way is to grab, then retrieve; allows for
    // multiple grab when processing below frame rate - v4l keeps a
    // number of frames buffered, which can lead to significant lag
    //      m_cap.grab();
    //      m_cap.retrieve(image);

    // detect April tags (requires a gray scale image)

    if (image.channels() == 3)
    	cv::cvtColor(image, image, CV_BGR2GRAY);
    double t0;
    if (m_timing) {
      t0 = tic();
    }
    detections = m_tagDetector->extractTags(image);
    if (m_timing) {
      double dt = tic()-t0;
      cout << "Extracting tags took " << dt << " seconds." << endl;
    }

    // print out each detection
    if (m_verbose){
      cout << detections.size() << " tags detected:" << endl;
      for (int i=0; i<detections.size(); i++) {
        printDetection(detections[i]);
      }
    }

    // show the current image including any detections
    if (m_draw) {
      for (int i=0; i<detections.size(); i++) {
        // also highlight in the image
        detections[i].draw(image);
      }
      imshow(m_windowName, image); // OpenCV call
    }
}


// right now the ID of tags on the board starts from 0 to 6 in the first row
// and begin with 8, end with 14 in the second row 
std::pair<double, double> getLocation(double squareDist, int id, int m_tagRows, int m_tagCols){
  double x = ( id % (m_tagCols+1) ) * squareDist;
  double y = ( id / (m_tagCols+1) ) * squareDist;
  
  return std::pair<double, double>(x, y);
}

bool AprilTagsDetector::getDetections(cv::Mat& img, 
                                      std::vector<AprilTags::TagDetection> &detections, 
                                      std::vector<cv::Point3f> &objPts,
                                      std::vector<cv::Point2f> &imgPts,
                                      std::vector<std::pair<bool, int> >& tagid_found){
  // tagid_found: first: if tag is found, corresponding index in objPts 
  processImage(img, detections);
  
  if (detections.size() <= 0)
    return false;

  size_t num_tags = m_tagRows * (m_tagCols+1);
  for (size_t i=0; i<num_tags; i++){
      tagid_found.push_back(std::pair<bool, int>(false, -1));
  }


  double squareDist = m_tagSize + m_tagSize * m_tagSpacing;
  double s = m_tagSize / 2.0;
  for (size_t i=0; i<detections.size(); i++){
      AprilTags::TagDetection tag = detections[i];
      int id = tag.id;

      tagid_found[id] = std::pair<bool, int>(true, i*4);

      std::pair<double, double> center = getLocation(squareDist, id, m_tagRows, m_tagCols);
      double cx = center.first;
      double cy = center.second;

      objPts.push_back(cv::Point3f(cx - s, cy + s, 0));
      objPts.push_back(cv::Point3f(cx + s, cy + s, 0));
      objPts.push_back(cv::Point3f(cx + s, cy - s, 0));
      objPts.push_back(cv::Point3f(cx - s, cy - s, 0));

      std::pair<float, float> p1 = tag.p[0];
      std::pair<float, float> p2 = tag.p[1];
      std::pair<float, float> p3 = tag.p[2];
      std::pair<float, float> p4 = tag.p[3];
      imgPts.push_back(cv::Point2f(p1.first, p1.second));
      imgPts.push_back(cv::Point2f(p2.first, p2.second));
      imgPts.push_back(cv::Point2f(p3.first, p3.second));
      imgPts.push_back(cv::Point2f(p4.first, p4.second));

  }

  return true;
}

