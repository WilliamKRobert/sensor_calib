#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ros/ros.h"
#include "ceres/rotation.h"

#include "camera_camera_calib/loadBag.h"
#include "camera_camera_calib/loadSettings.h"
#include "camera_camera_calib/objectPose.h"
#include "camera_camera_calib/optimizer.h"
#include "camera_camera_calib/omniModel.h"
#include "camera_camera_calib/aprilTagsDetector.h"

using namespace std;
using namespace cv;

/*
 * Lidar-camera extrinsic parameter calibration
 */
int main(int argc, char **argv)
{
    /* 
     * Load image, convert from ROS image format to OpenCV Mat
     */
    ros::init(argc, argv, "camera_camera_calib");    
    string bag_file("/home/audren/Documents/data/small_drone_v2/ufo_v1_calibration_chessboard_low_res.bag");

    vector<Mat> im0_seq, im1_seq;
    string topic0 = string("/synthetic_gimbal/cam0") + "/image_raw";
    string topic1 = string("/synthetic_gimbal/cam1") + "/image_raw";
    size_t sample_num = 10;
    size_t max_im_num = 500;
    loadBag(bag_file, topic0, topic1, im0_seq, im1_seq, sample_num, max_im_num);

    if (im0_seq.size() != im1_seq.size() || im0_seq.size() < 10){
        cout << "Inconsistent image numbers or too few images!" << endl;
        return 0;
    }
    cout << "---------------------------------------------" << endl;
    cout << "Number of left images:   " << im0_seq.size() << endl;
    cout << "Number of right images:  " << im1_seq.size() << endl;
    cout << "---------------------------------------------" << endl;

    /*
     * Read setting files
     */
    Settings s;
    string inputSettingsFile("/home/audren/lidar_camera_calib/calib_ws/src/camera_camera_calib/settings/settings.xml");
    FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
        return -1;
    }
    fs["Settings"] >> s;
    fs.release();                                         // close Settings file

    if (!s.goodInput)
    {
        cout << "Invalid input detected. Application stopping. " << endl;
        return -1;
    }

    // Camera intrinsics
    Mat cam0_proj = s.intrinsics0;
    Mat cam0_dist = s.distortion0;
    double cam0_xi =  s.xi0;
    Mat cam1_proj = s.intrinsics1;
    Mat cam1_dist = s.distortion1;
    double cam1_xi =  s.xi1;
    OmniModel cam0(cam0_proj, cam0_dist, cam0_xi);
    OmniModel cam1(cam1_proj, cam1_dist, cam1_xi);
    cout << "Camera intrinsic matrix: " << endl << cam0_proj << endl << endl;
    cout << "Distortion coefficients: " << endl << cam0_dist << endl << endl;
    cout << "Mirror parameter: " << endl << cam0_xi << endl  ;
    cout << "---------------------------------------------" << endl;

    Size patternsize = s.boardSize; // interior number of corners
    float squareSize = s.squareSize; // unit: mm
    cout << "Checkerboard pattern size: " << patternsize << endl;
    cout << "Square size: " << squareSize << endl;
    cout << "---------------------------------------------" << endl;
    // OmniModel model(camera_matrix, dist_coeffs, xi);
    vector<vector<KeyPoint> > kps_vec_1, kps_vec_2;
    AprilTagsDetector apriltags();
    vector<Eigen::Matrix4d> tagPoses0, tagPoses1;
    for (size_t i=0; i<im0_seq.size(); i++){
        /*
         * Step 1: Find out camera extrinsic parameter using PnP
         */
        // image pre-processing: 
        //      convert to gray scale
        //      undistort fisheye camera image
        Mat im0 = im0_seq[i], im1 = im1_seq[i];
        
        if (im0.channels() == 3)
            cvtColor(im0, im0, COLOR_RGB2GRAY);
        if (im1.channels() == 3)
            cvtColor(im1, im1, COLOR_RGB2GRAY);
        
        Eigen::Matrix4d pose0, pose1;
        bool bfind0 = findTagPose(im0, pose0);
        bool bfind1 = findTagPose(im1, pose1);
        if (bfind0 && bfind1){
            tagPoses0.push_back(pose0);
            tagPoses1.push_back(pose1);
        }
    }
    // /*      
    //  * Step 2: Obtain camera-Camera estimated transform 
    //  */
    // // initial guess of lidar transform in camera frame
    // Mat init_rvec = s.initialRotation; // 4 by 1, quaternion
    // Mat init_tvec = s.initialTranslation; // 3 by 1
    
    double transform[6];
    transform[0] = 0;
    transform[1] = 0;
    transform[2] = 0;
    transform[3] = 0;
    transform[4] = 0;
    transform[5] = 0;
    
    // /*
    //  * Step 3: Lidar-camera transform optimization
    //  *  cost function: square sum of distance between scan points and chessboard plane
    //  *  state vector: rotation (4 parameters) and translation (3 parameters)
    //  *  constraint: scan points should fall   in the range of chessboard (x, y)
    //  */
    double selection_ratio = 0.75;
    google::InitGoogleLogging("Bundle Adjustment");
    optimizer ba;    
    ba.bundleAdjustment(tagPoses0, tagPoses1, transform);

    cout << "--------------------------------------------" << endl;
    cout << "cam1 pose in cam0 frame: " << endl;
    for (size_t i=0; i<6; i++)
        cout << transform[i] << " ";
    cout << endl;

    // Mat rvec = Mat(3, 1, CV_64F);
    // Mat tvec = Mat(3, 1, CV_64F);
    // Mat rmatrix, new_rmatrix;
    // rvec.at<double>(0,0) = transform[0];
    // rvec.at<double>(1,0) = transform[1];
    // rvec.at<double>(2,0) = transform[2];
    // tvec.at<double>(0,0) = transform[3];
    // tvec.at<double>(1,0) = transform[4];
    // tvec.at<double>(2,0) = transform[5];
    // cv::Rodrigues(rvec, rmatrix);
    // transpose(rmatrix, new_rmatrix);
    // tvec = - new_rmatrix * tvec;
    // cv::Rodrigues(new_rmatrix, rvec);
    
    // cout << "--------------------------------------------" << endl;
    // cout << "Estimated camera pose in LIDAR frame:" << endl;
    // cout << " [rx (rad), ry (rad), rz (rad), tx (mm), ty (mm), tz (mm)]" << endl;
    // cout <<  0 << " " << 1.047198 << " " <<  -0.339816 << " " << 39.91 << " " << 35.38 << " " << 102.19;
    // cout << endl << endl;
    // cout << "Initial value feed into Ceres Optimization program:" << endl;
    // cout << " [rx (rad), ry (rad), rz (rad), tx (mm), ty (mm), tz (mm)]" << endl;
    // cout <<  0 << " " << 0 << " " <<  0 << " " << 10 << " " << 10 << " " << 10;
    // cout << endl << endl; 
    // cout << "Optimized camera pose in LIDAR frame " << endl;
    // cout << " [rx (rad), ry (rad), rz (rad), tx (mm), ty (mm), tz (mm)]:" << endl;
    // for (size_t i=0; i<3; i++)
    //     cout << rvec.at<double>(i, 0) << " ";
    // for (size_t i=0; i<3; i++)
    //     cout << tvec.at<double>(i, 0) << " ";
    // cout << endl;
    // cout << "--------------------------------------------" << endl;
    return 0;
}
