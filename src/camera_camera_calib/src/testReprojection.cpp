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
#include "camera_camera_calib/ocamCalibModel.h"

using namespace std;
using namespace cv;

std::pair<double, double> getXYZ(double squareDist, int id, int m_tagRows, int m_tagCols){
  double x = ( id % (m_tagCols+1) ) * squareDist;
  double y = ( id / (m_tagCols+1) ) * squareDist;
  
  return std::pair<double, double>(x, y);
}

/*
 * Camera-camera extrinsic parameter calibration
 */
int main(int argc, char **argv)
{
    /* 
     * Load image, convert from ROS image format to OpenCV Mat
     */
    ros::init(argc, argv, "camera_camera_calib");    
    string bag_file("/home/audren/Documents/data/small_drone_v2/ufo_2017-08-01-19-58-02.bag");

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
    string inputSettingsFile("/home/audren/Documents/lidar-camera-calib/src/camera_camera_calib/settings/settings.xml");
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

    double width = 1050;//526;
    double height = 1050; //526;
    int tagRows = 10, tagCols = 7;
    double tagSize = 0.088; // unit: m
    double tagSpacing = 0.25; // unit: %

    // // Camera intrinsics
    std::vector<double> cam0_ss;
    double temp[] = {-2.575876e+02, 0.000000e+00, 2.283578e-04, 8.908668e-06, -2.621133e-08, 3.037693e-11 };
    for (size_t i=0; i<6; i++)
        cam0_ss.push_back(temp[i]);

    double cam0_u0 = 532.425687;
    double cam0_v0 = 517.382409;
    double cam0_c = 1.000805;
    double cam0_d = 0.000125;
    double cam0_e = 2.5200e-04;

    Mat cam0_proj = s.intrinsics0;
    Mat cam0_dist = s.distortion0;
    double cam0_xi =  s.xi0;
   
    OmniModel cam0(cam0_proj, cam0_dist, cam0_xi, cam0_u0, cam0_v0, cam0_ss, cam0_c, cam0_d, cam0_e);
   
    
    AprilTagsDetector apriltags0(cam0, 
                                 width, height, 
                                 tagRows, tagCols,
                                 tagSize, tagSpacing,
                                 string("cam0_apriltags_detection"));
   

    // vector<Eigen::Matrix4d> tagPoses0;
    // vector<cv::Point2f> cam0_imgPts;

    OCamCalibModel ocamcalib_cam0;
    char ocamfile[] = "/home/audren/Documents/data/small_drone_v2/Dart_21905596_high_res/calib_results_dart_21905596_high_res.txt";
    bool bopen = ocamcalib_cam0.get_ocam_model(ocamfile);

    for (size_t i=0; i<im0_seq.size(); i++){
        /*
         * Step 1: Find out camera extrinsic parameter using PnP
         */
        // image pre-processing: 
        //      convert to gray scale
        //      undistort fisheye camera image
        cout << endl << endl;
        cout << "-----------------------------New image received-----------------------------" << endl;
        cout << endl;
        Mat im0 = im0_seq[i];
        
        if (im0.channels() == 3)
            cvtColor(im0, im0, COLOR_RGB2GRAY);

        Eigen::Matrix4d target_pose0;
        vector<AprilTags::TagDetection> detections0;
        vector<cv::Point3f> objPts0;
        vector<cv::Point2f> imgPts0;
        vector<std::pair<bool, int> >tagid_found0;
        
        Mat reproj_im0 = im0.clone();
        cv::cvtColor(im0, reproj_im0, cv::COLOR_GRAY2BGR);

        bool bfind0 = apriltags0.getDetections(im0, detections0, objPts0, imgPts0, tagid_found0);
        bool good_estimation0 = apriltags0.findCamPose(objPts0, imgPts0, target_pose0);
        cout << target_pose0 << endl;
        waitKey(1000);
        cout << "objPts number: " << objPts0.size() << endl;
        /*
         * test 1: cam2world and world2cam
         */
        // for (size_t j=0; j<objPts0.size(); j++){
        //     double Ms[2] = {imgPts0[j].x, imgPts0[j].y};
        //     double Ps[3];
        //     ocamcalib_cam0.cam2world(Ps, Ms);
        //     ocamcalib_cam0.world2cam(Ms, Ps);
            
        //     cv::circle(reproj_im0, cv::Point2f(imgPts0[j].x, imgPts0[j].y), 1, cv::Scalar(0,255,14,0), 1);
        //     cv::circle(reproj_im0, cv::Point2f(Ms[0], Ms[1]), 5, cv::Scalar(255,0,0,0), 1);
        // }

        /*
         * test 2: pose estimation
         */
        for (size_t j=0; j<objPts0.size(); j++){
            double Ms[2];

            Eigen::Vector4d old_pt;
            old_pt << objPts0[j].x, objPts0[j].y, objPts0[j].z, 1;
            Eigen::Vector4d new_pt = target_pose0 * old_pt;

            double Ps[3] = {new_pt(1), new_pt(0),  -new_pt(2)};
            ocamcalib_cam0.world2cam(Ms, Ps);
            cv::circle(reproj_im0, cv::Point2f(imgPts0[j].x, imgPts0[j].y), 1, cv::Scalar(0,255,14,0), 1);
            cv::circle(reproj_im0, cv::Point2f(Ms[1], Ms[0]), 8, cv::Scalar(255,0,0,0), 2);
        }
       
        imshow("Reprojection", reproj_im0);
        waitKey(0);

    }

     
    /*
     * test: using synthetic data 
     * test pass
     */
    // Eigen::Matrix4d pose_ground_truth;
    // pose_ground_truth << 1, 0, 0, 1,
    //                      0, 0.866, 0.5, 0.9,
    //                      0, -0.5, 0.866, 0.5,
    //                      0, 0, 0, 1;


    // double squareDist = tagSize + tagSize * tagSpacing;
    // double halfSquare = tagSize / 2.0;
    // vector<cv::Point3f> objPts;
    // for (size_t i=0; i<80; i++){
    //   std::pair<double, double> center = getXYZ(squareDist, i, 10, 7);
    //   double cx = center.first;
    //   double cy = center.second;

    //   /* pass
    //    * cout << "-------------------debug------------------" << endl;
    //    * cout << cx << " " << cy << endl;
    //    */
    //   objPts.push_back(cv::Point3f(cx - halfSquare, cy - halfSquare, 0));
    //   objPts.push_back(cv::Point3f(cx + halfSquare, cy - halfSquare, 0));
    //   objPts.push_back(cv::Point3f(cx + halfSquare, cy + halfSquare, 0));
    //   objPts.push_back(cv::Point3f(cx - halfSquare, cy + halfSquare, 0));
    // }

    // vector<cv::Point2f> imgPts;
    // Mat img(width, height, CV_8UC3, Scalar(0,0,0));
    // for (size_t i=0; i<objPts.size(); i++){
    //     Eigen::Vector4d old_pt;
    //     old_pt << objPts[i].x, objPts[i].y, objPts[i].z, 1;
    //     Eigen::Vector4d new_pt = pose_ground_truth * old_pt;

    //     double Ps[3] = {new_pt(1), new_pt(0),  -new_pt(2)};
    //     double Ms[2];
    //     ocamcalib_cam0.world2cam(Ms, Ps);
    //     cv::Point2f ipt;
    //     ipt.x = Ms[1];
    //     ipt.y = Ms[0];
    //     imgPts.push_back(ipt);

    //     /*
    //      * orientation is correct
    //      */
    //     cv::circle(img, cv::Point2f(ipt.x, ipt.y), 1, cv::Scalar(255,255,255,0), 1);

    // }

    // imshow("Test", img);
    // waitKey(0);

    // // from synthetic image poinsts and object points recover pose of target
    // /*
    //  * test pass
    //  */
    // Eigen::Matrix4d pose_estimated;
    // bool good_estimation0 = apriltags0.findCamPose(objPts, imgPts, pose_estimated);
    // cout << "---------------target pose in camera frame-----------------" << endl;
    // cout << pose_estimated << endl;
    
    
    return 0;
}
