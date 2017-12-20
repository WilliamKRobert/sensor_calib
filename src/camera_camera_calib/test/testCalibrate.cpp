#include <iostream>
#include <sstream>

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

void miniparameter(const Eigen::Matrix4d& pose, double minip[6]){
    Eigen::Matrix3d R = pose.block<3,3>(0,0);
    Eigen::Vector3d R_angle = R.eulerAngles(0, 1, 2);
    minip[0] = R_angle[0];
    minip[1] = R_angle[1];
    minip[2] = R_angle[2];
    minip[3] = pose(0,3);
    minip[4] = pose(1,3);
    minip[5] = pose(2,3);
}

void miniparameter(const cv::Mat rvec, const cv::Mat tvec, double minip[6]){
    for (size_t i=0; i<3; i++){
        minip[i] = rvec.at<double>(i, 0);
    }
    for (size_t i=3; i<6; i++){
        minip[i] = tvec.at<double>(i-3, 0);
    }
}

string IntToString (int a)
{
    ostringstream temp;
    temp<<a;
    return temp.str();
}
/*
 * Lidar-camera extrinsic parameter calibration
 */
int main(int argc, char **argv)
{
    /* 
     * Load image, convert from ROS image format to OpenCV Mat
     */
    ros::init(argc, argv, "camera_camera_calib");    

    string bag_file("../data/small_drone_v2/ufo_2017-08-01-19-58-02.bag");

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

    string inputSettingsFile("./src/camera_camera_calib/settings/settings.xml");

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

    double cam0_u0 = 532.425687;
    double cam0_v0 = 517.382409;
    
    double cam1_u0 = 512.560101;
    double cam1_v0 = 523.645938;
    
    Mat cam0_proj = s.intrinsics0;
    Mat cam0_dist = s.distortion0;
    
    Mat cam1_proj = s.intrinsics1;
    Mat cam1_dist = s.distortion1;

    AprilTagsDetector apriltags0(cam0_u0, cam0_v0, 
                                 cam0_proj.at<double>(0,0), cam0_proj.at<double>(1,1), 
                                 width, height, 
                                 tagRows, tagCols,
                                 tagSize, tagSpacing,
                                 string("cam0_apriltags_detection"));
    AprilTagsDetector apriltags1(cam1_u0, cam1_v0, 
                                 cam1_proj.at<double>(0,0), cam1_proj.at<double>(1,1), 
                                 width, height, 
                                 tagRows, tagCols,
                                 tagSize, tagSpacing,
                                 string("cam1_apriltags_detection"));

    vector<vector<cv::Point2f> > cam0_imgPts, cam1_imgPts;
    vector<vector<cv::Point3f> > cam0_objPts, cam1_objPts;

    size_t num_viewused = 0;
    OCamCalibModel ocamcalib_cam0;

    char ocamfile0[] = "../data/small_drone_v2/Dart_high_res/calib_results_dart_21905596_high_res.txt";
    bool bopen0 = ocamcalib_cam0.get_ocam_model(ocamfile0);

    OCamCalibModel ocamcalib_cam1;
    char ocamfile1[] = "../data/small_drone_v2/Dart_high_res/calib_results_dart_21905597_high_res.txt";
    bool bopen1 = ocamcalib_cam1.get_ocam_model(ocamfile1);
   
    int good_array[] = {24, 25, 26, 27, 30, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 49, 50, 51, 52, 53, 54};
    std::vector<int> good_frame (good_array, good_array + sizeof(good_array) / sizeof(int) );


    double *poses0 = new double[6*good_frame.size()];
    double *poses1 = new double[6*good_frame.size()];
    // for (size_t i=0; i<im0_seq.size(); i++){
    for (size_t iframe=0; iframe<good_frame.size(); iframe++){
        int i = good_frame[iframe];
        /*
         * Step 1: Find out camera extrinsic parameter using PnP
         */
        Mat im0 = im0_seq[i], im1 = im1_seq[i];
        
        if (im0.channels() == 3)
            cvtColor(im0, im0, COLOR_RGB2GRAY);
        if (im1.channels() == 3)
            cvtColor(im1, im1, COLOR_RGB2GRAY);

        Mat reproj_im0 = im0.clone();
        cv::cvtColor(im0, reproj_im0, cv::COLOR_GRAY2BGR);
        Mat reproj_im1 = im1.clone();
        cv::cvtColor(im1, reproj_im1, cv::COLOR_GRAY2BGR);

        
        vector<AprilTags::TagDetection> detections0, detections1;
        vector<cv::Point3f> objPts0, objPts1;
        vector<cv::Point2f> imgPts0, imgPts1;
        vector<std::pair<bool, int> >tagid_found0, tagid_found1;
        
        bool bfind0 = apriltags0.getDetections(im0, detections0, objPts0, imgPts0, tagid_found0);

        waitKey(0);

        bool bfind1 = apriltags1.getDetections(im1, detections1, objPts1, imgPts1, tagid_found1);
        waitKey(10);

        cv::Mat object_pose_rvec0(3, 1, CV_64F);
        cv::Mat object_pose_tvec0(3, 1, CV_64F);
        cv::Mat object_pose_rvec1(3, 1, CV_64F);
        cv::Mat object_pose_tvec1(3, 1, CV_64F);

        bool good_estimation0 = ocamcalib_cam0.findCamPose(imgPts0, objPts0, object_pose_rvec0, object_pose_tvec0);
        bool good_estimation1 = ocamcalib_cam1.findCamPose(imgPts1, objPts1, object_pose_rvec1, object_pose_tvec1);
        
        if (!good_estimation0) {
            cout << "bad estimation of pose 0!" << " " << imgPts0.size() << " cornes detected in cam0." << endl << endl;
            continue;
        }
        if (!good_estimation1) {
            cout << "bad estimation of pose 1!" << " " << imgPts1.size() << " cornes detected in cam1."<< endl << endl;
            continue;
        }
        

        // double reproj_error0 = 0;
        // double reproj_error1 = 0;
        // for (size_t j=0; j<objPts0.size(); j++){
        //     cv::Point2f cvMs = ocamcalib_cam0.targetPoint2ImagePixel(objPts0[j], object_pose0);

        //     cv::circle(reproj_im0, cv::Point2f(imgPts0[j].x, imgPts0[j].y), 1, cv::Scalar(0,255,14,0), 1);
        //     cv::circle(reproj_im0, cv::Point2f(cvMs.x, cvMs.y), 1, cv::Scalar(255,0,0,0), 2);
        //     reproj_error0 += (imgPts0[j].x - cvMs.x) * (imgPts0[j].x - cvMs.x) + 
        //                     (imgPts0[j].y - cvMs.y) * (imgPts0[j].y - cvMs.y);
        // }
        // cout << "Reprojection error for cam0: " << reproj_error0 << endl;
        // imshow("Reprojection of cam0", reproj_im0);
        // int key0 = waitKey(10) % 256;
        // for (size_t j=0; j<objPts1.size(); j++){
        //     cv::Point2f cvMs = ocamcalib_cam1.targetPoint2ImagePixel(objPts1[j], object_pose1);

        //     cv::circle(reproj_im1, cv::Point2f(imgPts1[j].x, imgPts1[j].y), 1, cv::Scalar(0,255,14,0), 1);
        //     cv::circle(reproj_im1, cv::Point2f(cvMs.x, cvMs.y), 1, cv::Scalar(255,0,0,0), 2);
        //     reproj_error1 += (imgPts1[j].x - cvMs.x) * (imgPts1[j].x - cvMs.x) + 
        //                     (imgPts1[j].y - cvMs.y) * (imgPts1[j].y - cvMs.y);
        // }
        // cout << "Reprojection error for cam1: " << reproj_error1 << endl;

        // imshow("Reprojection of cam1", reproj_im1);
        // int key1 = waitKey(10) % 256;
        
          
        if (bfind0 && bfind1){        
            vector<cv::Point2f> cam0_imgPtSet, cam1_imgPtSet;
            vector<cv::Point3f> cam0_objPtSet, cam1_objPtSet;
            num_viewused++;
            for (size_t j=0; j<tagid_found0.size(); j++){
                if (tagid_found0[j].first && tagid_found1[j].first){
                    int index_pt_cam0 = tagid_found0[j].second;
                    for (size_t k=index_pt_cam0; k<index_pt_cam0+4; k++){
                        cam0_imgPtSet.push_back(imgPts0[k]);
                        cam0_objPtSet.push_back(objPts0[k]);
                    }
                    int index_pt_cam1 = tagid_found1[j].second;
                    
                    for (size_t k=index_pt_cam1; k<index_pt_cam1+4; k++){
                        cam1_imgPtSet.push_back(imgPts1[k]);
                        cam1_objPtSet.push_back(objPts1[k]);
                       
                    }
                }
            }
            cam0_imgPts.push_back(cam0_imgPtSet);
            cam0_objPts.push_back(cam0_objPtSet);
            cam1_imgPts.push_back(cam1_imgPtSet);
            cam1_objPts.push_back(cam1_objPtSet);
            double object_pose0_mini[6], object_pose1_mini[6];

            miniparameter(object_pose_rvec0, object_pose_tvec0, object_pose0_mini);
            miniparameter(object_pose_rvec1, object_pose_tvec1, object_pose1_mini);

            for (size_t kk = 0; kk<6; kk++){
                poses0[iframe*6+kk] = object_pose0_mini[kk];
                poses1[iframe*6+kk] = object_pose1_mini[kk];
            }
        }

        
    }


    for (size_t i=0; i<23; i++){
        for (size_t j=0; j<6; j++){
            cout << poses0[6*i+j] <<" ";
        }
        cout << endl;
    }

    for (size_t i=0; i<23; i++){
        for (size_t j=0; j<6; j++){
            cout << poses1[6*i+j] <<" ";
        }
        cout << endl;
    }

    cout << "Totally " << num_viewused << " views used." << endl << endl; 
    // /*      
    //  * Step 2: Obtain camera-Camera estimated transform 
    //  */
    // initial guess of transform between cam0 and cam1

    double transform_array[6] = { -3.14, 0.000, 0.00, -0.00, 0.00, -0.195};

    cout << "--------------------------------------------" << endl;
    cout << "Initial value feed into Ceres Optimization program:" << endl;
    cout << " [rx (rad), ry (rad), rz (rad), tx (m), ty (m), tz (m)]" << endl;
    for (size_t i=0; i<6; i++)
        cout << transform_array[i] << " ";
    cout << endl << endl; 
    // /*
    //  * Step 3: Camera-camera transform optimization
    //  *  cost function: square sum of distance between scan points and chessboard plane
    //  *  state vector: rotation (3 parameters) and translation (3 parameters)
    //  */
    google::InitGoogleLogging("Bundle Adjustment");
    optimizer ba;    
    ba.bundleAdjustment(ocamcalib_cam0, cam0_imgPts, cam0_objPts, poses0, 
                        ocamcalib_cam1, cam1_imgPts, cam1_objPts, poses1,
                        transform_array);
    
    cout << "Optimized cam1 pose in cam0 frame " << endl;
    cout << " [rx (rad), ry (rad), rz (rad), tx (m), ty (m), tz (m)]:" << endl;
    for (size_t i=0; i<6; i++)
        cout << transform_array[i] << " ";
    cout << endl << endl;

    for (size_t i=0; i<23; i++){
        for (size_t j=0; j<6; j++){
            cout << poses0[6*i+j] <<" ";
        }
        cout << endl;
    }
    cout << endl;

    for (size_t i=0; i<23; i++){
        for (size_t j=0; j<6; j++){
            cout << poses1[6*i+j] <<" ";
        }
        cout << endl;
    }
    cout << "--------------------------------------------" << endl;
    return 0;
}
