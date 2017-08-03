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

    double width = 1050;//526;
    double height = 1050; //526;
    int tagRows = 10, tagCols = 7;
    double tagSize = 0.088; // unit: m
    double tagSpacing = 0.3; // unit: %

    // Camera intrinsics
    Mat cam0_proj = s.intrinsics0;
    Mat cam0_dist = s.distortion0;
    double cam0_xi =  s.xi0;
    Mat cam1_proj = s.intrinsics1;
    Mat cam1_dist = s.distortion1;
    double cam1_xi =  s.xi1;
    OmniModel cam0(cam0_proj, cam0_dist, cam0_xi);
    OmniModel cam1(cam1_proj, cam1_dist, cam1_xi);
    cout << "Cam0 intrinsic matrix: " << endl << cam0_proj << endl << endl;
    cout << "Distortion coefficients: " << endl << cam0_dist << endl << endl;
    cout << "Mirror parameter: " << endl << cam0_xi << endl  ;
    cout << "---------------------------------------------" << endl;
    cout << "Cam1 intrinsic matrix: " << endl << cam1_proj << endl << endl;
    cout << "Distortion coefficients: " << endl << cam1_dist << endl << endl;
    cout << "Mirror parameter: " << endl << cam1_xi << endl  ;
    cout << "---------------------------------------------" << endl;

    // OmniModel model(camera_matrix, dist_coeffs, xi);
    vector<vector<KeyPoint> > kps_vec_1, kps_vec_2;
    
    AprilTagsDetector apriltags0(cam0_proj, cam0_dist, cam0_xi, 
                                 width, height, 
                                 tagRows, tagCols,
                                 tagSize, tagSpacing,
                                 string("cam0_apriltags_detection"));
    AprilTagsDetector apriltags1(cam1_proj, cam1_dist, cam1_xi, 
                                 width, height, 
                                 tagRows, tagCols,
                                 tagSize, tagSpacing,
                                 string("cam1_apriltags_detection"));

    vector<Eigen::Matrix4d> tagPoses0, tagPoses1;
    vector<cv::Point2f> cam0_imgPts;
    vector<cv::Point3f> cam1_objPts;

    size_t num_viewused = 0;
    for (size_t i=0; i<im0_seq.size(); i++){
        /*
         * Step 1: Find out camera extrinsic parameter using PnP
         */
        // image pre-processing: 
        //      convert to gray scale
        //      undistort fisheye camera image
        Mat im0 = im0_seq[i], im1 = im1_seq[i];
        
        // if (im0.channels() == 3)
        //     cvtColor(im0, im0, COLOR_RGB2GRAY);
        // if (im1.channels() == 3)
        //     cvtColor(im1, im1, COLOR_RGB2GRAY);

        Eigen::Matrix4d cam0_pose, cam1_pose;
        vector<AprilTags::TagDetection> detections0, detections1;
        vector<cv::Point3f> objPts0, objPts1;
        vector<cv::Point2f> imgPts0, imgPts1;
        vector<std::pair<bool, int> >tagid_found0, tagid_found1;
        
        // im0 = imread("/home/audren/Documents/data/small_drone_v2/AprilGrid_7x10_with_black_squares_A0.tif", 0);
        bool bfind0 = apriltags0.getDetections(im0, detections0, objPts0, imgPts0, tagid_found0);
        // for (size_t j=0; i<objPts0.size(); i++){
        //     cout << objPts0[i].x << " " << objPts0[i].y << " " << objPts0[i].z << endl; 
        //     cout << imgPts0[i].x << " " << imgPts0[i].y << endl << endl;
        // }
        // for (size_t j=0; j<tagid_found0.size(); j++)
        //     cout << tagid_found0[j].first << " " << j << endl;
        // // cout << endl;
        // return 0;
        // waitKey(1);
        bool bfind1 = apriltags1.getDetections(im1, detections1, objPts1, imgPts1, tagid_found1);
        // waitKey(1);

        if (bfind0 && bfind1){
            apriltags0.findCamPose(objPts0, imgPts0, cam0_pose);
            apriltags1.findCamPose(objPts1, imgPts1, cam1_pose);
           

            cout << endl;
            cout << "--------------------------transform:------------------------------------- " << endl;
            Eigen::Matrix4d cam_transform = cam0_pose.inverse() * cam1_pose;
            cout << cam_transform << endl;
            cout << cam0_pose << endl;
            cout << cam1_pose << endl;
            // cout << " Debug info: " << endl;
            // cout << "obj pt num: "  << objPts1.size() << endl;
            // cout << "img pt num: " << imgPts1.size() << endl;
            // cout << cam1_pose << endl;
            Eigen::Matrix4d target_pose_in_cam1 = cam1_pose.inverse();
            // cout << "target pose in cam1: " << endl;
            // cout << target_pose_in_cam1 << endl;
            if ( isnan(target_pose_in_cam1(0,0)) ) continue;
            num_viewused++;
            for (size_t j=0; j<tagid_found0.size(); j++){
                if (tagid_found0[j].first && tagid_found1[j].first){
                    // store cam0 image points
                    int index_pt_cam0 = tagid_found0[j].second;
                    for (size_t k=index_pt_cam0; k<index_pt_cam0+4; k++){
                        cam0_imgPts.push_back(imgPts0[k]);
                        // cout << "Image points:" << endl;
                        // cout << imgPts0[k].x << " " << imgPts0[k].y << endl;
                    }
                    // transform cam1 obj points (in target frame) to cam1 frame
                    int index_pt_cam1 = tagid_found1[j].second;
                    
                    // cout << "Cam pose: " << endl;
                    // cout << cam1_pose << endl;
                    // cout << target_pose_in_cam1 << endl;
                    // cout << "          " << endl << endl;
                    for (size_t k=index_pt_cam1; k<index_pt_cam1+4; k++){
                        cv::Point3f cv_pt = objPts1[k];
                        // cout << "ma: " << cv_pt.x << " " << cv_pt.y << " " << cv_pt.z << endl;
                        Eigen::Vector4d pt(cv_pt.x, cv_pt.y, cv_pt.z, 1);

                        // cout << "ha:" << endl;
                        // cout << pt(0) << " " << pt(1) << " " << pt(2) << endl;
                        // cout << target_pose_in_cam1 << endl;
                        Eigen::Vector4d new_pt = target_pose_in_cam1 * pt;
                        cam1_objPts.push_back(cv::Point3f(new_pt(0), new_pt(1), new_pt(2)));
                        // cout << new_pt(0) << " " << new_pt(1) << " " << new_pt(2) << endl;
                        // cout << cv::Point3f(new_pt(0), new_pt(1), new_pt(2)) << endl;
                    }
                }
            }
        }
    }
    // return 0;
    cout << "Totally " << num_viewused << " views used." << endl << endl; 
    // /*      
    //  * Step 2: Obtain camera-Camera estimated transform 
    //  */
    // initial guess of transform between cam0 and cam1
    double transform_array[6] = { 3.14, 0.0, 0.0, 0.0, 0.0, 0.06};
    cout << "--------------------------------------------" << endl;
    cout << "Initial value feed into Ceres Optimization program:" << endl;
    cout << " [rx (rad), ry (rad), rz (rad), tx (m), ty (m), tz (m)]" << endl;
    for (size_t i=0; i<6; i++)
        cout << transform_array[i] << " ";
    cout << endl << endl; 
    // /*
    //  * Step 3: Lidar-camera transform optimization
    //  *  cost function: square sum of distance between scan points and chessboard plane
    //  *  state vector: rotation (4 parameters) and translation (3 parameters)
    //  *  constraint: scan points should fall in the range of chessboard (x, y)
    //  */
    google::InitGoogleLogging("Bundle Adjustment");
    optimizer ba;    
    ba.bundleAdjustment(apriltags0.camModel, cam0_imgPts, cam1_objPts, transform_array);
    
    cout << "Optimized cam1 pose in cam0 frame " << endl;
    cout << " [rx (rad), ry (rad), rz (rad), tx (m), ty (m), tz (m)]:" << endl;
    for (size_t i=0; i<3; i++)
        cout << transform_array[i] << " ";
    for (size_t i=3; i<6; i++)
        cout << transform_array[i] << " ";
    cout << endl;
    cout << "--------------------------------------------" << endl;
    return 0;
}
