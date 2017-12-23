// #include <iostream>

// #include <opencv2/opencv.hpp>
// #include <opencv2/calib3d/calib3d.hpp>
// #include <opencv2/highgui/highgui.hpp>

// #include "ros/ros.h"
// #include "ceres/rotation.h"

// #include "camera_camera_calib/loadBag.h"
// #include "camera_camera_calib/loadSettings.h"
// #include "camera_camera_calib/objectPose.h"
// #include "camera_camera_calib/optimizerRay.h"
// #include "camera_camera_calib/omniModel.h"
// #include "camera_camera_calib/aprilTagsDetector.h"
// #include "camera_camera_calib/ocamCalibModel.h"


// using namespace std;
// using namespace cv;

// /*
//  * Lidar-camera extrinsic parameter calibration
//  */
// int main(int argc, char **argv)
// {
//     /* 
//      * Load image, convert from ROS image format to OpenCV Mat
//      */
//     ros::init(argc, argv, "camera_camera_calib");    
//     string bag_file("/home/audren/Documents/data/small_drone_v2/ufo_2017-08-01-19-58-02.bag");

//     vector<Mat> im0_seq, im1_seq;
//     string topic0 = string("/synthetic_gimbal/cam0") + "/image_raw";
//     string topic1 = string("/synthetic_gimbal/cam1") + "/image_raw";
//     size_t sample_num = 10;
//     size_t max_im_num = 500;
//     loadBag(bag_file, topic0, topic1, im0_seq, im1_seq, sample_num, max_im_num);

//     if (im0_seq.size() != im1_seq.size() || im0_seq.size() < 10){
//         cout << "Inconsistent image numbers or too few images!" << endl;
//         return 0;
//     }
//     cout << "---------------------------------------------" << endl;
//     cout << "Number of left images:   " << im0_seq.size() << endl;
//     cout << "Number of right images:  " << im1_seq.size() << endl;
//     cout << "---------------------------------------------" << endl;
//     /*
//      * Read setting files
//      */
//     Settings s;
//     string inputSettingsFile("/home/audren/Documents/lidar-camera-calib/src/camera_camera_calib/settings/settings.xml");
//     FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
//     if (!fs.isOpened())
//     {
//         cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
//         return -1;
//     }
//     fs["Settings"] >> s;
//     fs.release();                                         // close Settings file

//     if (!s.goodInput)
//     {
//         cout << "Invalid input detected. Application stopping. " << endl;
//         return -1;
//     }

//     double width = 1050;//526;
//     double height = 1050; //526;
//     int tagRows = 10, tagCols = 7;
//     double tagSize = 0.088; // unit: m
//     double tagSpacing = 0.25; // unit: %

//     // Camera intrinsics
//     std::vector<double> cam0_ss;
//     double temp[] = {-2.575876e+02, 0.000000e+00, 2.283578e-04, 8.908668e-06, -2.621133e-08, 3.037693e-11 };
//     for (size_t i=0; i<6; i++)
//         cam0_ss.push_back(temp[i]);

//     double cam0_u0 = 532.425687;
//     double cam0_v0 = 517.382409;
//     double cam0_c = 1.000805;
//     double cam0_d = 0.000125;
//     double cam0_e = 2.5200e-04;

//     std::vector<double> cam1_ss;
//     double temp1[] = {-2.593081e+02, 0.000000e+00, 4.238530e-04, 7.434385e-06, -2.212919e-08, 2.642407e-11};
//     for (size_t i=0; i<6; i++)
//         cam1_ss.push_back(temp1[i]);

//     double cam1_u0 = 512.560101;
//     double cam1_v0 = 523.645938;
//     double cam1_c = 1.001192;
//     double cam1_d = 0.000227;
//     double cam1_e = 0.000224;

//     Mat cam0_proj = s.intrinsics0;
//     Mat cam0_dist = s.distortion0;
//     double cam0_xi =  s.xi0;
//     Mat cam1_proj = s.intrinsics1;
//     Mat cam1_dist = s.distortion1;
//     double cam1_xi =  s.xi1;

//     OmniModel cam0(cam0_proj, cam0_dist, cam0_xi, cam0_u0, cam0_v0);
//     OmniModel cam1(cam1_proj, cam1_dist, cam1_xi, cam1_u0, cam1_v0);
//     cout << "Cam0 intrinsic matrix: " << endl << cam0_proj << endl << endl;
//     cout << "Distortion coefficients: " << endl << cam0_dist << endl << endl;
//     cout << "Mirror parameter: " << endl << cam0_xi << endl  ;
//     cout << "---------------------------------------------" << endl;
//     cout << "Cam1 intrinsic matrix: " << endl << cam1_proj << endl << endl;
//     cout << "Distortion coefficients: " << endl << cam1_dist << endl << endl;
//     cout << "Mirror parameter: " << endl << cam1_xi << endl ;
//     cout << "---------------------------------------------" << endl;

//     AprilTagsDetector apriltags0(cam0_u0, cam0_v0, 
//                                  cam0_proj.at<double>(0,0), cam0_proj.at<double>(1,1), 
//                                  width, height, 
//                                  tagRows, tagCols,
//                                  tagSize, tagSpacing,
//                                  string("cam0_apriltags_detection"));
//     AprilTagsDetector apriltags1(cam1_u0, cam1_v0, 
//                                  cam1_proj.at<double>(0,0), cam1_proj.at<double>(1,1), 
//                                  width, height, 
//                                  tagRows, tagCols,
//                                  tagSize, tagSpacing,
//                                  string("cam1_apriltags_detection"));

//     vector<vector<cv::Point3f> > cam0_rays;
//     vector<vector<cv::Point3f> > cam1_rays;

//     size_t num_viewused = 0;
//     OCamCalibModel ocamcalib_cam0;
//     char ocamfile0[] = "/home/audren/Documents/data/small_drone_v2/Dart_21905596_high_res/calib_results_dart_21905596_high_res.txt";
//     bool bopen0 = ocamcalib_cam0.get_ocam_model(ocamfile0);

//     OCamCalibModel ocamcalib_cam1;
//     char ocamfile1[] = "/home/audren/Documents/data/small_drone_v2/Dart_21905597_high_res/calib_results_dart_21905597_high_res.txt";
//     bool bopen1 = ocamcalib_cam1.get_ocam_model(ocamfile1);
   
//     int good_array[] = {24, 25, 26, 27, 30, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 49, 50, 51, 52, 53, 54};
//     std::vector<int> good_frame (good_array, good_array + sizeof(good_array) / sizeof(int) );

//     for (size_t i=0; i<im0_seq.size(); i++){
//     // for (size_t iframe=0; iframe<good_frame.size(); iframe++){
//         //int i = good_frame[iframe];
//         /*
//          * Step 1: Find out camera extrinsic parameter using PnP
//          */
//         Mat im0 = im0_seq[i], im1 = im1_seq[i];
        
//         if (im0.channels() == 3)
//             cvtColor(im0, im0, COLOR_RGB2GRAY);
//         if (im1.channels() == 3)
//             cvtColor(im1, im1, COLOR_RGB2GRAY);

//         Mat reproj_im0 = im0.clone();
//         cv::cvtColor(im0, reproj_im0, cv::COLOR_GRAY2BGR);
//         Mat reproj_im1 = im1.clone();
//         cv::cvtColor(im1, reproj_im1, cv::COLOR_GRAY2BGR);

//         Eigen::Matrix4d object_pose0, object_pose1;
//         vector<AprilTags::TagDetection> detections0, detections1;
//         vector<cv::Point2f> imgPts0, imgPts1;
//         vector<cv::Point3f> objPts0, objPts1;
//         vector<std::pair<bool, int> >tagid_found0, tagid_found1;
        
//         bool bfind0 = apriltags0.getDetections(im0, detections0, objPts0, imgPts0, tagid_found0);
//         waitKey(10);
//         bool bfind1 = apriltags1.getDetections(im1, detections1, objPts1, imgPts1, tagid_found1);
//         waitKey(10);
        
//         if (bfind0 && bfind1){        
//             num_viewused++;
//             for (size_t j=0; j<tagid_found0.size(); j++){
//                 if (tagid_found0[j].first && tagid_found1[j].first){
//                     int index_pt_cam0 = tagid_found0[j].second;

//                     vector<cv::Point3f> vt0, vt1;
//                     for (size_t k=index_pt_cam0; k<index_pt_cam0+4; k++){
//                         double imgpt[2] = {imgPts0[k].x, imgPts0[k].y};
//                         double objpt[3];
//                         ocamcalib_cam0.cam2world(objpt, imgpt);
//                         vt0.push_back(cv::Point3f(objpt[0], objpt[1], objpt[2]));
//                     }
//                     cam0_rays.push_back(vt0);
//                     int index_pt_cam1 = tagid_found1[j].second;
//                     for (size_t k=index_pt_cam1; k<index_pt_cam1+4; k++){
//                         double imgpt[2] = {imgPts1[k].x, imgPts1[k].y};
//                         double objpt[3];
//                         ocamcalib_cam1.cam2world(objpt, imgpt);
//                         vt1.push_back(cv::Point3f(objpt[0], objpt[1], objpt[2]));
//                     }
//                     cam1_rays.push_back(vt1);
//                 }
//             }
//         }
//     }
   
//     /*
//      * test triangulate: calculate the distance between two vectors
//      */
//     // Eigen::Matrix<double, 3, 1> point1, point2;
//     // point1 << 0,0, 0;
//     // point2 << 0, 0, 1;
    
//     // Eigen::Matrix<double, 3, 1> ray1, ray2;
//     // ray1 << 0, 1, 0;
//     // ray2<< 1, 1 , 1 ;

//     // Eigen::Matrix<double, 3, 1> outTriangulatedPoint;
//     // double outGap;
//     // double outS0, outS1;
//     // ocamcalib_cam0.triangulate(point1, ray1, point2, ray2, outTriangulatedPoint, outGap, outS0, outS1);
//     // cout << endl;
//     // cout << "distance between two rays: " << outGap << endl;
//     // cout << endl;
//     // return 0;

//     cout << "Totally " << num_viewused << " views used." << endl << endl; 
//     // /*      
//     //  * Step 2: Obtain camera-Camera estimated transform 
//     //  */
//     // initial guess of transform between cam0 and cam1
//     double transform_array[6] = { -3.14051, 0.186225, -0.042514, -0.0498732, 0.0650859, -0.193217};
//     cout << "--------------------------------------------" << endl;
//     cout << "Initial value feed into Ceres Optimization program:" << endl;
//     cout << " [rx (rad), ry (rad), rz (rad), tx (m), ty (m), tz (m)]" << endl;
//     for (size_t i=0; i<6; i++)
//         cout << transform_array[i] << " ";
//     cout << endl << endl; 
//     // /*
//     //  * Step 3: Camera-camera transform optimization
//     //  *  cost function: square sum of distance between scan points and chessboard plane
//     //  *  state vector: rotation (3 parameters) and translation (3 parameters)
//     //  */
//     google::InitGoogleLogging("Bundle Adjustment");
//     optimizer ba;    
//     ba.bundleAdjustment(ocamcalib_cam0, cam0_rays, cam1_rays, transform_array);
    
//     cout << "Optimized cam1 pose in cam0 frame " << endl;
//     cout << " [rx (rad), ry (rad), rz (rad), tx (m), ty (m), tz (m)]:" << endl;
//     for (size_t i=0; i<3; i++)
//         cout << transform_array[i] << " ";
//     for (size_t i=3; i<6; i++)
//         cout << transform_array[i] << " ";
//     cout << endl;
//     cout << "--------------------------------------------" << endl;
//     return 0;
// }
