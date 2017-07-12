#include <iostream>
#include <boost/unordered_map.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ceres/rotation.h"

#include "lidar_camera_calib/loadBag.h"
#include "lidar_camera_calib/loadSettings.h"
#include "lidar_camera_calib/objectPose.h"
#include "lidar_camera_calib/omniModel.h"
#include "lidar_camera_calib/optimizer.h"
#include "lidar_camera_calib/hash.h"

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
    ros::init(argc, argv, "my_scan_to_cloud");    
    string bag_file("/home/audren/lidar_camera_calib/data/cameraLidarData.bag");
    vector<Mat> image_queue;
    vector<unsigned long> time_queue;
    vector<vector<Point3f> > lidar_queue;
    loadBag(bag_file, image_queue, time_queue, lidar_queue);
    // TODO: test if obtained LIDAR scan points are correct

    cout << "Number of images: " << image_queue.size() << '\n';
    cout << "Number of LIDAR scan: " << lidar_queue.size() << endl;
    cout << "---------------------------------------------" << endl;

    /*
     * Read setting files
     */
    Settings s;
    string inputSettingsFile("/home/audren/lidar_camera_calib/calib_ws/src/lidar_camera_calib/include/settings.xml");
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
    Mat camera_matrix = s.intrinsics;
    Mat dist_coeffs = s.distortion;
    double xi =  s.xi;
    cout << "Camera intrinsic matrix: " << endl << camera_matrix << endl ;
    cout << "Distortion coefficients: " << endl << dist_coeffs << endl;
    cout << "Mirror parameter: " << endl << xi << endl;
    cout << "---------------------------------------------" << endl;

    Size patternsize(7, 10); // interior number of corners
    float squareSize = 98.00; // unit: mm
    OmniModel model(camera_matrix, dist_coeffs, xi);
    vector<Eigen::Matrix4d> object_poses;
    vector<vector<Point3f> > lidar_scan_points;
    for (size_t i=0; i<lidar_queue.size(); i+=10){
        /*
         * Step 1: Find out camera extrinsic parameter using PnP
         */
        // image pre-processing: 
        //      convert to gray scale
        //      undistort fisheye camera image
        Mat image = image_queue[i];
        
        if (image.channels() == 3)
            cvtColor(image, image, COLOR_RGB2GRAY);
        
        Mat gray(image);
        
        // corners in world frame (origin is on the upper left chessboard)
        vector<Point3f> worldCorners;
        calcBoardCornerPositions(patternsize, s.squareSize, worldCorners,
                  Settings::CHESSBOARD);

        // test: good
        // for (int i=0; i<worldCorners.size(); i++){
        //     cout << worldCorners[i].x <<" " << worldCorners[i].y <<" " << worldCorners[i].z <<" ";
        //     cout <<endl;
        // }

        // corners in image frame
        vector<Point2f> imageCorners;
        bool find_corners = findBoardCorner(gray, patternsize, imageCorners, true);
        // test: good

        // find object pose using PnP
        //     Input iamge points and object points
        //     Output rotation and translation
        if (find_corners){
            Mat rvec; // Rotation in axis-angle form
            Mat tvec;
            Eigen::Matrix4d pose;
            model.estimateTransformation(imageCorners, worldCorners, pose);
            object_poses.push_back(pose);
            lidar_scan_points.push_back(lidar_queue[i]);
            // cout << pose(0,3) << " " << pose(1,3) << " " << pose(2,3) << endl;
        }
        // TODO: test if the pose is correct
    }
    /*      
     * Step 2: Obtain LIDAR-Camera estimated transform 
     */
    // initial guess of lidar transform in camera frame
    Mat init_rvec = s.initialRotation; // 4 by 1, quaternion
    Mat init_tvec = s.initialTranslation; // 3 by 1
    
    double transform[6];
    transform[0] = 0;
    transform[1] = 1.047198;
    transform[2] = -0.339816;
    transform[3] = 39.91;
    transform[4] = 35.38;
    transform[5] = 102.19;
    for (size_t i=0; i<6; i++)
        cout << transform[i] << " ";
    cout << endl;
    /*
     * Step 3: Lidar-camera transform optimization
     *  cost function: square sum of distance between scan points and chessboard plane
     *  state vector: rotation (4 parameters) and translation (3 parameters)
     *  constraint: scan points should fall   in the range of chessboard (x, y)
     */
    google::InitGoogleLogging("Bundle Adjustment");
    optimizer ba;    
    ba.bundleAdjustment(lidar_scan_points, object_poses, patternsize, squareSize, 100 /*cube depth*/, transform, init_rvec, init_tvec);

    cout << "Transform from camera to optimized LIDAR frame: " << endl;
    for (size_t i=0; i<6; i++)
        cout << transform[i] << " ";
    cout << endl;

    Mat rvec = Mat(3, 1, CV_64F);
    Mat tvec = Mat(3, 1, CV_64F);
    Mat rmatrix, new_rmatrix;
    rvec.at<double>(0,0) = transform[0];
    rvec.at<double>(1,0) = transform[1];
    rvec.at<double>(2,0) = transform[2];
    tvec.at<double>(0,0) = transform[3];
    tvec.at<double>(1,0) = transform[4];
    tvec.at<double>(2,0) = transform[5];
    cv::Rodrigues(rvec, rmatrix);
    transpose(rmatrix, new_rmatrix);
    tvec = - new_rmatrix * tvec;
    cv::Rodrigues(new_rmatrix, rvec);
    
    cout << "--------------------------------------------" << endl;
    cout << "Calibration Done!" << endl;
    cout << "The estimated transform between LIDAR and camera: " << endl;
    // for (size_t i=0; i<6; i++)
    //     cout << transform[i] << " ";
    for (size_t i=0; i<3; i++)
        cout << rvec.at<double>(i, 0) << " ";
    cout << endl;
    for (size_t i=0; i<3; i++)
        cout << tvec.at<double>(i, 0) << " ";
    cout << endl;
    return 0;
}
