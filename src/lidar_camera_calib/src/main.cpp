#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ceres/rotation.h"

#include "lidar_camera_calib/loadBagFile.h"
#include "lidar_camera_calib/loadSettings.h"
#include "lidar_camera_calib/objectPose.h"
#include "lidar_camera_calib/omniModel.h"
//#include "find_camera_pose/optimizer.h"
// Commented out to bypass compilation error stating the header file "map.h" in the header file "optimizer.h" could not be found.


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
    string bagFile("/home/audren/lidar_camera_calib/data/cameraLidarData.bag");
    vector<Mat> image_queue;
    vector<vector<Point3f> > lidar_queue;
    loadBag(bagFile, image_queue, lidar_queue);
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
    for (int i=0; i<1; i++){
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
        findBoardCorner(gray, patternsize, imageCorners, true);
        // test: good

        // find object pose using PnP
        //     Input camera matrix and distortion coefficient
        //     Output rotation and translation
        Mat rvec; // Rotation in axis-angle form
        Mat tvec;
        Eigen::Matrix4d camera_pose;
        model.estimateTransformation(imageCorners, worldCorners, camera_pose);
        cout << "Camera pose: " << endl;
        cout << camera_pose << endl;
        cout << "---------------------------------------------" << endl;
        /*      
         * Step 2: Obtain lidar scan in camera frame
         */
        // initial guess of lidar transform in camera frame
        Mat init_rvec = s.initialRotation; // 4 by 1, quaternion
        Mat init_tvec = s.initialTranslation; // 3 by 1


        /*
         * Step 3: Lidar-camera transform optimization
         */
        // cost function: square sum of distance between scan points and chessboard plane
        // state vector: rotation (4 parameters) and translation (3 parameters)
        // constraint: scan points should fall   in the range of chessboard (x, y)


        /* 
        // Commented out to bypass compilation error stating the header file "map.h" in the header file "optimizer.h" could not be found.
        google::InitGoogleLogging("Local bundle Adjustment");
        optimizer ba;
        ba.bundleAdjustment(local_map, parameter, P1.at<double>(0,0), P1.at<double>(1,1), P1.at<double>(0,2), P1.at<double>(1,2));
        */



        
    }
    cout << "Calibration Done!" << endl;
    return 0;
}
