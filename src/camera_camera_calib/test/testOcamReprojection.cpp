#include <iostream>
#include <iterator>
#include <random>
#include <chrono>

#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

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

bool SYNTHTIC = false;

std::pair<double, double> getXYZ(double squareDist, 
                                int id, 
                                int m_tagRows, 
                                int m_tagCols){
  double x = ( id % (m_tagCols+1) ) * squareDist;
  double y = ( id / (m_tagCols+1) ) * squareDist;
  
  return std::pair<double, double>(x, y);
}

cv::Point3d pointTransform(const cv::Point3d& p0, 
                            const Eigen::Matrix4d& transform){
    Eigen::Vector4d eigen_p0;
    eigen_p0 << p0.x, p0.y, p0.z, 1;
    Eigen::Vector4d eigen_p1 = transform * eigen_p0;
    return cv::Point3d(eigen_p1(0), eigen_p1(1), eigen_p1(2));
}

cv::Point2d targetPoint2ImagePixel(const OCamCalibModel& cam, 
                                   const cv::Point3d& p0, 
                                   const Eigen::Matrix4d& target_pose){
    cv::Point3d p1 = pointTransform(p0, target_pose);

    double Ps[3] = {p1.x, p1.y, p1.z};
    double Ms[2];
    cam.world2cam(Ms, Ps);

    return cv::Point2d(Ms[0], Ms[1]);
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
    string bag_file("../data/small_drone_v2/ufo_2017-08-01-19-58-02.bag");

    vector<Mat> im0_seq, im1_seq;
    string topic0 = string("/synthetic_gimbal/cam0") + "/image_raw";
    string topic1 = string("/synthetic_gimbal/cam1") + "/image_raw";
    size_t sample_num = 10;
    size_t max_im_num = 500;
    loadBag(bag_file, topic0, topic1, 
        im0_seq, im1_seq, sample_num, max_im_num);

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
    AprilTagOcamConfig s;
    string inputSettingsFile("./src/camera_camera_calib/settings/"
                            "settings_apriltag_ocam.xml");
    FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"" 
             << inputSettingsFile 
             << "\"" 
             << endl;
        return -1;
    }
    fs["Settings"] >> s;
    fs.release();                                         // close Settings file

    if (!s.goodInput)
    {
        cout << "Invalid input detected. Application stopping. " << endl;
        return -1;
    }

    int tagRows = s.boardSize.height, tagCols = s.boardSize.width;
    double tagSize = s.tagSize/1000; // unit: m
    double tagSpacing = s.tagSpace; // unit: %
    int width = s.imageWidth;
    int height = s.imageHeight;


    

    /*
     * Read camera parameters
     */
    OCamCalibModel ocamcalib_cam0;
    char ocamfile0[] = "../data/small_drone_v2/Dart_high_res/"
                        "calib_results_dart_21905596_high_res.txt";
    bool bopen0 = ocamcalib_cam0.get_ocam_model(ocamfile0);

    OCamCalibModel ocamcalib_cam1;
    char ocamfile1[] = "../data/small_drone_v2/Dart_high_res/"
                        "calib_results_dart_21905597_high_res.txt";
    bool bopen1 = ocamcalib_cam1.get_ocam_model(ocamfile1);




    /*
     * Pattern detection and pose estimation
     */
    vector<cv::Mat> &im_seq = im0_seq;
    OCamCalibModel &cam = ocamcalib_cam0;
    AprilTagsDetector apriltags(tagRows, tagCols,
                                 tagSize, tagSpacing,
                                 string("apriltags_detection"));
    if (!SYNTHTIC){
    for (size_t i=0; i<im_seq.size(); i++){
        /*
         * Find out camera extrinsic parameter using PnP
         */
        Mat im = im_seq[i];
        if (im.channels() == 3)
            cvtColor(im, im, COLOR_RGB2GRAY);

        Eigen::Matrix4d target_pose;
        vector<AprilTags::TagDetection> detections;
        vector<cv::Point3d> objPts;
        vector<cv::Point2d> imgPts;
        vector<std::pair<bool, int> >tagid_found;
        
        Mat reproj_im = im.clone();
        cv::cvtColor(im, reproj_im, cv::COLOR_GRAY2BGR);

        bool bfind = apriltags.getDetections(im, detections, 
                                            objPts, imgPts, 
                                            tagid_found);
        bool good_estimation = cam.findCamPose(imgPts, 
                                            objPts, 
                                            target_pose);
        // cout << target_pose << endl;
        // waitKey(0.1);
        // cout << "objPts number: " << objPts.size() << endl;




        /*
         * test 1: cam2world and world2cam
         */
        int TESTNO = 2;
        if (TESTNO == 1){   // verify projection function
                            // cam2world and world2cam
            for (size_t j=0; j<objPts.size(); j++){
                double Ms[2] = {imgPts[j].x, imgPts[j].y};
                double Ps[3];
                cam.cam2world(Ps, Ms);
                cam.world2cam(Ms, Ps);
                
                cv::circle(reproj_im, 
                        cv::Point2d(imgPts[j].x, imgPts[j].y), 
                        1, 
                        cv::Scalar(0,255,14,0), 
                        1);
                cv::circle(reproj_im, 
                        cv::Point2d(Ms[0], Ms[1]), 
                        5, 
                        cv::Scalar(255,0,0,0), 
                        1);
            }
        }else{
            /*
             * test 2: pose estimation
             */
            for (size_t j=0; j<objPts.size(); j++){
                cv::Point2d pt = targetPoint2ImagePixel(cam, objPts[j], target_pose);
        
                cv::circle(reproj_im, 
                        cv::Point2d(imgPts[j].x, imgPts[j].y), 
                        1, 
                        cv::Scalar(0,255,14,0),  // green, detections 
                        1);
                cv::circle(reproj_im, 
                        cv::Point2d(pt.x, pt.y), 
                        5, 
                        cv::Scalar(255,0,0,0),  // blue, reprojectiona
                        2);
            }
        }
        imshow("Reprojection", reproj_im);
        waitKey(0);

    }
    }




    else{
    /*
     * test: using synthetic data 
     *
     * a.
     * input: given points on the AprilTag (in apriltag frame)
     *        and pose of the AprilTag (in camera frame)
     * output: corresponding image points using ocam camera model  
     *         show these image points
     *
     * b.
     * input: given scene points and image points (from a output)
     *        estimate pose of AprilTag (without noise)
     *        estimate pose of AprilTag (Gaussian noise) 
     * test pass
     */
    // a. assume a pose beforehand, and project points on AprilTag
    //    on image
    Eigen::Matrix4d pose_ground_truth;
    pose_ground_truth << 1,     0,     0,   -1,
                         0, 0.866,   0.5,     0.3,
                         0,  -0.5, 0.866,   0.5,
                         0,     0,     0,     1;
    Eigen::Matrix3d rotation_ground_truth = pose_ground_truth.block<3,3>(0, 0);
    Eigen::Vector3d translation_ground_truth = pose_ground_truth.block<3,1>(0,3);
    std::cout << rotation_ground_truth << std::endl;
    std::cout << translation_ground_truth << std::endl;
    // pose_ground_truth << 1, 0, 0, 0,
    //                      0, 1, 0, 0,
    //                      0, 0, 1, 0.5,
    //                      0, 0, 0, 1;


    double squareDist = tagSize + tagSize * tagSpacing;
    double halfSquare = tagSize / 2.0;
    vector<cv::Point3d> objPts;
    vector<std::pair<double, double> > vec_center;
    Mat img(width, height, CV_8UC3, Scalar(0,0,0));
    for (size_t i=0; i<80; i++){
        std::pair<double, double> center = getXYZ(squareDist, i, 10, 7);
        vec_center.push_back(center);

        double cx = center.first;
        double cy = center.second;

        /*
         * draw ID
         */
        cv::Point2d center_i = 
        targetPoint2ImagePixel(cam, cv::Point3d(cx, cy, 0), 
                                pose_ground_truth);
        std::ostringstream strSt;
        strSt << "#" << i;
        cv::putText(img, strSt.str(),
                  center_i,
                  cv::FONT_HERSHEY_PLAIN, 
                  1, cv::Scalar(0,0,255));


        /* pass
        * cout << "-------------------debug------------------" << endl;
        * cout << cx << " " << cy << endl;
        */
        objPts.push_back(cv::Point3d(cx - halfSquare, cy - halfSquare, 0));
        objPts.push_back(cv::Point3d(cx + halfSquare, cy - halfSquare, 0));
        objPts.push_back(cv::Point3d(cx + halfSquare, cy + halfSquare, 0));
        objPts.push_back(cv::Point3d(cx - halfSquare, cy + halfSquare, 0));
    }



    // show ID of each tag
    vector<cv::Point2d> imgPts;
    for (size_t i=0; i<objPts.size(); i++){
        cv::Point2d ipt = 
        targetPoint2ImagePixel(cam, objPts[i], pose_ground_truth);
        imgPts.push_back(ipt);
        /*
         * orientation is correct
         */
        cv::circle(img, cv::Point2d(ipt.x, ipt.y), 1, cv::Scalar(255,255,255,0), 1);
    }

    imshow("Reprojection of AprilTag Points", img);
    waitKey(0);




    /* b. from synthetic image points and object points 
     *    recover pose of target in camera frame
     *
     * test with gaussian noise 
     *       with zero mean, and variance 0.0, 0.1, ..., 1.0
     */

    // construct a trivial random generator engine from a time-based seed:
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);

    for (size_t i=0; i<11; ++i){
        std::normal_distribution<double> distribution (0.0,0.1*i);
        std::cout << "Normal-distributed(0.0," 
                  << 0.1*i 
                  << ") results:" << std::endl;

        double rotation_error = 0;
        double translation_eror = 0;
        size_t num_exp = 20;
        for (size_t j=0; j<num_exp; j++){
            vector<cv::Point2d> noise_imgPts = imgPts;
            for (size_t k=0; k<noise_imgPts.size(); k++){
                noise_imgPts[k].x += distribution(generator);
                noise_imgPts[k].y += distribution(generator); 
            }
            
            Eigen::Matrix4d pose_estimated;
            bool good_est = cam.findCamPose(noise_imgPts, objPts, pose_estimated);

            // errors of rotation matrixS
            Eigen::Matrix3d rotation_estimated = pose_estimated.block<3,3>(0, 0);
            Eigen::Matrix3d delta_rotation = rotation_estimated 
                            * rotation_ground_truth.transpose();

            cv::Mat delta_rotation_cv(3, 3, CV_64F);
            cv::eigen2cv(delta_rotation, delta_rotation_cv);                
            cv::Mat rvec_estimted(3, 1, CV_64F);
            cv::Rodrigues(delta_rotation_cv, rvec_estimted);
            rotation_error += cv::norm(rvec_estimted);

            // errors of translation vectors
            Eigen::Vector3d translation_estimated = pose_estimated.block<3,1>(0, 3);
            Eigen::Vector3d delta_translation = translation_ground_truth 
                                            - translation_estimated;
            translation_eror += delta_translation.norm();                                            

        }

        std::cout << "average rotation error (in radian): " 
                  << rotation_error/num_exp << std::endl
                  << "average translation error (in meter): "
                  << translation_eror/num_exp << std::endl
                  << std::endl;
    }


    /* 
     * c. test analytical solutions to the extrinsic paramters
     *    recover pose of target in camera frame
     *  
     */
    cv::Mat rvec, tvec;
    bool good_est = cam.solveCamPose(imgPts, objPts, rvec, tvec);

    }
    
    return 0;
}
