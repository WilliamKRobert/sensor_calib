#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include "ceres/rotation.h"

#include <sstream>
#include <boost/unordered_map.hpp>

#include "lidar_camera_calib/visualization.h"
#include "lidar_camera_calib/loadBag.h"
#include "lidar_camera_calib/loadSettings.h"
#include "lidar_camera_calib/objectPose.h"
#include "lidar_camera_calib/omniModel.h"
#include "lidar_camera_calib/optimizer.h"

using namespace std;
using namespace cv;

vector<Mat> image_queue;
vector<unsigned long> stamp_set;
// boost::unordered_map<unsigned long, size_t> stamp_set;
OmniModel model;

Size patternsize(7, 10); // interior number of corners
float squareSize = 98.00; // unit: mm
Eigen::Matrix4d initial_transform;
unsigned long INIT_TIME = 1498121870886943937;

Mat init_rvec; // 4 by 1, quaternion
Mat init_tvec; // 3 by 1


size_t findIndex(unsigned long stamp){
	size_t i;
	for (size_t i=0; i<stamp_set.size(); i++){
		if (stamp_set[i] == stamp)
			return i;
	}

	if (i >= stamp_set.size())
		return -1;
}

Eigen::Quaternionf quatMult(Eigen::Quaternionf q1, Eigen::Quaternionf q2) {
    Eigen::Quaternionf resultQ;
    resultQ.setIdentity();

    resultQ.w() = q1.w() * q2.w() - q1.vec().dot(q2.vec());
    resultQ.vec() = q1.w() * q2.vec() + q2.w() * q1.vec() + q1.vec().cross(q2.vec());

    return resultQ;
}

void displayCallback(const sensor_msgs::Image::ConstPtr img)
{
 	// receive msg and retrieve msg from image_queue
 	unsigned long stamp = img->header.stamp.toNSec() - INIT_TIME;
	size_t index = findIndex(stamp);
	if ( index != -1){
		Mat gray = image_queue[index];
		// corners in world frame (origin is on the upper left chessboard)
		vector<Point3f> worldCorners;
		calcBoardCornerPositions(patternsize, squareSize, worldCorners,
		          Settings::CHESSBOARD);

		// corners in image frame
		vector<Point2f> imageCorners;
		bool find_corners = findBoardCorner(gray, patternsize, imageCorners, true);
		if (find_corners){
            Mat rvec; // Rotation in axis-angle form
            Mat tvec;
            Eigen::Matrix4d pose;
            model.estimateTransformation(imageCorners, worldCorners, pose);

            Eigen::Vector3f center_vec_t(squareSize*(patternsize.width-1) / 2.0, 
            							 squareSize*(patternsize.height-1) / 2.0,
            												0);
            Eigen::Translation<float,3> center_t(center_vec_t);

            Eigen::Quaternionf r_lidar_frame( init_rvec.at<double>(3,0),
		    								  init_rvec.at<double>(0,0),
		   									  init_rvec.at<double>(1,0),
		    								  init_rvec.at<double>(2,0));
		    Eigen::Vector3f trans_vec_E(init_tvec.at<double>(0,0),
		    							init_tvec.at<double>(1,0),
		    							init_tvec.at<double>(2,0));
		    Eigen::Translation<float,3> t_lidar_frame(trans_vec_E);
		    
		    Eigen::Matrix3d R = pose.block<3,3>(0,0);
		    Eigen::Quaternionf r_pose(R.cast<float>());
		    Eigen::Vector3f t_vec_pose(pose(0,3), pose(1,3), pose(2,3));
		    Eigen::Translation<float, 3> t_pose(t_vec_pose);

            Eigen::Transform<float,3, Eigen::Affine> combined = t_lidar_frame * r_lidar_frame * t_pose * r_pose * center_t;

            float a = combined.translation().x();
            Eigen::Quaternionf r_combined (combined.rotation());
            
            ///////////////////////////////////////////////////////////////////////////////////////////////
            // publish checkerboard pose
            ///////////////////////////////////////////////////////////////////////////////////////////////
            ros::Rate rate(10);
            ros::NodeHandle n_write;
            ros::Publisher marker_pub = n_write.advertise<visualization_msgs::Marker>("visualization_marker", 1);

            uint32_t shape = visualization_msgs::Marker::CUBE;

            while(ros::ok()){
	            visualization_msgs::Marker marker;
				// Set the frame ID and timestamp.  See the TF tutorials for information on these.
				marker.header.frame_id = "/lidar";
				marker.header.stamp = ros::Time::now();

				// Set the namespace and id for this marker.  This serves to create a unique ID
				// Any marker sent with the same namespace and id will overwrite the old one
				marker.ns = "basic_shapes";
				marker.id = 0;

				// Set the marker type. 
				marker.type = shape;

				// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
				marker.action = visualization_msgs::Marker::ADD;

				// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
				
				marker.pose.position.x = combined.translation().x()/1000.0;
				marker.pose.position.y = combined.translation().y()/1000.0;
				marker.pose.position.z = combined.translation().z()/1000.0;
				marker.pose.orientation.x = r_combined.x();	
				marker.pose.orientation.y = r_combined.y();
				marker.pose.orientation.z = r_combined.z();
				marker.pose.orientation.w = r_combined.w();
				// marker.pose.position.x = 0.03991;
				// marker.pose.position.y = 0.03538;
				// marker.pose.position.z = 0.10219;
				// marker.pose.orientation.x = -0.146437666759;	
				// marker.pose.orientation.y = 0.492800355296;
				// marker.pose.orientation.z = 0.0845458701372;
				// marker.pose.orientation.w = 0.853554811021;

				// Set the scale of the marker -- 1x1x1 here means 1m on a side
				marker.scale.x = squareSize*(patternsize.width-1) / 1000.0;
				marker.scale.y = squareSize*(patternsize.height-1) / 1000.0;
				marker.scale.z = 10.0 / 1000.0;

				// Set the color -- be sure to set alpha to something non-zero!
				marker.color.r = 0.0f;
				marker.color.g = 1.0f;
				marker.color.b = 0.0f;
				marker.color.a = 1.0;

				marker.lifetime = ros::Duration();

				// Publish the marker
				while (marker_pub.getNumSubscribers() < 1)
				{
					if (!ros::ok())
					{
						return;
					}
					ROS_WARN_ONCE("Please create a subscriber to the marker");
					sleep(1);
				}
				marker_pub.publish(marker);
				// rate.sleep();
				// ros::spinOnce();
				break;
				
			}
			////////////////////////////////////////////////////////////////////////////////////////////////////////////
        }

	}
}

/*
 *  publish the initial estimated pose of checkerboard
 */
int main(int argc, char** argv){
	/*
	 * ros::init
	 */
	ros::init(argc, argv, "calibration_result_visualization");

	/**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
	ros::NodeHandle n;

	/* 
     * Load image, convert from ROS image format to OpenCV Mat
     */
    string bag_file("/home/audren/lidar_camera_calib/data/cameraLidarData.bag");
    
    vector<vector<Point3f> > lidar_queue;
    loadBag(bag_file, image_queue, stamp_set, lidar_queue);
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
   
   	init_rvec = s.initialRotation; // 4 by 1, quaternion
    init_tvec = s.initialTranslation; // 3 by 1

    model.setParameter(camera_matrix, dist_coeffs, xi);
    ros::Subscriber sub = n.subscribe("/cam1/image_raw", 1000, displayCallback);
    ros::spin();
    return 0;
}