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

Mat optimized_rvec(4, 1, CV_64F);
Mat optimized_tvec(3, 1, CV_64F);

size_t last_index = 0;

size_t findIndex(unsigned long stamp){
	size_t i;
	size_t n = stamp_set.size();
	for (size_t i=0; i<n; i++){
		if (stamp_set[(last_index + i) % n] == stamp){
			last_index = (last_index + i) % n;
			return last_index;
		}
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
 	// ros::Time aa = ros::Time::now();
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

		// std::cout << "OK" << endl;
		ros::Time t0 = ros::Time::now();
		// ros::Duration cc = a - aa;
		// cout << "used time: " << cc.toSec() << endl;
		if (find_corners){
            Mat rvec; // Rotation in axis-angle form
            Mat tvec;
            Eigen::Matrix4d pose;
            model.estimateTransformation(imageCorners, worldCorners, pose);

            Eigen::Vector3f center_vec_t(squareSize*(patternsize.width-1) / 2.0, 
            							 squareSize*(patternsize.height-1) / 2.0,
            												0);
            Eigen::Translation<float,3> center_t(center_vec_t);

            // initial estimation
            Eigen::Quaternionf r_lidar_frame( init_rvec.at<double>(3,0),
		    								  init_rvec.at<double>(0,0),
		   									  init_rvec.at<double>(1,0),
		    								  init_rvec.at<double>(2,0));
		    Eigen::Vector3f trans_vec_E(init_tvec.at<double>(0,0),
		    							init_tvec.at<double>(1,0),
		    							init_tvec.at<double>(2,0));
		    Eigen::Translation<float,3> t_lidar_frame(trans_vec_E);

		    // optimized transform
		    Eigen::Quaternionf r_lidar_frame_optimized( optimized_rvec.at<double>(3,0),
		    								  optimized_rvec.at<double>(0,0),
		   									  optimized_rvec.at<double>(1,0),
		    								  optimized_rvec.at<double>(2,0));
		    Eigen::Vector3f trans_vec_E_optimized(optimized_tvec.at<double>(0,0),
		    							optimized_tvec.at<double>(1,0),
		    							optimized_tvec.at<double>(2,0));
		    Eigen::Translation<float,3> t_lidar_frame_optimized(trans_vec_E_optimized);
		    
		    // pose
		    Eigen::Matrix3d R = pose.block<3,3>(0,0);
		    Eigen::Quaternionf r_pose(R.cast<float>());
		    Eigen::Vector3f t_vec_pose(pose(0,3), pose(1,3), pose(2,3));
		    Eigen::Translation<float, 3> t_pose(t_vec_pose);

            Eigen::Transform<float,3, Eigen::Affine> combined = t_lidar_frame * r_lidar_frame * t_pose * r_pose * center_t;
            Eigen::Transform<float,3, Eigen::Affine> combined_optimized = t_lidar_frame_optimized * r_lidar_frame_optimized * t_pose * r_pose * center_t;

            Eigen::Quaternionf r_combined (combined.rotation());
            Eigen::Quaternionf r_combined_optimized (combined_optimized.rotation());
            
            ///////////////////////////////////////////////////////////////////////////////////////////////
            // publish checkerboard pose
            ///////////////////////////////////////////////////////////////////////////////////////////////
            ros::Rate rate(5);
            ros::NodeHandle n_write;
            ros::Publisher marker_pub = n_write.advertise<visualization_msgs::Marker>("visualization_marker", 0);

            uint32_t shape = visualization_msgs::Marker::CUBE;
            ros::Time t1 = ros::Time::now();
            ros::Duration diff = t1 - t0;
            cout << "Duration: " << diff.toSec() << endl;
            while(ros::ok()){
	            visualization_msgs::Marker marker;
	            visualization_msgs::Marker marker_optimized;
				// Set the frame ID and timestamp.  See the TF tutorials for information on these.
				marker.header.frame_id = "/lidar";
				marker.header.stamp = ros::Time::now();
				marker_optimized.header.frame_id = "/lidar";
				marker_optimized.header.stamp = ros::Time::now();

				// Set the namespace and id for this marker.  This serves to create a unique ID
				// Any marker sent with the same namespace and id will overwrite the old one
				marker.ns = "basic_shapes";
				marker.id = 0;
				marker_optimized.ns = "basic_shapes";
				marker_optimized.id = 1;

				// Set the marker type. 
				marker.type = shape;
				marker_optimized.type = shape;
				// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
				marker.action = visualization_msgs::Marker::ADD;
				marker_optimized.action = visualization_msgs::Marker::ADD;

				// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
				
				marker.pose.position.x = combined.translation().x()/1000.0;
				marker.pose.position.y = combined.translation().y()/1000.0;
				marker.pose.position.z = combined.translation().z()/1000.0;
				marker.pose.orientation.x = r_combined.x();	
				marker.pose.orientation.y = r_combined.y();
				marker.pose.orientation.z = r_combined.z();
				marker.pose.orientation.w = r_combined.w();

				marker_optimized.pose.position.x = combined_optimized.translation().x()/1000.0;
				marker_optimized.pose.position.y = combined_optimized.translation().y()/1000.0;
				marker_optimized.pose.position.z = combined_optimized.translation().z()/1000.0;
				marker_optimized.pose.orientation.x = r_combined_optimized.x();	
				marker_optimized.pose.orientation.y = r_combined_optimized.y();
				marker_optimized.pose.orientation.z = r_combined_optimized.z();
				marker_optimized.pose.orientation.w = r_combined_optimized.w();
				
				// Set the scale of the marker -- 1x1x1 here means 1m on a side
				marker.scale.x = squareSize*(patternsize.width-1) / 1000.0;
				marker.scale.y = squareSize*(patternsize.height-1) / 1000.0;
				marker.scale.z = 10.0 / 1000.0;

				marker_optimized.scale.x = squareSize*(patternsize.width-1) / 1000.0;
				marker_optimized.scale.y = squareSize*(patternsize.height-1) / 1000.0;
				marker_optimized.scale.z = 10.0 / 1000.0;

				// Set the color -- be sure to set alpha to something non-zero!
				marker.color.r = 0.0f;
				marker.color.g = 1.0f;
				marker.color.b = 0.0f;
				marker.color.a = 1.0;
				marker_optimized.color.r = 0.0f;
				marker_optimized.color.g = 0.0f;
				marker_optimized.color.b = 1.0f;
				marker_optimized.color.a = 1.0;

				marker.lifetime = ros::Duration();
				marker_optimized.lifetime = ros::Duration();
				
				// Publish the marker
				while (marker_pub.getNumSubscribers() < 1)
				{
					// if (!ros::ok())
					// {
					// 	return;
					// }
					// ROS_WARN_ONCE("Please create a subscriber to the marker");
					sleep(0.01);
				}	
				marker_pub.publish(marker);
				
				ros::Time t2 = ros::Time::now();
				diff = t2 - t1;
				cout << "used time: " << diff.toSec() << endl;
				marker_pub.publish(marker_optimized);	
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

    Eigen::Matrix3f m;
	m = Eigen::AngleAxisf(-0.295931, Eigen::Vector3f::UnitX())
  					* Eigen::AngleAxisf(1.06568,  Eigen::Vector3f::UnitY())
  					* Eigen::AngleAxisf(0.167964, Eigen::Vector3f::UnitZ());

    Eigen::Quaternionf r_pose(m);
  	optimized_tvec.at<double>(0,0) = 29.9334;
  	optimized_tvec.at<double>(1,0) = 34.2440;
  	optimized_tvec.at<double>(2,0) = 93.6712;
  	optimized_rvec.at<double>(0,0) = r_pose.x();
  	optimized_rvec.at<double>(1,0) = r_pose.y();
  	optimized_rvec.at<double>(2,0) = r_pose.z();
  	optimized_rvec.at<double>(3,0) = r_pose.w();
  	

    model.setParameter(camera_matrix, dist_coeffs, xi);
    ros::Subscriber sub = n.subscribe("/cam1/image_raw", 10, displayCallback);
    ros::spin();
    return 0;
}