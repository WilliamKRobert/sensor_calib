#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>

#include "lidar_camera_calib/hash.h"

class Visualizer
{
public:
	// width, height, depth: dimension of checkerboard
	//   unit: meter
	Visualizer(double width, double height, double depth, HashMap stamp_set)
		:_rate(1),
		_width_checkerboard(width), 
		_height_checkerboard(height), 
		_depth_checkerboard(depth),
		_stamp_set(stamp_set)
		{
  		// _rate = 1;
  		_marker_pub = _node.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  		// Set shape type to be a cube
  		_shape = visualization_msgs::Marker::CUBE;
	}

	Visualizer():_rate(1)
	{
  		_marker_pub = _node.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  		_shape = visualization_msgs::Marker::CUBE;
	}

	void setParameter(double width, double height, double depth){
		_width_checkerboard = width;
		_height_checkerboard = height; 
		_depth_checkerboard = depth;
	}

	void update_checkerboard_pose(Eigen::Vector3d translation, Eigen::Vector4d rotation){
		visualization_msgs::Marker marker;
		// Set the frame ID and timestamp.  See the TF tutorials for information on these.
		marker.header.frame_id = "/base";
		marker.header.stamp = ros::Time::now();

		// Set the namespace and id for this marker.  This serves to create a unique ID
		// Any marker sent with the same namespace and id will overwrite the old one
		marker.ns = "basic_shapes";
		marker.id = 0;

		// Set the marker type. 
		marker.type = _shape;

		// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
		marker.action = visualization_msgs::Marker::ADD;

		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		marker.pose.position.x = translation(0);
		marker.pose.position.y = translation(1);
		marker.pose.position.z = translation(2);
		marker.pose.orientation.x = rotation(0);
		marker.pose.orientation.y = rotation(1);
		marker.pose.orientation.z = rotation(2);
		marker.pose.orientation.w = rotation(3);

		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		marker.scale.x = _width_checkerboard;
		marker.scale.y = _height_checkerboard;
		marker.scale.z = _depth_checkerboard;

		// Set the color -- be sure to set alpha to something non-zero!
		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;

		marker.lifetime = ros::Duration();

		// Publish the marker
		while (_marker_pub.getNumSubscribers() < 1)
		{
			if (!ros::ok())
			{
				return;
			}
			ROS_WARN_ONCE("Please create a subscriber to the marker");
			sleep(1);
		}
		_marker_pub.publish(marker);

		_rate.sleep();
	}

	


private:
	double _width_checkerboard;
	double _height_checkerboard;
	double _depth_checkerboard;

	ros::Publisher _marker_pub;
	ros::NodeHandle _node;
	uint32_t _shape;
	ros::Rate _rate;

	HashMap _stamp_set;



};

#endif