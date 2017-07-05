#include <iostream>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include "std_msgs/Float32.h"
#include "sensor_msgs/PointCloud.h"
#include "laser_geometry/laser_geometry.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include <ros/exceptions.h> 

#include <find_camera_pose/loadBagFile.h>

using namespace std;
using namespace cv;


void convertImage(const sensor_msgs::Image::ConstPtr &img,
                   Mat& cv_img)
{
    cv_bridge::CvImagePtr cv_ptr;

    try{
        if (img == NULL) {
            cerr << "From convert_image.cpp (line 33): ROS image is empty!" << endl;
            return;
        }
        else{
            cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
            cv_img = cv_ptr->image;
        }
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

}

void convertLaser (const sensor_msgs::LaserScan::ConstPtr& scan_in, vector<Point3f> &pt_lidar)
{
    // laser_geometry::LaserProjection projector;
    // tf::TransformListener listener;

    // sensor_msgs::PointCloud cloud;

    // try{
    //     listener.waitForTransform(scan_in->header.frame_id, 
    //                                 "/base_link", 
    //                                 scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment), 
    //                                 ros::Duration(1.0));
    //     projector.transformLaserScanToPointCloud("base_link",*scan_in, cloud,listener);
    // }
    // catch(tf::TransformException& e){
    //     // cerr << e.what() <<endl;
    //     return;
    // }

    for (unsigned int i = 0; i < scan_in->ranges.size(); ++i)
    {
        Point3f p;
        float range = scan_in->ranges[i];
        if (range > scan_in->range_min && range < scan_in->range_max)
        {
            float angle = scan_in->angle_min + i*scan_in->angle_increment;

            p.x = range * cos(angle);
            p.y = range * sin(angle);
            p.z = 0.0;
            pt_lidar.push_back(p);
        }
        else
            continue;
  }
}


void lidarCameraAssociation(const vector<Mat> &image_queue_raw, const vector<ros::Time> &camera_time_stamp, 
                            const vector<vector<Point3f> > &lidar_queue_raw, const vector<ros::Time> &lidar_time_stamp, 
                            vector<Mat> &image_queue, vector<vector<Point3f> > &lidar_queue){
    size_t n = image_queue_raw.size();
    for (size_t i=0; i<n; i++){
        ros::Time image_t = camera_time_stamp[i];

        size_t j = i-1;
        double d;
        do{
            ros::Duration diff = lidar_time_stamp[++j] - image_t;
            d = diff.toSec();
        }while(d < 0 && j < lidar_time_stamp.size());

        size_t k = j;
        do{
            ros::Duration diff = lidar_time_stamp[--k] - image_t;
            d = diff.toSec();
        }while(d > 0 && k >= 0);
        
        if (j < lidar_time_stamp.size() && k >= 0){
            image_queue.push_back(image_queue_raw[i]);
            
            vector<Point3f> interpolation_points;
            for (size_t ii=0; ii<lidar_queue_raw[j].size(); ii++){
                Point3f pt1 = lidar_queue_raw[j][ii], pt2 = lidar_queue_raw[k][ii];
                Point3f pt((pt1.x+pt2.x)/2.0, (pt1.y+pt2.y)/2.0, (pt1.z+pt2.z)/2.0);
                interpolation_points.push_back(pt);
            }
            lidar_queue.push_back(interpolation_points);
        }
    }

}

void loadBag(const string &filename, vector<Mat>& image_queue, vector<vector<Point3f> >& lidar_queue)
{
    rosbag::Bag bag;
    try{
        bag.open(filename, rosbag::bagmode::Read);
    }
    catch(rosbag::BagException& e){
        ROS_ERROR("Cannot open bag files: %s!", e.what());
        return;
    }    

    string cam_image = string("/cam1") + "/image_raw";
    string cam_info = string("/cam1") + "/camera_info";
    string laser_scan = string("/UST_20LX/scan");

    vector<string> topics;
    topics.push_back(cam_image);
    topics.push_back(cam_info);
    topics.push_back(laser_scan);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    vector<ros::Time> camera_time_stamp, lidar_time_stamp;
    vector<Mat> image_queue_raw;
    vector<vector<Point3f> > lidar_queue_raw;
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::Image::ConstPtr img = m.instantiate<sensor_msgs::Image>();
        if (img){
            Mat cv_img;
            convertImage(img, cv_img);
            if (cv_img.empty()){
               cout << "Could not open or find the image" << endl;
               return;
            }
            image_queue_raw.push_back(cv_img);
            camera_time_stamp.push_back(img->header.stamp);
            continue;
        }

        sensor_msgs::LaserScan::ConstPtr lidar = m.instantiate<sensor_msgs::LaserScan>();
        if (lidar){
            vector<Point3f> pt_lidar;
            convertLaser(lidar, pt_lidar);
            lidar_queue_raw.push_back(pt_lidar);
            lidar_time_stamp.push_back(lidar->header.stamp);
            continue;
        }

    }

    lidarCameraAssociation(image_queue_raw, camera_time_stamp, lidar_queue_raw, lidar_time_stamp, 
        image_queue, lidar_queue);

}


