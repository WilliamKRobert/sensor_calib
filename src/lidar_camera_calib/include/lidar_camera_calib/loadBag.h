#include <iostream>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include "lidar_camera_calib/hash.h"

void convertImage(const sensor_msgs::Image::ConstPtr &img, cv::Mat &cv_img);
void loadBag(const std::string &filename, 
			std::vector<cv::Mat>& image_queue, 
			HashMap& time_stamp, 
			std::vector<std::vector<cv::Point3f> >& lidar_queue);

void loadBag(const std::string &filename, 
            std::vector<cv::Mat>& image_queue, 
            std::vector<unsigned long>& time_stamp, 
            std::vector<std::vector<cv::Point3f> >& lidar_queue);

class cameraData
{
public:
    sensor_msgs::Image::ConstPtr image;
    sensor_msgs::CameraInfo::ConstPtr cam_info;

    cameraData(const sensor_msgs::Image::ConstPtr &img,
               const sensor_msgs::CameraInfo::ConstPtr &info):
        image(img),
        cam_info(info)
    {}
};




