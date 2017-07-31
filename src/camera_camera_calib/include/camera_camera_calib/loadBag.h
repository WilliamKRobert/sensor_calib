#include <iostream>
#include <boost/unordered_map.hpp>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

void convertImage(const sensor_msgs::Image::ConstPtr &img, cv::Mat &cv_img);

void loadBag(const std::string &filename,
            const std::string topic0,
            const std::string toppic1, 
            std::vector<cv::Mat>& im0, 
            std::vector<cv::Mat>& im1,
            size_t sample_num,
            size_t max_im_num
            );