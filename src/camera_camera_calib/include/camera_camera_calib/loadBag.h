/**
 * Camera-Camera calibration.
 *
 * Muyuan Lin, 2017
 */

#ifndef LOADBAG_H
#define LOADBAG_H

#include <iostream>
#include <boost/unordered_map.hpp>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/TransformStamped.h>

void convertImage(const sensor_msgs::Image::ConstPtr &img, cv::Mat &cv_img);

void loadBag(const std::string &filename,
            const std::string topic0,
            const std::string toppic1, 
            std::vector<cv::Mat>& im0, 
            std::vector<cv::Mat>& im1,
            size_t sample_num,
            size_t max_im_num
            );

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, 
                                                sensor_msgs::Image> NoCloudSyncPolicy;
typedef message_filters::Subscriber<sensor_msgs::Image> image_sub_type;                                                

class MyClass {
public:
	MyClass(){}
	~MyClass(){_bag.close();}

    message_filters::Subscriber<sensor_msgs::Image> *topic0_sub_;      
    message_filters::Subscriber<sensor_msgs::Image> *topic1_sub_;        
    message_filters::Synchronizer<NoCloudSyncPolicy>* no_cloud_sync_;

    rosbag::Bag _bag;
    std::string _topic0, _topic1;

	// method definitions needed here
    void init(const std::string topic0, const std::string topic1);
    void callbackMethod (const sensor_msgs::ImageConstPtr& cam0_img_msg,
                              const sensor_msgs::ImageConstPtr& cam1_img_msg);
};

#endif