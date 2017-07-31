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
#include <ros/exceptions.h> 

#include <camera_camera_calib/loadBag.h>

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


void loadBag(const string &filename, const string topic0, const string topic1, vector<Mat>& im0, vector<Mat>& im1, size_t sample_num, size_t max_im_num)
{
    rosbag::Bag bag;
    try{
        bag.open(filename, rosbag::bagmode::Read);
    }
    catch(rosbag::BagException& e){
        ROS_ERROR("Cannot open bag files: %s!", e.what());
        return;
    }    

    vector<string> topics;
    topics.push_back(topic0);
    topics.push_back(topic1);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    size_t index0 = 0, index1 = 0;
    bool camFlag = true;
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
            

            if (m.getTopic() == topic0) {
                if (index0 % sample_num == 0 && im0.size() < max_im_num)
                    im0.push_back(cv_img);
                index0++;
            }
            else if (m.getTopic() == topic1 && im1.size() < max_im_num) {
                if (index1 % sample_num == 0)
                    im1.push_back(cv_img);  
                index1++;
            }
            else{

            }
        }
    }

}


