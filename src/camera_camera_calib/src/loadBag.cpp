
#include <cv_bridge/cv_bridge.h>
#include <boost/foreach.hpp>

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


void loadBag(const string &filename, const string topic0, 
            const string topic1, vector<Mat>& im0, 
            vector<Mat>& im1, size_t sample_num, size_t max_im_num)
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

void MyClass::init(const std::string topic0, const std::string topic1)
{
    _bag.open("test.bag", rosbag::bagmode::Write);
    
    _topic0 = topic0;
    _topic1 = topic1;

    int q = 10; //queue size
    ros::NodeHandle nh;

    topic0_sub_ = new image_sub_type(nh, _topic0, q);
    topic1_sub_ = new image_sub_type(nh, _topic1, q);

    no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(
        NoCloudSyncPolicy(q),  *topic0_sub_, *topic1_sub_);

    no_cloud_sync_->registerCallback(boost::bind(&MyClass::callbackMethod, this, _1, _2));
}

//The callback method
void MyClass::callbackMethod (const sensor_msgs::ImageConstPtr& cam0_img_msg,
                              const sensor_msgs::ImageConstPtr& cam1_img_msg) 
{
    std_msgs::Header h = cam0_img_msg->header;

    std_msgs::Header h1 = cam1_img_msg->header;

    _bag.write(_topic0, h.stamp, cam0_img_msg);
    _bag.write(_topic1, h1.stamp, cam1_img_msg);
}



