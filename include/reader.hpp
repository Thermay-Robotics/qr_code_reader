#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>

#include <iostream>
#include <algorithm>
#include <vector>
#include <zbar.h>
#include <cmath>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Include message
#include "qr_code_reader/qr_msg.h"

// TODO add distance to msg

class Reader
{
private:
    ros::NodeHandle nh;
    ros::Publisher qr_pub;
    image_transport::Subscriber camera_sub;
    image_transport::ImageTransport it;

    std::string camera_topic;
    bool viewer;

    void imageCallback(const sensor_msgs::ImageConstPtr &msg);

public:
    Reader(/* args */);
    ~Reader() { cv::destroyAllWindows(); };

    void decode(cv::Mat &im);
};
