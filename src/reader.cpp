#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;


void display(Mat &im, Mat &bbox)
{
  int n = bbox.rows;
  for(int i = 0 ; i < n ; i++)
  {
    line(im, Point2i(bbox.at<float>(i,0),bbox.at<float>(i,1)), Point2i(bbox.at<float>((i+1) % n,0), bbox.at<float>((i+1) % n,1)), Scalar(255,0,0), 3);
  }
  imshow("display", im);
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::Mat img_mat = cv_bridge::toCvShare(msg, "bgr8")->image;


    cv::QRCodeDetector::QRCodeDetector qrDecoder = cv::QRCodeDetector::QRCodeDetector();

    Mat bbox, rectifiedImage;

    std::string data = qrDecoder.detectAndDecode(img_mat, bbox, rectifiedImage);
    if(data.length()>0)
    {
      cout << "Decoded Data : " << data << endl;

      display(img_mat, bbox);
      rectifiedImage.convertTo(rectifiedImage, CV_8UC3);
      imshow("view", rectifiedImage);

      waitKey(0);
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::namedWindow("display");

  image_transport::ImageTransport it(nh);
  // Change topic depending on your camera
  image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("display");
}
