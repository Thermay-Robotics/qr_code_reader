#include "reader.hpp"

Reader::Reader() : it(nh)
{
  cv::namedWindow("view");
  nh.getParam("/qr_reader/camera_topic", camera_topic);

  qr_pub = nh.advertise<qr_code_reader::qr_msg>("/QrCodeMsg", 1000);
  camera_sub = it.subscribe(camera_topic, 1, &Reader::imageCallback, this);
}

void Reader::decode(cv::Mat &im)
{

  // Create zbar scanner
  zbar::ImageScanner scanner;

  // Configure scanner
  scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

  // Convert image to grayscale
  cv::Mat imGray;
  cv::cvtColor(im, imGray, CV_BGR2GRAY);

  // Wrap image data in a zbar image
  zbar::Image image(im.cols, im.rows, "Y800", (uchar *)imGray.data, im.cols * im.rows);

  // Scan the image for barcodes and QRCodes
  int n = scanner.scan(image);

  // Print results
  for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
  {
    qr_code_reader::qr_msg msg;

    msg.type = symbol->get_type_name();
    msg.data = symbol->get_data();

    for (int i = 0; i < symbol->get_location_size(); i++)
    {
      geometry_msgs::Point p;

      p.x = symbol->get_location_x(i);
      p.y = symbol->get_location_y(i);

      msg.locations.push_back(p);

      cv::Scalar black(0, 0, 0);
      cv::Point center((int)p.x, (int)p.y);
      cv::circle(im, center, 10, black);
    }

    qr_pub.publish(msg);
  }
}

void Reader::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    cv::Mat img_mat = cv_bridge::toCvShare(msg, "bgr8")->image;

    decode(img_mat);

    cv::imshow("view", img_mat);

    cv::waitKey(30);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}