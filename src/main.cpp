#include "reader.hpp"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");

  Reader reader;

  ros::spin();

  return 0;
}
