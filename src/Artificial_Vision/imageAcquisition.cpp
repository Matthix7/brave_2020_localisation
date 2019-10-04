#include <ros/ros.h>

// OpenCV headers.
#include "opencv2/highgui.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imageAcquisition");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");
}
