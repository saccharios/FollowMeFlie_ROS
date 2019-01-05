#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "main_window_test");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");
}
