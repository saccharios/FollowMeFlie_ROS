#include <ros/ros.h>
#include "mainwindow.h"
#include <QApplication>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "main_window_test");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");


  QApplication a(argc, argv);
  MainWindow w;
  w.show();

  return a.exec();
}
