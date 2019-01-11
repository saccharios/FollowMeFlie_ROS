#include "gui/main_window.h"
#include <QApplication>

#include <ros/ros.h>
#include <QtMultimedia>
#include <QtMultimediaWidgets>
#include <QDebug>
#include <memory>
#include "crazyflie/crazy_flie.h"

#include "text_logger.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "follow_me_flie_main");
    ros::NodeHandle nh;
    ROS_INFO("Hello world!");

    textLogger.Init();
    QApplication app(argc, argv);
    MainWindow w;
    w.show();
    return app.exec();
}


