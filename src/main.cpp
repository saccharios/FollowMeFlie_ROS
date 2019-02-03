#include "gui/main_window.h"
#include <QApplication>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "follow_me_flie_main");
    ros::NodeHandle nh;

    textLogger.Init();
    QApplication app(argc, argv);
    MainWindow w;
    w.show();
    return app.exec();
}


