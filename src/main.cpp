#include "gui/main_window.h"
#include <QApplication>
#include <ros/ros.h>
#include "thread"

void run_ros()
{
    ros::spin();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "follow_me_flie_main");
    ros::NodeHandle nh;

    textLogger.Init();
    QApplication app(argc, argv);
    MainWindow w;
    w.show();

    std::thread t(run_ros);

    return app.exec();
}


