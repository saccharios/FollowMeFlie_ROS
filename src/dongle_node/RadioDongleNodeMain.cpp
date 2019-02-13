#include "ros/ros.h"
#include "RadioDongleNode.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "radio_dongle_node");

    RadioDongleNode radioDongleNode;
    std::cout << "hello world\n";

    radioDongleNode.Run();
    return 0;
}
