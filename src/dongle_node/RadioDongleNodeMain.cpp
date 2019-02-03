#include "ros/ros.h"
#include "RadioDongleNode.h"
#include "follow_me_flie_ros/RawPacket.h"

void callback(follow_me_flie_ros::RawPacketConstPtr const & rawPacket)
{
    ROS_INFO_STREAM( "Length of received data is : " << rawPacket->length);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "radio_dongle_node");


    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("SendPackets", 1000, &callback);


    ros::spin();
    return 0;
}
