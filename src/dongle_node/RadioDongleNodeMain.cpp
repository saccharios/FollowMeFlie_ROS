#include <src/crazyflie/crtp_packet.h>
#include <src/crazyflie/raw_packet.h>
#include "ros/ros.h"
#include "RadioDongleNode.h"
#include "follow_me_flie_ros/RawPacket.h"
#include "crazyflie/crtp_packet.h"
#include "crazyflie/raw_packet.h"
#include "ros_topic_definitions.h"

void callback(follow_me_flie_ros::RawPacketConstPtr const & rawPacket)
{
    std::array<uint8_t, 64> buffer;
    for (int i = 0; i < rawPacket->length; ++i)
    {
        buffer.at(i) = rawPacket->data[i];
    }
    CRTPPacket packet(buffer, rawPacket->length);
//    ROS_INFO_STREAM( "Length of received data is : " << static_cast<int>(rawPacket->length) << " " <<  static_cast<int>(rawPacket->data[0]));
    ROS_INFO_STREAM("" << packet);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "radio_dongle_node");


    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe(RosTopics::SendPackets, 1000, &callback);
    ROS_INFO_STREAM("Radio Donge up and running.");

    ros::spin();
    return 0;
}
