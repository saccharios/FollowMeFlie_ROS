#include "RadioDongleNode.h"
#include "ros/ros.h"
#include "ros_service_definitions.h"
#include "follow_me_flie_ros/StartRadio.h"
#include "follow_me_flie_ros/StopRadio.h"
#include "follow_me_flie_ros/Status.h"
#include "time_levels.h"
#include <src/ros_topic_definitions.h>
#include "follow_me_flie_ros/RawPacket.h"
#include "follow_me_flie_ros/USBConnStatus.h"

RadioDongleNode::RadioDongleNode() :
    _nh(),
    _dongle()
{
    _serviceStartRadio = _nh.advertiseService(RosService::StartRadio, &RadioDongleNode::StartRadio, this);
    _serviceStopRadio = _nh.advertiseService(RosService::StopRadio, &RadioDongleNode::StopRadio, this);
    _serviceStatus = _nh.advertiseService(RosService::Status, &RadioDongleNode::Status, this);

    _timer = _nh.createTimer(ros::Duration(sendReceiveSamplingTime_seconds), &RadioDongleNode::RunCallBack, this);

    _subscriberRegisterPacketToSend = _nh.subscribe(RosTopics::SendPackets, 1000, &RadioDongleNode::RegisterPacktToSend, this);

    _publisherRawPacketReady = _nh.advertise<follow_me_flie_ros::RawPacket>(RosTopics::RawPacketReceived, 1000);
    _publisherUSBConnectionIsOk = _nh.advertise<follow_me_flie_ros::USBConnStatus>(RosTopics::USBConnStatus, 1000);

}

bool RadioDongleNode::StartRadio(
        follow_me_flie_ros::StartRadio::Request  &,
        follow_me_flie_ros::StartRadio::Response &)
{
    _dongle.ReleaseRadio(true);
    return true;
}

bool RadioDongleNode::StopRadio(
        follow_me_flie_ros::StopRadio::Request  &,
        follow_me_flie_ros::StopRadio::Response &)
{
    _dongle.ReleaseRadio(false);
    return true;
}

bool RadioDongleNode::Status(
        follow_me_flie_ros::Status::Request  & req,
        follow_me_flie_ros::Status::Response & res)
{
    res.response = _dongle.RadioIsConnected();
    return true;
}

void RadioDongleNode::Run()
{
    ros::spin();
}

void RadioDongleNode::RegisterPacktToSend(follow_me_flie_ros::RawPacketConstPtr const & packet)
{
    RawPacket rawPacket = ConvertMsgPacketToRawPacket(packet);

    // Register
    _dongle.RegisterPacketToSend(rawPacket);
}

void RadioDongleNode::RunCallBack(ros::TimerEvent const & event)
{
    follow_me_flie_ros::USBConnStatus USBConnStatusMsg;
    USBConnStatusMsg.USBConnStatus = _dongle.IsUsbConnectionOk();

    _publisherUSBConnectionIsOk.publish(USBConnStatusMsg);

    // Send Packets
    _dongle.SendPacketsNow();

    // Receive packets, needed to be converted
    auto response = _dongle.ReceivePacket();
    if(response.has_value())
    {
        RawPacket rawPacket = response.value();

        follow_me_flie_ros::RawPacket packet = ConvertRawPacketToMsgPacket(rawPacket);

        _publisherRawPacketReady.publish(packet);
    }


}



