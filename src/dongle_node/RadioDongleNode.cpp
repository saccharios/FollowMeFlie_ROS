#include "RadioDongleNode.h"
#include "ros/ros.h"
#include "ros_service_definitions.h"
#include "follow_me_flie_ros/StartRadio.h"
#include "follow_me_flie_ros/StopRadio.h"
#include "follow_me_flie_ros/Status.h"
#include "time_levels.h"
#include <src/ros_topic_definitions.h>
#include "follow_me_flie_ros/RawPacket.h"

RadioDongleNode::RadioDongleNode() :
    _nh(),
    _dongle(),
    _loop_rate(1.0f/sendReceiveSamplingTime)
{
    _serviceStartRadio = _nh.advertiseService(RosService::StartRadio, &RadioDongleNode::StartRadio, this);
    _serviceStopRadio = _nh.advertiseService(RosService::StopRadio, &RadioDongleNode::StopRadio, this);
    _serviceStatus = _nh.advertiseService(RosService::Status, &RadioDongleNode::Status, this);

    _timer = _nh.createTimer(ros::Duration(sendReceiveSamplingTime_seconds), &RadioDongleNode::RunCallBack, this);

    _subscriberRegisterPacketToSend = _nh.subscribe(RosTopics::SendPackets, 1000, &RadioDongleNode::RegisterPacktToSend, this);

    _publisherRawPacketReady = _nh.advertise<follow_me_flie_ros::RawPacket>(RosTopics::RawPacketReceived, 1000);

}

bool RadioDongleNode::StartRadio(
        follow_me_flie_ros::StartRadio::Request  &,
        follow_me_flie_ros::StartRadio::Response &)
{
    _dongle.StartRadio();
    return true;
}

bool RadioDongleNode::StopRadio(
        follow_me_flie_ros::StopRadio::Request  &,
        follow_me_flie_ros::StopRadio::Response &)
{
    _dongle.StopRadio();
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
    // Run at rate of SendPacketsNow and ReceivePacket (every 1ms)
    while(ros::ok())
    {
        // do
        ros::spinOnce();
        _loop_rate.sleep();
    }


}

void RadioDongleNode::RegisterPacktToSend(follow_me_flie_ros::RawPacketConstPtr const & packet)
{
    // Convert
    RawPacket rawPacket;
    for(int i = 0; i < RawPacket::maxBufferLength; ++i)
    {
        rawPacket._data.at(i) = packet->data.at(i);
    }
    rawPacket._length = packet->length;

    // Register
    _dongle.RegisterPacketToSend(rawPacket);
}

void RadioDongleNode::RunCallBack(ros::TimerEvent const & event)
{
    _dongle.SendPacketsNow();


    auto response = _dongle.ReceivePacket();
    if(response.has_value())
    {
        follow_me_flie_ros::RawPacket packet;
        packet.length = response.value()._length;
        for(int i = 0; i < RawPacket::maxBufferLength; ++i)
        {
            packet.data.at(i) = response.value()._data.at(i);
        }

        _publisherRawPacketReady.publish(packet);
    }


}



