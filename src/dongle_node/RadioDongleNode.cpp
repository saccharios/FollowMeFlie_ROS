#include "RadioDongleNode.h"
#include "ros/ros.h"
#include "ros_service_definitions.h"
#include "follow_me_flie_ros/StartRadio.h"
#include "follow_me_flie_ros/StopRadio.h"
#include "follow_me_flie_ros/Status.h"
#include "time_levels.h"
RadioDongleNode::RadioDongleNode() :
    _nh(),
    _dongle(),
    _loop_rate(1.0f/sendReceiveSamplingTime)
{
    _serviceStartRadio = _nh.advertiseService(RosService::StartRadio, &RadioDongleNode::StartRadio, this);
    _serviceStopRadio = _nh.advertiseService(RosService::StopRadio, &RadioDongleNode::StopRadio, this);
    _serviceStatus = _nh.advertiseService(RosService::Status, &RadioDongleNode::Status, this);

    _timer = _nh.createTimer(ros::Duration(sendReceiveSamplingTime_seconds), &RadioDongleNode::RunCallBack, this);



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

void RadioDongleNode::RunCallBack(ros::TimerEvent const & event)
{
    _dongle.SendPacketsNow();
    _dongle.ReceivePacket();
}

