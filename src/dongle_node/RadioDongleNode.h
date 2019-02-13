#pragma once

#include "crazyflie/radio_dongle.h"
#include "ros/ros.h"
#include "follow_me_flie_ros/StartRadio.h"
#include "follow_me_flie_ros/StopRadio.h"
#include "follow_me_flie_ros/Status.h"
#include <follow_me_flie_ros/RawPacket.h>

class RadioDongleNode {
public:
    RadioDongleNode();
    void Run();

private:
    RadioDongle _dongle;
    ros::NodeHandle _nh;

    ros::ServiceServer _serviceStartRadio;
    ros::ServiceServer _serviceStopRadio;
    ros::ServiceServer _serviceStatus;

    ros::Timer _timer;

    ros::Subscriber _subscriberRegisterPacketToSend;
    ros::Publisher _publisherRawPacketReady;
    ros::Publisher _publisherUSBConnectionIsOk;


    bool StartRadio(
            follow_me_flie_ros::StartRadio::Request  &req,
            follow_me_flie_ros::StartRadio::Response &res) ;
    bool StopRadio(
            follow_me_flie_ros::StopRadio::Request  &req,
            follow_me_flie_ros::StopRadio::Response &res) ;
    bool Status(
            follow_me_flie_ros::Status::Request  &req,
            follow_me_flie_ros::Status::Response &res) ;

    void RegisterPacktToSend(follow_me_flie_ros::RawPacketConstPtr const & packet);


    void RunCallBack(ros::TimerEvent const & event);
};

