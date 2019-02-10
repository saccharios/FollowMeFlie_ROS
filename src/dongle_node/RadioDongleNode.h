#pragma once
#include "crazyflie/radio_dongle.h"
#include "ros/ros.h"
#include "follow_me_flie_ros/StartRadio.h"
#include "follow_me_flie_ros/StopRadio.h"
#include "follow_me_flie_ros/Status.h"

class RadioDongleNode {
public:
    RadioDongleNode();




private:
    RadioDongle _dongle;
    ros::NodeHandle _nh;
    ros::ServiceServer _serviceStartRadio;
    ros::ServiceServer _serviceStopRadio;
    ros::ServiceServer _serviceStatus;

    bool StartRadio(
            follow_me_flie_ros::StartRadio::Request  &req,
            follow_me_flie_ros::StartRadio::Response &res) ;
    bool StopRadio(
            follow_me_flie_ros::StopRadio::Request  &req,
            follow_me_flie_ros::StopRadio::Response &res) ;
    bool Status(
            follow_me_flie_ros::Status::Request  &req,
            follow_me_flie_ros::Status::Response &res) ;


};

