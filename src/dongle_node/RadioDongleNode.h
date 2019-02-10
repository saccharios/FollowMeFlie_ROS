#pragma once
#include "crazyflie/radio_dongle.h"
#include "ros/ros.h"
#include "follow_me_flie_ros/StartRadio.h"

class RadioDongleNode {
public:
    RadioDongleNode();




private:
    RadioDongle _dongle;
    ros::NodeHandle _nh;


    bool StartRadio(
            follow_me_flie_ros::StartRadio::Request  &req,
            follow_me_flie_ros::StartRadio::Response &res) ;


};

