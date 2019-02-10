#include "RadioDongleNode.h"
#include "ros/ros.h"
#include "ros_service_definitions.h"
#include "follow_me_flie_ros/StartRadio.h"

RadioDongleNode::RadioDongleNode() :
    _nh(),
    _dongle()
{


    ros::ServiceServer service = _nh.advertiseService(RosService::StartRadio, &RadioDongleNode::StartRadio, this);


}

bool RadioDongleNode::StartRadio(
        follow_me_flie_ros::StartRadio::Request  &req,
        follow_me_flie_ros::StartRadio::Response &res)
{
    return true;
}
