#pragma  once
#include <QObject>
#include "crtp_packet.h"
#include "raw_packet.h"
#include "ros/ros.h"
#include "follow_me_flie_ros/RawPacket.h"
#include "follow_me_flie_ros/USBConnStatus.h"
#include "ros_topic_definitions.h"
class PacketHandler :  public QObject
{
    Q_OBJECT
public:
    PacketHandler() : _nh()
    {
        _publisherSendPackets = _nh.advertise<follow_me_flie_ros::RawPacket>(RosTopics::SendPackets, 1000);
        _subscriberRawPacketReady = _nh.subscribe(RosTopics::RawPacketReceived, 1000, &PacketHandler::ReceiveRawPacketMsg, this);
        _subscriberUSBConnectionIsOk = _nh.subscribe(RosTopics::USBConnStatus, 1000, &PacketHandler::USBConnStatus, this);
    }
    bool AckReceived() const;
    bool IsUsbConnectionOk() const;

public slots:
    void ReceiveRawPacket(RawPacket rawPacket);
    void RegisterPacketToSend(CRTPPacket packet);
    void USBConnectionOK(bool ok);
signals:
    void RawPacketReadyToSend(RawPacket rawPacket);
    void NewParameterPacket(CRTPPacket packet);

    void NewLoggerPacket(CRTPPacket packet);

private:
    void ProcessPacket(CRTPPacket packet);
    bool _ackReceived = false;
    bool _isUsbConnectionOk = false;
    void ReceiveRawPacketMsg(follow_me_flie_ros::RawPacketConstPtr const & packet);
    void USBConnStatus(follow_me_flie_ros::USBConnStatusConstPtr const & packet);


    ros::NodeHandle _nh;
    ros::Publisher _publisherSendPackets;
    ros::Subscriber _subscriberRawPacketReady;
    ros::Subscriber _subscriberUSBConnectionIsOk;
};


