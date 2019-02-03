#pragma  once
#include <QObject>
#include "crtp_packet.h"
#include "raw_packet.h"
#include "ros/ros.h"
#include "follow_me_flie_ros/RawPacket.h"

class PacketHandler :  public QObject
{
    Q_OBJECT
public:
    PacketHandler() : _nh()
    {
        _pub = _nh.advertise<follow_me_flie_ros::RawPacket>("SendPackets", 1000);
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


    ros::NodeHandle _nh;
    ros::Publisher _pub;
};


