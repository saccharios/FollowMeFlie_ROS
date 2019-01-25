#pragma  once
#include <QObject>
#include "crtp_packet.h"

class PacketHandler :  public QObject
{
    Q_OBJECT
public:


public slots:
    void ReceiveRawPacket(CRTPPacket packet);
    void RegisterPacketToSend(CRTPPacket packet);
signals:
    void RawPacketReadyToSend(CRTPPacket packet);
    void NewParameterPacket(CRTPPacket packet);
    void NewLoggerPacket(CRTPPacket packet);

private:
    void ProcessPacket(CRTPPacket packet);

};


