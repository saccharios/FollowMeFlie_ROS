#pragma  once
#include <QObject>
#include "crtp_packet.h"

class PacketHandler :  public QObject
{
    Q_OBJECT
public:

    void RegisterPacketToSend(CRTPPacket packet);
    void ProcessPacket(CRTPPacket packet);


public slots:
    void ReceiveRawPacket(CRTPPacket packet);
signals:
    void RawPacketReadyToSend(CRTPPacket packet);
    void NewParameterPacket(CRTPPacket packet);
    void NewLoggerPacket(CRTPPacket packet);

private:

};


