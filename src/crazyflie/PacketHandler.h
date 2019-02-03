#pragma  once
#include <QObject>
#include "crtp_packet.h"
#include "raw_packet.h"
class PacketHandler :  public QObject
{
    Q_OBJECT
public:
    bool AckReceived() const;
    bool IsUsbConnectionOk() const;

public slots:
    void ReceiveRawPacket(RawPacket rawPacket);
    void RegisterPacketToSend(CRTPPacket packet);
    void USBConnectionOK(bool ok);
signals:
    void RawPacketReadyToSend(CRTPPacket packet);
    void NewParameterPacket(CRTPPacket packet);

    void NewLoggerPacket(CRTPPacket packet);

private:
    void ProcessPacket(CRTPPacket packet);
    bool _ackReceived = false;
    bool _isUsbConnectionOk = false;
};


