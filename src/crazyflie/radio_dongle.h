#pragma once
#include <libusb-1.0/libusb.h>
#include <unistd.h>
#include "crtp_packet.h"
#include <QObject>
#include "math/double_buffer.h"
#include <queue>

class RadioDongle :  public QObject
{
    Q_OBJECT

public:
    enum class PowerSettings
    {
        // Power at -18dbm
        P_M18DBM = 0,
        // Power at -12dbm
        P_M12DBM = 1,
        // Power at -6dbm
        P_M6DBM = 2,
        // Power at 0dbm
        P_0DBM = 3
    };

    enum class DongleConfiguration
    {
        SET_RADIO_CHANNEL = 0x01,
        SET_RADIO_ADDRESS = 0x02,
        SET_DATA_RATE = 0x03,
        SET_RADIO_POWER = 0x04,
        SET_RADIO_ARD = 0x05,
        SET_RADIO_ARC = 0x06,
        ACK_ENABLE = 0x10,
        SET_CONT_CARRIER = 0x20,
        SCANN_CHANNELS = 0x21,
        LAUNCH_BOOTLOADER = 0xFF
    };

    RadioDongle();
    ~RadioDongle();

    void StartRadio();
    void StopRadio();

    RadioDongle::PowerSettings Power();
    void SetPower(PowerSettings power);


    bool AckReceived();
    bool IsUsbConnectionOk();

    bool RadioIsConnected() const;

    bool SendPacket(CRTPPacket && packet); // Call when a packet is requested to send
    void RegisterPacketToSend(CRTPPacket &&  packet);
    void ProcessPacket(CRTPPacket && packet);

public slots:
    void SendPacketsNow();
    void ReceivePacket();
signals:
    void NewParameterPacket(CRTPPacket packet);
    void NewLoggerPacket(CRTPPacket packet);

private:
    libusb_context* _context;
    libusb_device* _devDevice;
    libusb_device_handle* _device;
    const int _arc = 10;
    const int _channel = 80;
    const std::string _dataRate = "2M";
    const int _ardTime = 0; // not used
    const int _ardBytes = 32;
    const PowerSettings _power = PowerSettings::P_0DBM;
    static constexpr int addrLength = 5;
    uint8_t _address[addrLength];
    int _contCarrier;
    float _deviceVersion;
    bool _ackReceived;
    bool _radioIsConnected;


    std::queue<CRTPPacket> _packetsToSend;

    void ReadRadioSettings();
    std::vector<libusb_device*> ListDevices(int vendorID, int productID);
    float ConvertToDeviceVersion(short number) const;
    bool OpenUSBDongle();
    bool ClaimInterface(int nInterface);
    void CloseDevice();

    CRTPPacket CreatePacketFromData( uint8_t* buffer, int totalLength);

    bool WriteData(uint8_t * data, int length);
    bool ReadData(uint8_t* data, int maxLength, int & actualLength);

    void SetARC(int ARC);
    void WriteChannel(int channel);
    int GetChannel() const;
    std::string const & GetDataRate() const;
    void WriteDataRate(std::string dataRate);
    void SetARDBytes(int ARDBytes);
    void SetARDTime(int ARDTime);
    void SetAddress(uint8_t* address);
    void SetContCarrier(bool contCarrier);
    bool WriteRadioControl(uint8_t* data, int length, DongleConfiguration request, uint16_t value, uint16_t index);




};

