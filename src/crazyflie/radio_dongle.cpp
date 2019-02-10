#include "radio_dongle.h"
#include <chrono>
#include "math/types.h"
#include "crazyflie/protocol.h"
#include "text_logger.h"
#include "raw_packet.h"
#include <libusb-1.0/libusb.h>

RadioDongle::RadioDongle() :
    _context(nullptr),
    _devDevice(nullptr),
    _device(nullptr),
    _deviceVersion(0.0f),
    _radioIsConnected(false),
    _packetsToSend()
{
    // TODO SF
    //    int returnVal = libusb_init(&_context);
    // Do error checking here.
    libusb_init(&_context);
//
    _address[0] = 0xe7;
    _address[1] = 0xe7;
    _address[2] = 0xe7;
    _address[3] = 0xe7;
    _address[4] = 0xe7;
}
RadioDongle::~RadioDongle()
{
    CloseDevice();

    if(_context)
    {
        libusb_exit(_context);
    }
}

void RadioDongle::CloseDevice()
{
    if(_device)
    {
        libusb_close(_device);
        libusb_unref_device(_devDevice);

        _device = nullptr;
        _devDevice = nullptr;
    }
}

std::vector<libusb_device*> RadioDongle::ListDevices(int vendorID, int productID)
{
    std::vector<libusb_device*> devices;
    ssize_t count;
    libusb_device** pdevices;

    count = libusb_get_device_list(_context, &pdevices);
    for(ssize_t i = 0; i < count; i++)
    {
        libusb_device_descriptor ddDescriptor;

        libusb_get_device_descriptor(pdevices[i], &ddDescriptor);

        if(ddDescriptor.idVendor == vendorID && ddDescriptor.idProduct == productID)
        {
            libusb_ref_device(pdevices[i]);
            devices.emplace_back(pdevices[i]);
        }
    }

    if(count > 0)
    {
        libusb_free_device_list(pdevices, 1);
    }


    return devices;
}

bool RadioDongle::OpenUSBDongle()
{
    CloseDevice();
    auto devices = ListDevices(0x1915, 0x7777);

    if( !devices.empty())
    {
        // For now, just take the first device. Give it a second to
        // initialize the system permissions.
        sleep(1.0);

        auto* devFirst = devices.front();
        int error = libusb_open(devFirst, &_device);
        // If ok, remove the first element and save it
        if(error == 0)
        {
            // Opening device OK. Don't free the first device just yet.
            devices.erase(devices.begin());
            _devDevice = devFirst;
        }
        else
        {
             std::cout << "Error! No dongle found. ERR "<< error <<std::endl;
        }

        for(auto & device : devices)
        {
            libusb_unref_device(device);
        }

        return !error;
    }
    else
    {
        textLogger << "Error! No dongle found.\n";
    }

    return false;
}

void RadioDongle::StartRadio()
{
    _radioIsConnected = false;
    auto USBDongleIsOpen = OpenUSBDongle();
    if( USBDongleIsOpen )
    {
        // Read device version
        libusb_device_descriptor descriptor;
        libusb_get_device_descriptor(_devDevice, &descriptor);
        _deviceVersion = ConvertToDeviceVersion(descriptor.bcdDevice);
        textLogger << "Got device version " << _deviceVersion << "\n";
        if(_deviceVersion < 0.4)
        {
            std::cout << "Device version too low. Device is not supported.\n";
            return;
        }

        // Set active configuration to 1
        libusb_set_configuration(_device, 1);
        // Claim interface
        bool claimInterfaceOK = ClaimInterface(0);
        if(claimInterfaceOK)
        {
            WriteContCarrier(_contCarrier);
            WriteAddress(_address);
            WritePower(_power);
            WriteARC(_arc);
            WriteARDBytes(_ardBytes);
            WriteDataRate(_dataRate);
            WriteChannel(_channel);

            _radioIsConnected = true;
        }
    }
}


void RadioDongle::StopRadio()
{

}

bool RadioDongle::WriteData(uint8_t * data, int length)
{
    int actWritten;
    int retValue = libusb_bulk_transfer(_device, (0x01 | LIBUSB_ENDPOINT_OUT), data, length, &actWritten, 1000);

    if(retValue == 0 && actWritten == length)
    {
        return true;
    }
    switch(retValue)
    {
    case 0:
        textLogger << "Writing data failed partially\n";
        break;
    case LIBUSB_ERROR_TIMEOUT:
        textLogger << "USB timeout" << "\n";
        break;
    default:
        break;
    }
    return false;
}



bool RadioDongle::ReadData(uint8_t* data, int maxLength, int & actualLength)
{
    int actRead;
    int retValue = libusb_bulk_transfer(_device, (0x81 | LIBUSB_ENDPOINT_IN), data,  maxLength, &actRead, 50);

    switch(retValue)
    {
    case 0:
        actualLength = actRead;
        break;
    case LIBUSB_ERROR_TIMEOUT:
        actualLength = maxLength;
        break;
    default:
        actualLength = maxLength;
        break;
    }

    return (retValue == 0);
}

bool RadioDongle::WriteRadioControl(uint8_t* data, int length, DongleConfiguration  request, uint16_t value, uint16_t index)
{
    int timeout = 1000;

    /*int nReturn = */libusb_control_transfer(_device,
                                              LIBUSB_REQUEST_TYPE_VENDOR,
                                              static_cast<uint8_t>(request),
                                              value,
                                              index,
                                              data,
                                              length,
                                              timeout);

    // if(nReturn == 0) {
    //   return true;
    // }

    // Hack. TODO SF Fix
    return true;
}

void RadioDongle::WriteARC(int arc)
{
    WriteRadioControl(nullptr, 0, DongleConfiguration::SET_RADIO_ARC, arc, 0);
}

int RadioDongle::GetChannel() const
{
    return _channel;
}
void RadioDongle::WriteChannel(int channel)
{
    WriteRadioControl(nullptr, 0, DongleConfiguration::SET_RADIO_CHANNEL, channel, 0);
}

std::string const & RadioDongle::GetDataRate() const
{
    return _dataRate;
}
void RadioDongle::WriteDataRate(std::string dataRate)
{
    int dataRateCoded = -1;

    if(dataRate == "250K")
    {
        dataRateCoded = 0;
    }
    else if(dataRate == "1M")
    {
        dataRateCoded = 1;
    }
    else if(dataRate == "2M")
    {
        dataRateCoded = 2;
    }

    WriteRadioControl(nullptr, 0, DongleConfiguration::SET_DATA_RATE, dataRateCoded, 0);
}



void RadioDongle::WriteARDTime(int ARDTime)
{ // in uSec
    auto T = static_cast<int>((ARDTime / 250) - 1);
    if(T < 0)
    {
        T = 0;
    }
    else if(T > 0xf)
    {
        T = 0xf;
    }

    WriteRadioControl(nullptr, 0, DongleConfiguration::SET_RADIO_ARD, T, 0);
}

void RadioDongle::WriteARDBytes(int ARDBytes)
{
    WriteRadioControl(nullptr, 0, DongleConfiguration::SET_RADIO_ARD, 0x80 | ARDBytes, 0);
}

RadioDongle::PowerSettings RadioDongle::GetPower()
{
    return _power;
}

void RadioDongle::WritePower(PowerSettings power)
{
    WriteRadioControl(nullptr, 0, DongleConfiguration::SET_RADIO_POWER, static_cast<unsigned short>(power), 0);
}

void RadioDongle::WriteAddress(uint8_t*  address)
{
    WriteRadioControl(address, addrLength, DongleConfiguration::SET_RADIO_ADDRESS, 0, 0);
}

void RadioDongle::WriteContCarrier(bool contCarrier)
{
    WriteRadioControl(nullptr, 0, DongleConfiguration::SET_CONT_CARRIER, (contCarrier ? 1 : 0), 0);
}

bool RadioDongle::ClaimInterface(int interface)
{
    int errorCode = libusb_claim_interface(_device, interface);
    switch(errorCode)
    {
    case LIBUSB_SUCCESS:
            return true;
    case LIBUSB_ERROR_NOT_FOUND:
        textLogger<< "Failed to claim usb interface, device not found\n";
        break;
    case LIBUSB_ERROR_BUSY:
        textLogger<< "Failed to claim usb interface, device is busy\n";
        break;
    case LIBUSB_ERROR_NO_DEVICE:
        textLogger<< "Failed to claim usb interface, no device\n";
        break;
    case LIBUSB_ERROR_OTHER:
        textLogger<< "Failed to claim usb interface, undefined error\n";
        break;
    default:
        textLogger << "Failed to claim usb interface, code = " << errorCode << "\n";
        break;
    }

    return false;
}


void RadioDongle::IsUsbConnectionOk()
{
    libusb_device_descriptor descriptor;
    bool ok = (libusb_get_device_descriptor(_devDevice, &descriptor) == 0);
    emit USBOKSignal(ok);
}

bool RadioDongle::RadioIsConnected() const
{
    return _radioIsConnected;
}

float RadioDongle::ConvertToDeviceVersion(short number) const
{
    float version = static_cast<float>(number) / 100.0;
    return version;
}

void RadioDongle::SendPacketsNow()
{
    // Call function periodically
    // Sends one package every call.
    // If _packetsSending is empty, a ping packet is sent to keep the connection open
    if(_packetsToSend.empty())
    {
        CRTPPacket ping_packet{Console::id, Console::Print::id, {static_cast<uint8_t>(0xff)}};
        SendPacket(ping_packet);

    }
    else
    {
        CRTPPacket packet = _packetsToSend.front();
        _packetsToSend.pop();
        //        packet.Print();
        SendPacket(packet);
        //    textLogger << "Sending one packet, " << _packetsSending.size() << " left to send\n";
    }
}

bool RadioDongle::SendPacket(CRTPPacket packet)
{
    if(!_radioIsConnected)
        return false;

    IsUsbConnectionOk();
    return WriteData(packet.SendableData(), packet.GetSendableDataLength());
}

void RadioDongle::RegisterPacketToSend(RawPacket rawPacket)
{
    CRTPPacket packet(rawPacket);
    _packetsToSend.push(packet);
}

std::optional<RawPacket> RadioDongle::ReceivePacket() // executed every 1ms
{
    if(!_radioIsConnected)
        return {};

    int bufferSize = 64;
    uint8_t buffer[bufferSize];
    int bytesRead = 0;
    // Read the raw data from the dongle
    bool readDataOK = ReadData(buffer, bufferSize, bytesRead) ;
    // Check validity of packet
    if(!readDataOK)
    {
        return {};
    }
    if(bytesRead > 0)
    {
        // Convert the raw data to a packet
        RawPacket rawPacket(buffer, bytesRead);


        // Process the packe and distribute to ports + channels
        emit RawPacketReady(rawPacket);
        return rawPacket;
    }
    return {};
}
