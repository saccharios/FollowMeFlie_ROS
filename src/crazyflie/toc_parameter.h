#pragma once
#include "math/types.h"
#include "toc_shared.h"
#include <QObject>
#include "protocol.h"
#include <queue>

class TocParameter : public QObject
{
    Q_OBJECT

public:

    enum class ParameterElementType  : uint8_t
    {
            UINT8  = 0x08,
            UINT16  = 0x09,
            UINT32  = 0x0A,
            UINT64  = 0x0B,
            INT8  = 0x00,
            INT16  = 0x01,
            INT32  = 0x02,
            INT64  = 0x03,
            FP16   = 0x05,
            FLOAT = 0x06,
            DOUBLE= 0x07,
    };
// Parameter definitions
    enum class cpu : uint8_t
    {
        flash = 0,
        id0 = 1,
        id1 = 2,
        id2 = 3,
    };
    enum class system : uint8_t
    {
        selftestPassed = 4,
        taskDump = 86,
    };


    enum class commander : uint8_t
    {
        enHighLevel = 5,
    };

    enum class flightmode : uint8_t
    {
        althold = 6,
        poshold = 7,
        posSet = 8,
        yawMode = 9,
        yawRst = 10,
        stabModeRoll = 11,
        stabModePitch = 12,
        stabModeYaw = 13,
    };
    enum class cmdrCPPM : uint8_t
    {
        rateRoll = 14,
        ratePitch = 15,
        rateYaw = 16,
        angRoll = 17,
        angPitch = 18,
    };
    enum class locSrv : uint8_t
    {
        enRangeStreamFP32 = 19,
    };
    enum class pid_attitude : uint8_t
    {
        roll_kp = 20,
        roll_ki = 21,
        roll_kd = 22,
        pitch_kp = 23,
        pitch_ki = 24,
        pitch_kd = 25,
        yaw_kp = 26,
        yaw_ki = 27,
        yaw_kd = 28,
    };
    enum class pid_rate : uint8_t
    {
        roll_kp = 29,
        roll_ki = 30,
        roll_kd = 31,
        pitch_kp = 32,
        pitch_ki = 33,
        pitch_kd = 34,
        yaw_kp = 35,
        yaw_ki = 36,
        yaw_kd = 37,
    };
    enum class sensorfusion6 : uint8_t
    {
        kp = 38,
        ki = 39,
        baseZacc = 40,
    };

    enum class health : uint8_t
    {
        startPropTest = 41,
    };

    enum class stabilizer : uint8_t
    {
        estimator = 42,
        controller = 43,
    };


    enum class posEstAlt : uint8_t
    {
        estAlphaAsl = 44,
        estAlphaZr = 45,
        velFactor = 46,
        velZAlpha = 47,
        vAccDeadband= 48,
    };
    enum class posCtlPid : uint8_t
    {
        xKp = 49,
        xKi = 50,
        xKd = 51,
        yKp = 52,
        yKi = 53,
        yKd = 54,
        zKp = 55,
        zKi = 56,
        zKd = 57,
        thrustBase = 58,
        thrustMin = 59,
        rpLimit = 60,
        xyVelMax = 61,
        zVelMax = 62,
    };
    enum class velCtlPid : uint8_t
    {
        vxKp = 63,
        vxKi = 64,
        vxKd = 65,
        vyKp = 66,
        vyKi = 67,
        vyKd = 68,
        vzKp = 69,
        vzKi = 70,
        vzKd = 71,
    };
    enum class controller : uint8_t
    {
        tiltComp = 72
    };

    enum class ctrlMel : uint8_t
    {
        kp_xy = 73,
        kd_xy = 74,
        ki_xy = 75,
        i_range_xy = 76,
        kp_z = 77,
        kd_z = 78,
        ki_z = 79,
        i_range_z = 80,
        mass = 81,
        massThrust = 82,
        kR_xy = 83,
        kR_z = 84,
        kw_xy = 85,
        kw_z = 86,
        ki_m_xy = 87,
        ki_m_z = 88,
        kd_omega_rp = 89,
        i_range_m_xy = 90,
        i_range_m_z = 91,
    };

    enum class motorPowerSet : uint8_t
    {
        enable = 92,
        m1 = 93,
        m2 = 94,
        m3 = 95,
        m4 = 96,
    };
    enum class firmware : uint8_t
    {
        revision0 = 97,
        revision1 = 98,
        modified = 99,
    };
    enum class imu_sensors : uint8_t
    {
        HMC5883L = 100,
        MS5611 = 101,
    };
    enum class imu_tests : uint8_t
    {
        MPU6500 = 102,
        HMC5883L = 103,
        MS5611 = 104,
    };
    enum class sound : uint8_t
    {
        effect = 105,
        neffect = 106,
        freq = 107,
        ratio = 108,
    };
    enum class kalman : uint8_t
    {
        resetEstimation = 110,
        quadIsFlying = 111,
        pNacc_xy = 112,
        pNacc_z = 113,
        pNVel = 114,
        pNPos = 115,
        pNatt = 116,
        pNSkew = 117,
        mNBaro = 118,
        mNGyro_rollpitch = 119,
        mNGyro_yaq = 120,
        initialX = 121,
        initialY = 122,
        initialZ = 123,
    };
    enum class ring : uint8_t
    {
        effect = 126,
        neffect = 127,
        solidRed = 128,
        solidGreen = 129,
        solidBlue = 130,
        headlightEnable = 131,
        glowstep = 132,
        emptyCharge = 133,
        fullCharge = 134,
        fadeColor = 135,
        fadetime = 136,
    };
    enum class deck : uint8_t
    {
        bcBuzzer = 137,
        bcGTGPS = 138,
        bcCPPM = 139,
        bcUSD = 140,
        bcZRanger = 141,
        bcZRagner2 = 142,
        bcDWM1000 = 143,
        bcFlow = 147,
        bcFlow2 = 148,
        bcOA = 150,
        bcMultiranger = 151,
    };
    enum class loco : uint8_t
    {
        mode = 144,
    };
    enum class tdoa3 : uint8_t
    {
        logld = 145,
        logOthrld = 146,
    };
    enum class motion : uint8_t
    {
        disable = 149,
    };


    TocParameter(PacketHandler & packetHandler) :
      _itemCount(0),
      _elements(),
       _shared_impl(_itemCount, _elements, packetHandler ),
       _lastReadParameter(-1)
    {}


    bool Setup() {return _shared_impl.Setup();}
    bool ReadAll();

    void WriteParameterFast(uint8_t id, float value);

    std::vector<TOCElement> const & GetElements() const {return _elements; }

    TOCElement GetElement(uint8_t idx );
    void LogAll() {_shared_impl.LogAll();}
    void Reset();
signals:
    void ParameterRead(uint8_t const &);
    void ParameterWriteFailed (TOCElement const &);
    void SendPacket(CRTPPacket packet);

public slots:
    void WriteParameter(uint8_t, float);
    void ReceivePacket(CRTPPacket packet);
    void WriteParametersPeriodically();

private:
    unsigned int _itemCount;
    std::vector<TOCElement> _elements;
    TOCShared<Parameter::id, Parameter::Access> _shared_impl;
    int8_t _lastReadParameter;

    struct ParameterSend
    {
        uint8_t id;
        float value;
        int cntr;
    };

    std::queue<ParameterSend> _requestWritingParameter;

    void ProcessReadData(Data const & data);
    void ProcessWriteData(Data const & data);
    void ProcessMiscData(Data const & data);
    void ReadData(Data const & data, uint8_t parameterIdPosition, uint8_t valuePosition);
    void ReadElement(uint8_t elementId);
    void AddToRequestWritingParamteter(uint8_t id, float value);
    bool WriteValue(TOCElement & element, float value);

};
