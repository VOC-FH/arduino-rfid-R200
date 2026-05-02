#ifndef R200_h
#define R200_h

// Generate additional debug information to the serial connection when defined
// #define DEBUG

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>

#ifndef R200_RX_BUFFER_LENGTH
#define R200_RX_BUFFER_LENGTH 128
#endif

#ifndef R200_MAX_PAYLOAD_LENGTH
#define R200_MAX_PAYLOAD_LENGTH 96
#endif

#define RX_BUFFER_LENGTH R200_RX_BUFFER_LENGTH

class R200 {
  public:
    static constexpr uint8_t MaxEpcLength = 62;
    static constexpr uint8_t MaxDataLength = R200_MAX_PAYLOAD_LENGTH;

    struct TagReport {
      uint8_t rssi = 0;
      uint16_t pc = 0;
      uint8_t epc[MaxEpcLength] = {0};
      uint8_t epcLength = 0;
      uint16_t crc = 0;
    };

    struct Frame {
      uint8_t type = 0;
      uint8_t command = 0;
      uint16_t length = 0;
      uint8_t payload[MaxDataLength] = {0};
    };

    enum ModuleInfoType : uint8_t {
      ModuleInfo_HardwareVersion = 0x00,
      ModuleInfo_SoftwareVersion = 0x01,
      ModuleInfo_Manufacturer = 0x02,
    };

    enum MemoryBank : uint8_t {
      MemoryBank_Reserved = 0x00,
      MemoryBank_EPC = 0x01,
      MemoryBank_TID = 0x02,
      MemoryBank_User = 0x03,
    };

    enum Status : uint8_t {
      Status_Ok = 0x00,
      Status_CommandError = 0x17,
      Status_FHSSFail = 0x20,
      Status_InventoryFail = 0x15,
      Status_AccessFail = 0x16,
      Status_ReadFail = 0x09,
      Status_WriteFail = 0x10,
      Status_LockFail = 0x13,
      Status_KillFail = 0x12,
      Status_Unknown = 0xFE,
      Status_Timeout = 0xFD,
      Status_BadFrame = 0xFC,
      Status_BufferTooSmall = 0xFB,
    };

    enum R200_FrameStructure : uint8_t {
      R200_HeaderPos = 0x00,
      R200_TypePos = 0x01,
      R200_CommandPos = 0x02,
      R200_ParamLengthMSBPos = 0x03,
      R200_ParamLengthLSBPos = 0x04,
      R200_ParamPos = 0x05,
    };

    enum R200_FrameControl : uint8_t {
      R200_FrameHeader = 0xAA,
      R200_FrameEnd = 0xDD,
    };

    enum R200_FrameType : uint8_t {
      FrameType_Command = 0x00,
      FrameType_Response = 0x01,
      FrameType_Notification = 0x02,
    };

    enum R200_Command : uint8_t {
      CMD_GetModuleInfo = 0x03,
      CMD_EnterIdleMode = 0x04,
      CMD_SetWorkArea = 0x07,
      CMD_GetSelectParameter = 0x0B,
      CMD_SetSelectParameter = 0x0C,
      CMD_GetQueryParameters = 0x0D,
      CMD_SetQueryParameters = 0x0E,
      CMD_SetSendSelectInstruction = 0x12,
      CMD_ModuleSleep = 0x17,
      CMD_ControlIOPort = 0x1A,
      CMD_SetModuleIdleSleepTime = 0x1D,
      CMD_SinglePollInstruction = 0x22,
      CMD_MultiplePollInstruction = 0x27,
      CMD_StopMultiplePoll = 0x28,
      CMD_ReadLabel = 0x39,
      CMD_WriteLabel = 0x49,
      CMD_KillTag = 0x65,
      CMD_LockLabel = 0x82,
      CMD_GetWorkingChannel = 0xAA,
      CMD_SetWorkingChannel = 0xAB,
      CMD_SetAutoFrequencyHopping = 0xAD,
      CMD_SetTransmitContinuousCarrier = 0xB0,
      CMD_SetTransmitPower = 0xB6,
      CMD_AcquireTransmitPower = 0xB7,
      CMD_BlockPermalock = 0xD3,
      CMD_BlockPermalock2 = 0xD4,
      CMD_NXPChangeConfig = 0xE0,
      CMD_NXPReadProtect = 0xE1,
      CMD_NXPChangeEAS = 0xE3,
      CMD_NXPEASAlarm = 0xE4,
      CMD_ImpinjMonzaQTReadWrite = 0xE5,
      CMD_ImpinjMonzaQTControl = 0xE6,
      CMD_SetReceiverDemodulatorParameters = 0xF0,
      CMD_GetReceiverDemodulatorParameters = 0xF1,
      CMD_TestRFInputBlockingSignal = 0xF2,
      CMD_TestChannelRSSI = 0xF3,
      CMD_ExecutionFailure = 0xFF,
    };

    enum R200_ErrorCode : uint8_t {
      ERR_CommandError = 0x17,
      ERR_FHSSFail = 0x20,
      ERR_InventoryFail = 0x15,
      ERR_AccessFail = 0x16,
      ERR_ReadFail = 0x09,
      ERR_WriteFail = 0x10,
      ERR_LockFail = 0x13,
      ERR_KillFail = 0x12,
    };

    R200();

    uint8_t uid[12] = {0};
    TagReport lastTag;

    bool begin(HardwareSerial *serial = &Serial2, int baud = 115200, uint8_t RxPin = 16, uint8_t TxPin = 17);
    void loop();
    bool dataAvailable();

    // Low-level protocol access. Useful for commands not yet wrapped or vendor-specific variants.
    bool sendCommand(uint8_t command, const uint8_t *payload = nullptr, uint16_t payloadLength = 0);
    Status command(uint8_t command, const uint8_t *payload, uint16_t payloadLength, Frame *response = nullptr, unsigned long timeout = 500);
    Status readFrame(Frame &frame, unsigned long timeout = 500);
    uint8_t lastError() const;

    // Inventory
    void poll();
    bool singlePoll(TagReport *tag = nullptr, unsigned long timeout = 500);
    void setMultiplePollingMode(bool enable = true);
    bool startMultiplePolling(uint16_t count = 0xFFFF);
    bool stopMultiplePolling();
    bool readTagNotification(TagReport &tag, unsigned long timeout = 50);

    // Module information
    void dumpModuleInfo();
    bool getModuleInfo(ModuleInfoType type, char *buffer, size_t bufferLength, unsigned long timeout = 500);

    // Select/query/session parameters. Raw wrappers map directly to the protocol PDF payloads.
    bool setSelectParameter(const uint8_t *payload, uint16_t payloadLength);
    bool getSelectParameter(uint8_t *payload, uint16_t &payloadLength, uint16_t maxPayloadLength, unsigned long timeout = 500);
    bool sendSelectInstruction();
    bool getQueryParameters(uint8_t *payload, uint16_t &payloadLength, uint16_t maxPayloadLength, unsigned long timeout = 500);
    bool setQueryParameters(const uint8_t *payload, uint16_t payloadLength);

    // Tag memory operations. Addresses and lengths are in words (16 bit), as in EPC Gen2.
    bool readLabel(MemoryBank bank, uint16_t wordAddress, uint8_t wordCount, uint32_t accessPassword,
                   uint8_t *data, uint16_t &dataLength, uint16_t maxDataLength, unsigned long timeout = 1000);
    bool writeLabel(MemoryBank bank, uint16_t wordAddress, const uint8_t *data, uint8_t wordCount,
                    uint32_t accessPassword, unsigned long timeout = 1000);
    bool lockLabel(uint32_t accessPassword, uint32_t mask, uint32_t action, unsigned long timeout = 1000);
    bool killTag(uint32_t killPassword, unsigned long timeout = 1000);

    // RF configuration
    bool setWorkArea(uint8_t regionCode);
    bool setWorkingChannel(uint8_t channelIndex);
    bool getWorkingChannel(uint8_t &channelIndex, unsigned long timeout = 500);
    bool setAutoFrequencyHopping(const uint8_t *channels, uint8_t channelCount);
    bool clearAutoFrequencyHopping();
    bool getTransmitPower(uint16_t &centiDbm, unsigned long timeout = 500);
    bool setTransmitPower(uint16_t centiDbm);
    bool setTransmitPowerDbm(float dbm);
    bool setTransmitContinuousCarrier(bool enable);
    bool getReceiverDemodulatorParameters(uint8_t *payload, uint16_t &payloadLength, uint16_t maxPayloadLength, unsigned long timeout = 500);
    bool setReceiverDemodulatorParameters(const uint8_t *payload, uint16_t payloadLength);
    bool testRFInputBlockingSignal(uint8_t *payload, uint16_t &payloadLength, uint16_t maxPayloadLength, unsigned long timeout = 1000);
    bool testChannelRSSI(uint8_t channelIndex, uint8_t &rssi, unsigned long timeout = 1000);

    // IO and power management
    bool controlIOPort(uint8_t port, bool high);
    bool moduleSleep();
    bool setModuleIdleSleepTime(uint8_t minutes);
    bool setIdleMode(bool enable, uint8_t idleTime = 1, uint8_t reserved = 1);

    // NXP / Impinj / extended operations. These expose documented command frames without hard-coding tag-model policy.
    bool nxpChangeConfig(uint32_t accessPassword, uint16_t configWord, uint8_t *payload, uint16_t &payloadLength, uint16_t maxPayloadLength, unsigned long timeout = 1000);
    bool nxpReadProtect(uint32_t accessPassword, bool reset, unsigned long timeout = 1000);
    bool nxpChangeEAS(uint32_t accessPassword, bool enable, unsigned long timeout = 1000);
    bool nxpEASAlarm(uint8_t *payload, uint16_t &payloadLength, uint16_t maxPayloadLength, unsigned long timeout = 1000);
    bool impinjMonzaQT(uint8_t command, const uint8_t *payload, uint16_t payloadLength, Frame *response = nullptr, unsigned long timeout = 1000);
    bool blockPermalock(uint8_t command, const uint8_t *payload, uint16_t payloadLength, Frame *response = nullptr, unsigned long timeout = 1000);

    void dumpUIDToSerial();

  private:
    HardwareSerial *_serial = nullptr;
    uint8_t _buffer[R200_RX_BUFFER_LENGTH] = {0};
    uint16_t _bufferLength = 0;
    uint8_t _lastError = Status_Ok;
    const uint8_t blankUid[12] = {0};

    uint8_t calculateCheckSum(const uint8_t *buffer) const;
    uint8_t calculateCheckSum(uint8_t type, uint8_t command, const uint8_t *payload, uint16_t payloadLength) const;
    uint16_t arrayToUint16(const uint8_t *array) const;
    bool parseReceivedData();
    bool dataIsValid() const;
    bool receiveData(unsigned long timeOut = 500);
    void dumpReceiveBufferToSerial();
    uint8_t flush();
    uint16_t framePayloadLength() const;
    bool decodeFrame(Frame &frame) const;
    bool parseTagReport(const Frame &frame, TagReport &tag);
    Status expectAck(uint8_t command, unsigned long timeout = 500);
    static void appendU16(uint8_t *payload, uint16_t &pos, uint16_t value);
    static void appendU32(uint8_t *payload, uint16_t &pos, uint32_t value);
    static bool copyPayload(const Frame &frame, uint8_t *payload, uint16_t &payloadLength, uint16_t maxPayloadLength);
};

#endif
