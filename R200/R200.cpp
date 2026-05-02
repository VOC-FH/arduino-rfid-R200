#include <Arduino.h>
#include "R200.h"

R200::R200() {}

bool R200::begin(HardwareSerial *serial, int baud, uint8_t RxPin, uint8_t TxPin) {
  _serial = serial;
  _serial->begin(baud, SERIAL_8N1, RxPin, TxPin);
  return true;
}

static void printHexByte(const char *name, uint8_t value) {
  Serial.print(name);
  Serial.print(":");
  Serial.print(value < 0x10 ? "0x0" : "0x");
  Serial.println(value, HEX);
}

static void printHexBytes(const char *name, const uint8_t *value, uint8_t len) {
  Serial.print(name);
  Serial.print(":0x");
  for (uint8_t i = 0; i < len; i++) {
    Serial.print(value[i] < 0x10 ? "0" : "");
    Serial.print(value[i], HEX);
  }
  Serial.println();
}

static void printHexWord(const char *name, uint16_t value) {
  Serial.print(name);
  Serial.print(":0x");
  if (value < 0x1000) Serial.print("0");
  if (value < 0x0100) Serial.print("0");
  if (value < 0x0010) Serial.print("0");
  Serial.println(value, HEX);
}

void R200::loop() {
  if (!dataAvailable()) {
    return;
  }

  Frame frame;
  Status status = readFrame(frame);
  if (status != Status_Ok) {
    return;
  }

  if (frame.command == CMD_SinglePollInstruction || frame.command == CMD_MultiplePollInstruction) {
    TagReport tag;
    if (parseTagReport(frame, tag)) {
      lastTag = tag;
      if (tag.epcLength >= sizeof(uid)) {
        memcpy(uid, tag.epc, sizeof(uid));
      }
      #ifdef DEBUG
        printHexByte("RSSI", tag.rssi);
        printHexWord("PC", tag.pc);
        printHexBytes("EPC", tag.epc, tag.epcLength);
        printHexWord("CRC", tag.crc);
      #endif
    }
  } else if (frame.command == CMD_ExecutionFailure && frame.length > 0) {
    _lastError = frame.payload[0];
    if (_lastError == ERR_InventoryFail && memcmp(uid, blankUid, sizeof(uid)) != 0) {
      memset(uid, 0, sizeof(uid));
      memset(&lastTag, 0, sizeof(lastTag));
    }
  }
}

bool R200::sendCommand(uint8_t command, const uint8_t *payload, uint16_t payloadLength) {
  if (_serial == nullptr || payloadLength > R200_MAX_PAYLOAD_LENGTH) {
    _lastError = Status_BufferTooSmall;
    return false;
  }

  uint8_t frame[5 + R200_MAX_PAYLOAD_LENGTH + 2] = {0};
  uint16_t pos = 0;
  frame[pos++] = R200_FrameHeader;
  frame[pos++] = FrameType_Command;
  frame[pos++] = command;
  frame[pos++] = static_cast<uint8_t>(payloadLength >> 8);
  frame[pos++] = static_cast<uint8_t>(payloadLength & 0xFF);

  for (uint16_t i = 0; i < payloadLength; i++) {
    frame[pos++] = payload[i];
  }

  frame[pos++] = calculateCheckSum(FrameType_Command, command, payload, payloadLength);
  frame[pos++] = R200_FrameEnd;

  _serial->write(frame, pos);
  return true;
}

R200::Status R200::command(uint8_t commandCode, const uint8_t *payload, uint16_t payloadLength, Frame *response, unsigned long timeout) {
  flush();
  if (!sendCommand(commandCode, payload, payloadLength)) {
    return static_cast<Status>(_lastError);
  }

  Frame frame;
  Status status = readFrame(frame, timeout);
  if (status != Status_Ok) {
    return status;
  }

  if (frame.command == CMD_ExecutionFailure) {
    _lastError = frame.length > 0 ? frame.payload[0] : Status_Unknown;
    return static_cast<Status>(_lastError);
  }

  if (frame.command != commandCode) {
    _lastError = Status_BadFrame;
    return Status_BadFrame;
  }

  if (response != nullptr) {
    *response = frame;
  }
  _lastError = Status_Ok;
  return Status_Ok;
}

R200::Status R200::readFrame(Frame &frame, unsigned long timeout) {
  if (!receiveData(timeout)) {
    _lastError = Status_Timeout;
    return Status_Timeout;
  }
  if (!dataIsValid() || !decodeFrame(frame)) {
    _lastError = Status_BadFrame;
    return Status_BadFrame;
  }
  _lastError = Status_Ok;
  return Status_Ok;
}

uint8_t R200::lastError() const {
  return _lastError;
}

bool R200::dataAvailable() {
  return _serial != nullptr && _serial->available() > 0;
}

void R200::dumpUIDToSerial() {
  Serial.print("0x");
  for (uint8_t i = 0; i < sizeof(uid); i++) {
    Serial.print(uid[i] < 0x10 ? "0" : "");
    Serial.print(uid[i], HEX);
  }
}

void R200::dumpReceiveBufferToSerial() {
  Serial.print("0x");
  for (uint16_t i = 0; i < _bufferLength; i++) {
    Serial.print(_buffer[i] < 0x10 ? "0" : "");
    Serial.print(_buffer[i], HEX);
  }
  Serial.println(". Done.");
}

bool R200::parseReceivedData() {
  Frame frame;
  if (!decodeFrame(frame)) {
    return false;
  }
  if (frame.command == CMD_SinglePollInstruction || frame.command == CMD_MultiplePollInstruction) {
    return parseTagReport(frame, lastTag);
  }
  return true;
}

uint8_t R200::flush() {
  uint8_t bytesDiscarded = 0;
  if (_serial == nullptr) {
    return 0;
  }
  while (_serial->available()) {
    _serial->read();
    bytesDiscarded++;
  }
  return bytesDiscarded;
}

bool R200::receiveData(unsigned long timeOut) {
  if (_serial == nullptr) {
    return false;
  }

  const unsigned long startTime = millis();
  _bufferLength = 0;
  memset(_buffer, 0, sizeof(_buffer));

  bool inFrame = false;
  while ((millis() - startTime) < timeOut) {
    while (_serial->available()) {
      uint8_t b = static_cast<uint8_t>(_serial->read());

      if (!inFrame) {
        if (b != R200_FrameHeader) {
          continue;
        }
        inFrame = true;
        _bufferLength = 0;
      }

      if (_bufferLength >= sizeof(_buffer)) {
        flush();
        _lastError = Status_BufferTooSmall;
        return false;
      }

      _buffer[_bufferLength++] = b;

      if (b == R200_FrameEnd && _bufferLength >= 7) {
        return true;
      }
    }
  }

  return false;
}

bool R200::dataIsValid() const {
  if (_bufferLength < 7 || _buffer[0] != R200_FrameHeader || _buffer[_bufferLength - 1] != R200_FrameEnd) {
    return false;
  }

  uint16_t paramLength = framePayloadLength();
  uint16_t expectedLength = 7 + paramLength;
  if (_bufferLength != expectedLength) {
    return false;
  }

  const uint16_t checksumPos = 5 + paramLength;
  return calculateCheckSum(_buffer) == _buffer[checksumPos];
}

uint16_t R200::framePayloadLength() const {
  uint16_t paramLength = _buffer[R200_ParamLengthMSBPos];
  paramLength <<= 8;
  paramLength += _buffer[R200_ParamLengthLSBPos];
  return paramLength;
}

bool R200::decodeFrame(Frame &frame) const {
  if (!dataIsValid()) {
    return false;
  }

  const uint16_t payloadLength = framePayloadLength();
  if (payloadLength > sizeof(frame.payload)) {
    return false;
  }

  frame.type = _buffer[R200_TypePos];
  frame.command = _buffer[R200_CommandPos];
  frame.length = payloadLength;
  if (payloadLength > 0) {
    memcpy(frame.payload, &_buffer[R200_ParamPos], payloadLength);
  }
  return true;
}

bool R200::parseTagReport(const Frame &frame, TagReport &tag) {
  if (frame.length < 5) {
    return false;
  }

  tag.rssi = frame.payload[0];
  tag.pc = (static_cast<uint16_t>(frame.payload[1]) << 8) | frame.payload[2];
  tag.epcLength = frame.length - 5; // RSSI + PC(2) + EPC + CRC(2)
  if (tag.epcLength > MaxEpcLength) {
    tag.epcLength = MaxEpcLength;
  }
  memcpy(tag.epc, &frame.payload[3], tag.epcLength);
  tag.crc = (static_cast<uint16_t>(frame.payload[3 + tag.epcLength]) << 8) | frame.payload[4 + tag.epcLength];
  return true;
}

R200::Status R200::expectAck(uint8_t commandCode, unsigned long timeout) {
  Frame response;
  Status status = command(commandCode, nullptr, 0, &response, timeout);
  if (status != Status_Ok) {
    return status;
  }
  if (response.length > 0 && response.payload[0] != Status_Ok) {
    _lastError = response.payload[0];
    return static_cast<Status>(_lastError);
  }
  return Status_Ok;
}

void R200::poll() {
  sendCommand(CMD_SinglePollInstruction);
}

bool R200::singlePoll(TagReport *tag, unsigned long timeout) {
  Frame response;
  Status status = command(CMD_SinglePollInstruction, nullptr, 0, &response, timeout);
  if (status != Status_Ok) {
    return false;
  }

  TagReport parsed;
  if (!parseTagReport(response, parsed)) {
    _lastError = Status_BadFrame;
    return false;
  }

  lastTag = parsed;
  if (parsed.epcLength >= sizeof(uid)) {
    memcpy(uid, parsed.epc, sizeof(uid));
  }
  if (tag != nullptr) {
    *tag = parsed;
  }
  return true;
}

void R200::setMultiplePollingMode(bool enable) {
  if (enable) {
    startMultiplePolling(0xFFFF);
  } else {
    stopMultiplePolling();
  }
}

bool R200::startMultiplePolling(uint16_t count) {
  uint8_t payload[3] = {0x22, static_cast<uint8_t>(count >> 8), static_cast<uint8_t>(count & 0xFF)};
  return command(CMD_MultiplePollInstruction, payload, sizeof(payload), nullptr) == Status_Ok;
}

bool R200::stopMultiplePolling() {
  return command(CMD_StopMultiplePoll, nullptr, 0, nullptr) == Status_Ok;
}

bool R200::readTagNotification(TagReport &tag, unsigned long timeout) {
  Frame frame;
  Status status = readFrame(frame, timeout);
  if (status != Status_Ok) {
    return false;
  }
  if (frame.command != CMD_SinglePollInstruction && frame.command != CMD_MultiplePollInstruction) {
    _lastError = Status_BadFrame;
    return false;
  }
  return parseTagReport(frame, tag);
}

void R200::dumpModuleInfo() {
  uint8_t param = ModuleInfo_HardwareVersion;
  sendCommand(CMD_GetModuleInfo, &param, 1);
}

bool R200::getModuleInfo(ModuleInfoType type, char *buffer, size_t bufferLength, unsigned long timeout) {
  if (buffer == nullptr || bufferLength == 0) {
    _lastError = Status_BufferTooSmall;
    return false;
  }

  uint8_t payload = static_cast<uint8_t>(type);
  Frame response;
  Status status = command(CMD_GetModuleInfo, &payload, 1, &response, timeout);
  if (status != Status_Ok) {
    return false;
  }

  size_t n = response.length;
  if (n >= bufferLength) {
    n = bufferLength - 1;
  }
  memcpy(buffer, response.payload, n);
  buffer[n] = '\0';
  return true;
}

bool R200::setSelectParameter(const uint8_t *payload, uint16_t payloadLength) {
  return command(CMD_SetSelectParameter, payload, payloadLength, nullptr) == Status_Ok;
}

bool R200::getSelectParameter(uint8_t *payload, uint16_t &payloadLength, uint16_t maxPayloadLength, unsigned long timeout) {
  Frame response;
  Status status = command(CMD_GetSelectParameter, nullptr, 0, &response, timeout);
  return status == Status_Ok && copyPayload(response, payload, payloadLength, maxPayloadLength);
}

bool R200::sendSelectInstruction() {
  return command(CMD_SetSendSelectInstruction, nullptr, 0, nullptr) == Status_Ok;
}

bool R200::getQueryParameters(uint8_t *payload, uint16_t &payloadLength, uint16_t maxPayloadLength, unsigned long timeout) {
  Frame response;
  Status status = command(CMD_GetQueryParameters, nullptr, 0, &response, timeout);
  return status == Status_Ok && copyPayload(response, payload, payloadLength, maxPayloadLength);
}

bool R200::setQueryParameters(const uint8_t *payload, uint16_t payloadLength) {
  return command(CMD_SetQueryParameters, payload, payloadLength, nullptr) == Status_Ok;
}

bool R200::readLabel(MemoryBank bank, uint16_t wordAddress, uint8_t wordCount, uint32_t accessPassword,
                     uint8_t *data, uint16_t &dataLength, uint16_t maxDataLength, unsigned long timeout) {
  uint8_t payload[8] = {0};
  uint16_t pos = 0;
  appendU32(payload, pos, accessPassword);
  payload[pos++] = static_cast<uint8_t>(bank);
  appendU16(payload, pos, wordAddress);
  payload[pos++] = wordCount;

  Frame response;
  Status status = command(CMD_ReadLabel, payload, pos, &response, timeout);
  return status == Status_Ok && copyPayload(response, data, dataLength, maxDataLength);
}

bool R200::writeLabel(MemoryBank bank, uint16_t wordAddress, const uint8_t *data, uint8_t wordCount,
                      uint32_t accessPassword, unsigned long timeout) {
  const uint16_t byteCount = static_cast<uint16_t>(wordCount) * 2;
  if (data == nullptr || byteCount > 64) {
    _lastError = Status_BufferTooSmall;
    return false;
  }

  uint8_t payload[4 + 1 + 2 + 1 + 64] = {0};
  uint16_t pos = 0;
  appendU32(payload, pos, accessPassword);
  payload[pos++] = static_cast<uint8_t>(bank);
  appendU16(payload, pos, wordAddress);
  payload[pos++] = wordCount;
  memcpy(&payload[pos], data, byteCount);
  pos += byteCount;

  return command(CMD_WriteLabel, payload, pos, nullptr, timeout) == Status_Ok;
}

bool R200::lockLabel(uint32_t accessPassword, uint32_t mask, uint32_t action, unsigned long timeout) {
  uint8_t payload[12] = {0};
  uint16_t pos = 0;
  appendU32(payload, pos, accessPassword);
  appendU32(payload, pos, mask);
  appendU32(payload, pos, action);
  return command(CMD_LockLabel, payload, pos, nullptr, timeout) == Status_Ok;
}

bool R200::killTag(uint32_t killPassword, unsigned long timeout) {
  uint8_t payload[4] = {0};
  uint16_t pos = 0;
  appendU32(payload, pos, killPassword);
  return command(CMD_KillTag, payload, pos, nullptr, timeout) == Status_Ok;
}

bool R200::setWorkArea(uint8_t regionCode) {
  return command(CMD_SetWorkArea, &regionCode, 1, nullptr) == Status_Ok;
}

bool R200::setWorkingChannel(uint8_t channelIndex) {
  return command(CMD_SetWorkingChannel, &channelIndex, 1, nullptr) == Status_Ok;
}

bool R200::getWorkingChannel(uint8_t &channelIndex, unsigned long timeout) {
  Frame response;
  Status status = command(CMD_GetWorkingChannel, nullptr, 0, &response, timeout);
  if (status != Status_Ok || response.length < 1) {
    return false;
  }
  channelIndex = response.payload[0];
  return true;
}

bool R200::setAutoFrequencyHopping(const uint8_t *channels, uint8_t channelCount) {
  if (channelCount > 0 && channels == nullptr) {
    _lastError = Status_BufferTooSmall;
    return false;
  }
  if (channelCount + 1 > R200_MAX_PAYLOAD_LENGTH) {
    _lastError = Status_BufferTooSmall;
    return false;
  }

  uint8_t payload[1 + 64] = {0};
  payload[0] = channelCount;
  if (channelCount > 0) {
    memcpy(&payload[1], channels, channelCount);
  }
  return command(CMD_SetAutoFrequencyHopping, payload, channelCount + 1, nullptr) == Status_Ok;
}

bool R200::clearAutoFrequencyHopping() {
  uint8_t count = 0;
  return command(CMD_SetAutoFrequencyHopping, &count, 1, nullptr) == Status_Ok;
}

bool R200::getTransmitPower(uint16_t &centiDbm, unsigned long timeout) {
  Frame response;
  Status status = command(CMD_AcquireTransmitPower, nullptr, 0, &response, timeout);
  if (status != Status_Ok || response.length < 2) {
    return false;
  }
  centiDbm = (static_cast<uint16_t>(response.payload[0]) << 8) | response.payload[1];
  return true;
}

bool R200::setTransmitPower(uint16_t centiDbm) {
  uint8_t payload[2] = {static_cast<uint8_t>(centiDbm >> 8), static_cast<uint8_t>(centiDbm & 0xFF)};
  return command(CMD_SetTransmitPower, payload, sizeof(payload), nullptr) == Status_Ok;
}

bool R200::setTransmitPowerDbm(float dbm) {
  if (dbm < 0.0f) {
    dbm = 0.0f;
  }
  uint16_t centiDbm = static_cast<uint16_t>(dbm * 100.0f + 0.5f);
  return setTransmitPower(centiDbm);
}

bool R200::setTransmitContinuousCarrier(bool enable) {
  uint8_t payload = enable ? 0xFF : 0x00;
  return command(CMD_SetTransmitContinuousCarrier, &payload, 1, nullptr) == Status_Ok;
}

bool R200::getReceiverDemodulatorParameters(uint8_t *payload, uint16_t &payloadLength, uint16_t maxPayloadLength, unsigned long timeout) {
  Frame response;
  Status status = command(CMD_GetReceiverDemodulatorParameters, nullptr, 0, &response, timeout);
  return status == Status_Ok && copyPayload(response, payload, payloadLength, maxPayloadLength);
}

bool R200::setReceiverDemodulatorParameters(const uint8_t *payload, uint16_t payloadLength) {
  return command(CMD_SetReceiverDemodulatorParameters, payload, payloadLength, nullptr) == Status_Ok;
}

bool R200::testRFInputBlockingSignal(uint8_t *payload, uint16_t &payloadLength, uint16_t maxPayloadLength, unsigned long timeout) {
  Frame response;
  Status status = command(CMD_TestRFInputBlockingSignal, nullptr, 0, &response, timeout);
  return status == Status_Ok && copyPayload(response, payload, payloadLength, maxPayloadLength);
}

bool R200::testChannelRSSI(uint8_t channelIndex, uint8_t &rssi, unsigned long timeout) {
  Frame response;
  Status status = command(CMD_TestChannelRSSI, &channelIndex, 1, &response, timeout);
  if (status != Status_Ok || response.length < 1) {
    return false;
  }
  rssi = response.payload[0];
  return true;
}

bool R200::controlIOPort(uint8_t port, bool high) {
  uint8_t payload[2] = {port, static_cast<uint8_t>(high ? 0x01 : 0x00)};
  return command(CMD_ControlIOPort, payload, sizeof(payload), nullptr) == Status_Ok;
}

bool R200::moduleSleep() {
  return command(CMD_ModuleSleep, nullptr, 0, nullptr) == Status_Ok;
}

bool R200::setModuleIdleSleepTime(uint8_t minutes) {
  if (minutes > 30) {
    minutes = 30;
  }
  return command(CMD_SetModuleIdleSleepTime, &minutes, 1, nullptr) == Status_Ok;
}

bool R200::setIdleMode(bool enable, uint8_t idleTime, uint8_t reserved) {
  uint8_t payload[3] = {static_cast<uint8_t>(enable ? 0x01 : 0x00), reserved, idleTime};
  return command(CMD_EnterIdleMode, payload, sizeof(payload), nullptr) == Status_Ok;
}

bool R200::nxpChangeConfig(uint32_t accessPassword, uint16_t configWord, uint8_t *payloadOut, uint16_t &payloadLength, uint16_t maxPayloadLength, unsigned long timeout) {
  uint8_t payload[6] = {0};
  uint16_t pos = 0;
  appendU32(payload, pos, accessPassword);
  appendU16(payload, pos, configWord);
  Frame response;
  Status status = command(CMD_NXPChangeConfig, payload, pos, &response, timeout);
  return status == Status_Ok && copyPayload(response, payloadOut, payloadLength, maxPayloadLength);
}

bool R200::nxpReadProtect(uint32_t accessPassword, bool reset, unsigned long timeout) {
  uint8_t payload[5] = {0};
  uint16_t pos = 0;
  appendU32(payload, pos, accessPassword);
  payload[pos++] = reset ? 0x01 : 0x00;
  return command(CMD_NXPReadProtect, payload, pos, nullptr, timeout) == Status_Ok;
}

bool R200::nxpChangeEAS(uint32_t accessPassword, bool enable, unsigned long timeout) {
  uint8_t payload[5] = {0};
  uint16_t pos = 0;
  appendU32(payload, pos, accessPassword);
  payload[pos++] = enable ? 0x01 : 0x00;
  return command(CMD_NXPChangeEAS, payload, pos, nullptr, timeout) == Status_Ok;
}

bool R200::nxpEASAlarm(uint8_t *payload, uint16_t &payloadLength, uint16_t maxPayloadLength, unsigned long timeout) {
  Frame response;
  Status status = command(CMD_NXPEASAlarm, nullptr, 0, &response, timeout);
  return status == Status_Ok && copyPayload(response, payload, payloadLength, maxPayloadLength);
}

bool R200::impinjMonzaQT(uint8_t commandCode, const uint8_t *payload, uint16_t payloadLength, Frame *response, unsigned long timeout) {
  if (commandCode != CMD_ImpinjMonzaQTReadWrite && commandCode != CMD_ImpinjMonzaQTControl) {
    _lastError = Status_CommandError;
    return false;
  }
  return command(commandCode, payload, payloadLength, response, timeout) == Status_Ok;
}

bool R200::blockPermalock(uint8_t commandCode, const uint8_t *payload, uint16_t payloadLength, Frame *response, unsigned long timeout) {
  if (commandCode != CMD_BlockPermalock && commandCode != CMD_BlockPermalock2) {
    _lastError = Status_CommandError;
    return false;
  }
  return command(commandCode, payload, payloadLength, response, timeout) == Status_Ok;
}

uint8_t R200::calculateCheckSum(const uint8_t *buffer) const {
  uint16_t paramLength = static_cast<uint16_t>(buffer[3]) << 8;
  paramLength += buffer[4];

  uint16_t check = 0;
  for (uint16_t i = 1; i < paramLength + 5; i++) {
    check += buffer[i];
  }
  return static_cast<uint8_t>(check & 0xFF);
}

uint8_t R200::calculateCheckSum(uint8_t type, uint8_t commandCode, const uint8_t *payload, uint16_t payloadLength) const {
  uint16_t check = type;
  check += commandCode;
  check += static_cast<uint8_t>(payloadLength >> 8);
  check += static_cast<uint8_t>(payloadLength & 0xFF);
  for (uint16_t i = 0; i < payloadLength; i++) {
    check += payload[i];
  }
  return static_cast<uint8_t>(check & 0xFF);
}

uint16_t R200::arrayToUint16(const uint8_t *array) const {
  uint16_t value = *array;
  value <<= 8;
  value += *(array + 1);
  return value;
}

void R200::appendU16(uint8_t *payload, uint16_t &pos, uint16_t value) {
  payload[pos++] = static_cast<uint8_t>(value >> 8);
  payload[pos++] = static_cast<uint8_t>(value & 0xFF);
}

void R200::appendU32(uint8_t *payload, uint16_t &pos, uint32_t value) {
  payload[pos++] = static_cast<uint8_t>((value >> 24) & 0xFF);
  payload[pos++] = static_cast<uint8_t>((value >> 16) & 0xFF);
  payload[pos++] = static_cast<uint8_t>((value >> 8) & 0xFF);
  payload[pos++] = static_cast<uint8_t>(value & 0xFF);
}

bool R200::copyPayload(const Frame &frame, uint8_t *payload, uint16_t &payloadLength, uint16_t maxPayloadLength) {
  if (frame.length > maxPayloadLength) {
    payloadLength = frame.length;
    return false;
  }
  if (frame.length > 0 && payload == nullptr) {
    payloadLength = frame.length;
    return false;
  }
  if (frame.length > 0) {
    memcpy(payload, frame.payload, frame.length);
  }
  payloadLength = frame.length;
  return true;
}
