#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

namespace {

constexpr uint8_t kMagic0 = 'D';
constexpr uint8_t kMagic1 = 'L';
constexpr uint8_t kVersion = 1;
constexpr size_t kHeaderSize = 8;
constexpr size_t kMaxPayload = 2048;

constexpr uint8_t kMsgConfig = 0x01;
constexpr uint8_t kMsgSend = 0x02;
constexpr uint8_t kMsgStatus = 0x81;
constexpr uint8_t kMsgAck = 0x82;
constexpr uint8_t kMsgError = 0x83;

WiFiUDP udp;
IPAddress droneIp(192, 168, 1, 1);
uint16_t dronePort = 7099;
bool bridgeReady = false;
uint16_t txSeq = 0;

uint8_t rxBuffer[kHeaderSize + kMaxPayload + 2];
size_t rxLen = 0;

uint16_t crc16Ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

uint16_t readLe16(const uint8_t *data) {
  return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
}

void writeLe16(uint8_t *data, uint16_t value) {
  data[0] = static_cast<uint8_t>(value & 0xFF);
  data[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void sendFrame(uint8_t type, const uint8_t *payload, uint16_t payloadLen) {
  uint8_t header[kHeaderSize];
  header[0] = kMagic0;
  header[1] = kMagic1;
  header[2] = kVersion;
  header[3] = type;
  writeLe16(header + 4, txSeq++);
  writeLe16(header + 6, payloadLen);

  uint16_t crc = crc16Ccitt(header, sizeof(header));
  if (payloadLen > 0) {
    uint8_t temp[kHeaderSize + kMaxPayload];
    memcpy(temp, header, sizeof(header));
    memcpy(temp + sizeof(header), payload, payloadLen);
    crc = crc16Ccitt(temp, sizeof(header) + payloadLen);
  }

  uint8_t crcBytes[2];
  writeLe16(crcBytes, crc);
  Serial.write(header, sizeof(header));
  if (payloadLen > 0) {
    Serial.write(payload, payloadLen);
  }
  Serial.write(crcBytes, sizeof(crcBytes));
}

void sendText(uint8_t type, const String &text) {
  sendFrame(type, reinterpret_cast<const uint8_t *>(text.c_str()), text.length());
}

String fieldAt(const uint8_t *payload, uint16_t len, int index) {
  int current = 0;
  uint16_t start = 0;
  for (uint16_t i = 0; i <= len; ++i) {
    if (i == len || payload[i] == 0) {
      if (current == index) {
        String value;
        value.reserve(i - start);
        for (uint16_t j = start; j < i; ++j) {
          value += static_cast<char>(payload[j]);
        }
        return value;
      }
      current += 1;
      start = i + 1;
    }
  }
  return "";
}

void handleConfig(const uint8_t *payload, uint16_t len) {
  const String ssid = fieldAt(payload, len, 0);
  const String password = fieldAt(payload, len, 1);
  const String ip = fieldAt(payload, len, 2);
  const String port = fieldAt(payload, len, 3);

  if (ssid.isEmpty()) {
    sendText(kMsgError, "missing ssid");
    return;
  }
  if (!droneIp.fromString(ip)) {
    sendText(kMsgError, "invalid drone ip");
    return;
  }
  dronePort = static_cast<uint16_t>(port.toInt());
  if (dronePort == 0) {
    sendText(kMsgError, "invalid drone port");
    return;
  }

  bridgeReady = false;
  udp.stop();
  WiFi.disconnect(true, true);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid.c_str(), password.isEmpty() ? nullptr : password.c_str());

  const uint32_t started = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - started < 10000) {
    delay(50);
  }
  if (WiFi.status() != WL_CONNECTED) {
    sendText(kMsgError, "wifi connect timeout");
    return;
  }

  udp.begin(0);
  bridgeReady = true;
  sendText(kMsgStatus, "READY " + WiFi.localIP().toString() + " -> " + droneIp.toString() + ":" + String(dronePort));
}

void handleSend(const uint8_t *payload, uint16_t len) {
  if (!bridgeReady || WiFi.status() != WL_CONNECTED) {
    sendText(kMsgError, "bridge not ready");
    bridgeReady = false;
    return;
  }
  udp.beginPacket(droneIp, dronePort);
  udp.write(payload, len);
  if (udp.endPacket() != 1) {
    sendText(kMsgError, "udp send failed");
  }
}

void handleFrame(uint8_t type, const uint8_t *payload, uint16_t payloadLen) {
  switch (type) {
    case kMsgConfig:
      handleConfig(payload, payloadLen);
      break;
    case kMsgSend:
      handleSend(payload, payloadLen);
      break;
    default:
      sendText(kMsgError, "unknown message type");
      break;
  }
}

void compactRx(size_t consumed) {
  if (consumed >= rxLen) {
    rxLen = 0;
    return;
  }
  memmove(rxBuffer, rxBuffer + consumed, rxLen - consumed);
  rxLen -= consumed;
}

void processRx() {
  while (rxLen >= kHeaderSize) {
    if (rxBuffer[0] != kMagic0 || rxBuffer[1] != kMagic1) {
      compactRx(1);
      continue;
    }
    if (rxBuffer[2] != kVersion) {
      compactRx(2);
      continue;
    }

    const uint8_t type = rxBuffer[3];
    const uint16_t payloadLen = readLe16(rxBuffer + 6);
    if (payloadLen > kMaxPayload) {
      compactRx(2);
      sendText(kMsgError, "payload too large");
      continue;
    }

    const size_t frameLen = kHeaderSize + payloadLen + 2;
    if (rxLen < frameLen) {
      return;
    }

    const uint16_t expected = readLe16(rxBuffer + kHeaderSize + payloadLen);
    const uint16_t actual = crc16Ccitt(rxBuffer, kHeaderSize + payloadLen);
    if (expected != actual) {
      compactRx(2);
      sendText(kMsgError, "crc mismatch");
      continue;
    }

    handleFrame(type, rxBuffer + kHeaderSize, payloadLen);
    compactRx(frameLen);
  }
}

void readSerial() {
  while (Serial.available() > 0 && rxLen < sizeof(rxBuffer)) {
    rxBuffer[rxLen++] = static_cast<uint8_t>(Serial.read());
  }
  if (rxLen == sizeof(rxBuffer)) {
    rxLen = 0;
    sendText(kMsgError, "rx overflow");
  }
  processRx();
}

}  // namespace

void setup() {
  Serial.begin(921600);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  sendText(kMsgStatus, "BOOT");
}

void loop() {
  readSerial();
  if (bridgeReady && WiFi.status() != WL_CONNECTED) {
    bridgeReady = false;
    sendText(kMsgError, "wifi disconnected");
  }
}
