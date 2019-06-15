#include "ESP8266WiFi.h"
#include <WiFiUdp.h>
#include <Servo.h>
#include <SoftwareSerial.h>

#define VERSION_MAJOR 0
#define VERSION_MINOR 3
#define VERSION_PATCH 0

#define NODE_TYPE_CAR 0

//Smart audio
SoftwareSerial smartAudioSerial(D8, D7); // RX, TX

// WiFi access point connection configuration
const char* ssid = "CarInSitu";
const char* password = "Roulez jeunesse !";

// Servo for steering and throttle
Servo steeringServo;
Servo throttleServo;

// Network
byte mac[6];

WiFiUDP Udp;
unsigned int localUdpPort = 4210;
char incomingPacket[256];
char outgoingPacket[256];
IPAddress cisServerIpAddress;

int steeringLimitLeft;
int steeringLimitRight;
int steeringTrim = 0;

void setup() {
  // Init serial monitoring
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(10);


  smartAudioSerial.begin(4800);
  // setMode();
  // setFrequency();
  //getSettings();
  setChannel();
    //pinMode(D7, INPUT);
    
//    0xaa, 0x55, 0x07, 0x01, 0x25, 0x37
//    0xaa, 0x55, 0x03, 0x03, 0x25, 0x01, 0xd8
    
//    0xaa, 0x55, 0x07, 0x01, 0x25, 0x37
    
//    0xaa, 0x55, 0x05, 0x01, 0x01, 0xbe
//    0xaa, 0x55, 0x05, 0x01, 0x01, 0xbe
//    aa

  while (1) {
    if (smartAudioSerial.available() > 0) {
      // read the incoming byte:
      char incomingByte = smartAudioSerial.read();
  
      // say what you got:
      Serial.printf("RX: 0x%02x\n", incomingByte);
    }
  }

  // Init WiFi
  WiFi.disconnect();
  delay(10);

  WiFi.macAddress(mac);
  // Start WiFi server and wait for connection
  String hostname = String("CarNode-") + String(mac[5], HEX) + String(mac[4], HEX) + String(mac[3], HEX) + String(mac[2], HEX) + String(mac[1], HEX) + String(mac[0], HEX);
  WiFi.hostname(hostname);
  WiFi.begin(ssid, password);
  Serial.printf("\nHostname set to: %s\n", WiFi.hostname().c_str());

  while (WiFi.status() != WL_CONNECTED) {
    delay(2000);
    Serial.printf("WiFi: Connecting... (status: %d)\n", WiFi.status());
  }
  digitalWrite(LED_BUILTIN, LOW);
  Serial.printf("WiFi: Connected to SSID: \"%s\" with IP: %s\n", ssid, WiFi.localIP().toString().c_str());

  //Init servos
  steeringServo.attach(5); // D1
  throttleServo.attach(4); // D2

  steeringServo.writeMicroseconds(1500);
  throttleServo.writeMicroseconds(1500);

  computeSteeringLimits();

  Udp.begin(localUdpPort);
  Serial.printf("UDP: Listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
}

void setChannel()
{
  //0xaa, 0x55, 0x07, 0x01, 0x25, 0x37
  // set frequency 5732
  // 0x00 -> 5860 -> 5865
  // 0x01 -> 5843 -> 5845
  // 0x0a -> 5765 -> 5752
  // 0x0b -> 5790 -> 
  // 0x0f -> 5865
  // 0x10 -> 5695
  // 0x11 -> 5685
  // 0x12 -> 5665
  // 0x13 -> 5665

  // 0x20 -> 5665
  // 0x21 -> 5695
  // 0x22 -> 5732
  // 0x27 -> 5905
  
  uint8_t frame[] = {0xaa, 0x55, 0x07, 0x01, 0x27, 0x00};
  uint8_t crc = crc8(frame, 5);
  frame[5] = crc;
  pinMode(D7, OUTPUT);
  Serial.printf("\nFrame: ");
  smartAudioSerial.write(frame, sizeof(frame));
  for (int i=0; i<sizeof(frame); i++) {
    Serial.printf("0x%02x ", frame[i]);
  }
  
  Serial.println("\nSet channel done.");
  pinMode(D7, INPUT);
}

void setFrequency() {
  // set frequency 5732
  uint8_t frame[] = {0xaa, 0x55, 0x09, 0x02, 0x16, 0x64, 0x00};
  uint8_t crc = crc8(frame, 6);
  frame[6] = crc;
  pinMode(D7, OUTPUT);
  smartAudioSerial.write(frame, sizeof(frame));
  pinMode(D7, INPUT);
}

void setMode() {
  // set frequency 5732
  uint8_t frame[] = {0xaa, 0x55, 0x0b, 0x01, 0x08, 0x00};
  uint8_t crc = crc8(frame, 5);
  frame[5] = crc;
  pinMode(D7, OUTPUT);
  smartAudioSerial.write(frame, sizeof(frame));
  pinMode(D7, INPUT);
}

void getSettings() {
  uint8_t frame[] = {0xaa, 0x55, 0x03, 0x00, 0x9F};
  //uint8_t crc = crc8(frame, 6);
  //frame[6] = crc;
  pinMode(D7, OUTPUT);
  smartAudioSerial.write(frame, sizeof(frame));
  pinMode(D7, INPUT);
}

void loop() {
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    // receive incoming UDP packets
    Serial.printf("UDP: Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    int len = Udp.read(incomingPacket, 255);
    // Serial.printf("UDP packet contents: %s\n", incomingPacket);
    processIncomingPackets(len);
  }
  sendSensorsData();

  // Give some CPU to process internal things...
  // UDP is instable without this...
  yield();
}

void computeSteeringLimits() {
  steeringLimitLeft = 2000 - steeringTrim;
  steeringLimitRight = 1000 - steeringTrim;

  int center = map(0, -32768, 32767, steeringLimitLeft, steeringLimitRight);
  Serial.printf("Config: Steering center is now at %d, left limit at %d, right limit at %d\n", center, steeringLimitLeft, steeringLimitRight);
}

#define DISCOVERY_REQUEST 0x01
#define STEERING 0x10
#define THROTTLE 0x11
#define TRIM_STEERING 0x20

void processIncomingPackets(const int len) {
  int16_t* int_value = (int16_t*)&incomingPacket[1];
  int value;
  switch (incomingPacket[0]) {
  case DISCOVERY_REQUEST:
    cisServerIpAddress = Udp.remoteIP();
    outgoingPacket[0] = DISCOVERY_REQUEST; // repeat command code
    outgoingPacket[1] = NODE_TYPE_CAR;     // says im a car node
    outgoingPacket[2] = VERSION_MAJOR;     // says my firmware version
    outgoingPacket[3] = VERSION_MINOR;
    outgoingPacket[4] = VERSION_PATCH;
    sendUdpPacket(5);
    Serial.printf("Replied to DISCOVERY request to CIS server: %s\n", cisServerIpAddress.toString().c_str());
    break;
  case STEERING:
    value = map(*int_value, -32768, 32767, steeringLimitLeft, steeringLimitRight);
    steeringServo.writeMicroseconds(value);
    break;
  case THROTTLE:
    value = map(*int_value, -32768, 32767, 1000, 2000);
    throttleServo.writeMicroseconds(value);
    break;
  case TRIM_STEERING:
    int8_t* int8_value = (int8_t*)&incomingPacket[1];
    steeringTrim = *int8_value;
    computeSteeringLimits();
    break;
  }
}

void sendUdpPacket(const int len) {
  Udp.beginPacket(cisServerIpAddress, 4200);
  Udp.write(outgoingPacket, len);
  Udp.endPacket();
}

#define SENSOR_RSSI 0x80

void sendSensorsData() {
  if (!cisServerIpAddress)
    return;

  // Only send sensors data each 100 runs
  static int prescaler = 0;
  prescaler++;
  prescaler %= 100;
  if (prescaler)
    return;

  // RSSI
  outgoingPacket[0] = SENSOR_RSSI;
  int32_t rssi = WiFi.RSSI();
  memcpy(outgoingPacket + 1, &rssi, 4); // Copy int32_t (ie. 4 bytes) starting at outgoingPacket[1] address
  sendUdpPacket(5);
  // Serial.printf("RSSI: %d\n", rssi);
}

/* CRC8 implementation with polynom = x
7+ x
6+ x
4+ x
2+ x
0
(0xD5) */
unsigned char crc8tab[256] = {
0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};
uint8_t crc8(const uint8_t * ptr, uint8_t len)
{
uint8_t crc = 0;
for (uint8_t i=0; i<len; i++) {
crc = crc8tab[crc ^ *ptr++];
}
return crc;
}
