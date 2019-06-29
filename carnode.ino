#include "ESP8266WiFi.h"
#include <WiFiUdp.h>

#include <Servo.h>

#include "SmartAudio.h"

#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>

#define VERSION_MAJOR 0
#define VERSION_MINOR 4
#define VERSION_PATCH 1

#define NODE_TYPE_CAR 0

#define SENSOR_RSSI 0x80
#define SENSOR_IR 0x81

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

// Smart audio
SmartAudio smartAudio(D8, D7); // RX, TX

// IR
IRrecv irrecv(D5);
decode_results results;

void setup() {
  // Init serial monitoring
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(10);

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
  steeringServo.attach(D1);
  throttleServo.attach(D2);

  steeringServo.writeMicroseconds(1500);
  throttleServo.writeMicroseconds(1500);

  computeSteeringLimits();

  Udp.begin(localUdpPort);
  Serial.printf("UDP: Listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);

  // IR
  irrecv.enableIRIn(); // Start the receiver

  // SmartAudio
  smartAudio.begin();
  smartAudio.setChannel(32);
}

void loop() {
  // UDP: receive incoming packets
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Serial.printf("UDP: Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    int len = Udp.read(incomingPacket, 255);
    // Serial.printf("UDP packet contents: %s\n", incomingPacket);
    processIncomingPackets(len);
  }

  // IR: Grab IR decoded results
  if (irrecv.decode(&results)) {
    serialPrintUint64(results.value, HEX);
    Serial.println("");

    if (!cisServerIpAddress)
      return;

    sendUdpSensorData(SENSOR_IR, &(results.value), sizeof(results.value));

    irrecv.resume(); // Receive the next value
  }

  sendSensorsData();

  smartAudio.debugRx();

  // Give some CPU to process internal things...
  // UDP is instable without this...
  yield();
}

void sendUdpSensorData(const uint8_t type, const void* data, const int lenght) {
  outgoingPacket[0] = type;
  memcpy(outgoingPacket + 1, data, lenght + 1);
  sendUdpPacket(lenght + 1);
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
  int32_t rssi = WiFi.RSSI();
  sendUdpSensorData(SENSOR_RSSI, &rssi, sizeof(rssi));
}
