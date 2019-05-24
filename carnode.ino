#include "ESP8266WiFi.h"
#include <WiFiUdp.h>
#include <Servo.h>

#define VERSION_MAJOR 0
#define VERSION_MINOR 2
#define VERSION_PATCH 0

#define NODE_TYPE_CAR 0

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

int steeringLimitLeft;
int steeringLimitRight;
int steeringTrim = 0;

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
  Serial.printf("Hostname set to: %s\n", WiFi.hostname().c_str());

  while (WiFi.status() != WL_CONNECTED) {
    delay(2000);
    Serial.println("Connecting..");
    Serial.println(WiFi.status());
  }
  digitalWrite(LED_BUILTIN, LOW);
  Serial.print("Connected to WiFi. IP:");
  Serial.println(WiFi.localIP());

  //Init servos
  steeringServo.attach(5); // D1
  throttleServo.attach(4); // D2

  steeringServo.writeMicroseconds(1500);
  throttleServo.writeMicroseconds(1500);

  computeSteeringLimits();

  Udp.begin(localUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
}

void loop() {
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    // receive incoming UDP packets
    Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    int len = Udp.read(incomingPacket, 255);
    // Serial.printf("UDP packet contents: %s\n", incomingPacket);
    processIncomingPackets(len);
  }
}

void computeSteeringLimits() {
  steeringLimitLeft = 2000 - steeringTrim;
  steeringLimitRight = 1000 - steeringTrim;

  int center = map(0, -32768, 32767, steeringLimitLeft, steeringLimitRight);
  Serial.printf("Steering center is now at %d, left limit at%d, right limit at %d\n", center, steeringLimitLeft, steeringLimitRight);
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
    Udp.beginPacket(Udp.remoteIP(), 4200);
    outgoingPacket[0] = 0x01;          // repeat command code
    outgoingPacket[1] = NODE_TYPE_CAR; // says im a car node
    outgoingPacket[2] = VERSION_MAJOR; // says my firmware version
    outgoingPacket[3] = VERSION_MINOR;
    outgoingPacket[4] = VERSION_PATCH;
    Udp.write(outgoingPacket, 5);
    Udp.endPacket();
    Serial.printf("Replied to DISCOVERY request to %s\n", Udp.remoteIP().toString().c_str());
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
