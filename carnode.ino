#include "ESP8266WiFi.h"
#include <WiFiUdp.h>
#include <Servo.h>

// WiFi access point connection configuration
const char* ssid = "CarInSitu";
const char* password =  "Roulez jeunesse !";

// Servo for steering and throttle
Servo steeringServo;
Servo throttleServo;

// Network
byte mac[6];

WiFiUDP Udp;
unsigned int localUdpPort = 4210;
char incomingPacket[256];
char replyPacket[] = "Hi there! Got the message :-)";
 
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
  String hostname = String("CarNode-")
    + String(mac[5],HEX)
    + String(mac[4],HEX)
    + String(mac[3],HEX)
    + String(mac[2],HEX)
    + String(mac[1],HEX)
    + String(mac[0],HEX);
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
  
  Udp.begin(localUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
}
 
void loop() {  
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    // receive incoming UDP packets
    Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    int len = Udp.read(incomingPacket, 255);
    // Serial.printf("UDP packet contents: %s\n", incomingPacket);
    processIncomingPackets(len);
  }
}

void processIncomingPackets(const int len)
{
    int16_t* int_value = (int16_t*)&incomingPacket[1];
    int value;
    switch(incomingPacket[0]) {
      case 0x10: // Steering (inverted)
        value = map(*int_value, -32768, 32767, 2000, 1000);
        steeringServo.writeMicroseconds(value);
        break;
      case 0x11: // Throttle
        value = map(*int_value, -32768, 32767, 1000, 2000);
        throttleServo.writeMicroseconds(value);
        break;
    }
}
