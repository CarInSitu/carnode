#include "ESP8266WiFi.h"
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <EEPROM.h>

#include <Servo.h>
#include <Ticker.h>

#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>

#include "OTA.h"
#include "FrontHeadlights.h"

#include <NeoPixelBus.h>

#define VERSION_MAJOR 0
#define VERSION_MINOR 20
#define VERSION_PATCH 0

#define NODE_TYPE_CAR 0

#define SENSOR_RSSI 0x80
#define SENSOR_IR 0x81

// WiFi access point connection configuration
const char* ssid = "CarInSitu";
const char* password = "Roulez jeunesse !";

// Servo for steering and throttle
Servo steeringServo;
Servo throttleServo;
Ticker throttleWatchdog;
bool throttleWatchdogBone = false;

// Network
byte mac[6];

WiFiUDP Udp;
unsigned int localUdpPort = 4210;
char incomingPacket[256];
char outgoingPacket[256];
IPAddress cisServerIpAddress;
WiFiClient cisClient;

int steeringLimitLeft;
int steeringLimitRight;
int steeringTrim;

// IR
IRrecv irrecv(IR_PIN);
decode_results results;

// OTA
OTA ota;

// Front headlights
FrontHeadlights frontHeadlights(FRONTHEADLIGHTS_PIN);

// Back/top headlight(s)
NeoPixelBus<NeoGrbFeature, NeoEsp8266Uart1800KbpsMethod> strip(LED_STRIP_COUNT, LED_STRIP_PIN);

void print_wifi_status(int status) {
  switch (status) {
  case WL_CONNECTED:
    Serial.print("Connected to a WiFi network");
    break;
  case WL_NO_SHIELD:
    Serial.print("No WiFi shield is present");
    break;
  case WL_IDLE_STATUS:
    Serial.print("Idle");
    break;
  case WL_NO_SSID_AVAIL:
    Serial.print("No SSID are available");
    break;
  case WL_SCAN_COMPLETED:
    Serial.print("Scan networks is completed");
    break;
  case WL_CONNECT_FAILED:
    Serial.print("Connection fails for all the attempts");
    break;
  case WL_CONNECTION_LOST:
    Serial.print("Connection is lost");
    break;
  case WL_DISCONNECTED:
    Serial.print("Disconnected");
    break;
  }
}

void setup() {
  // Turn off front headlights
  frontHeadlights.enable(false);
  strip.Begin();
  strip.Show(); // Initialize all pixels to 'off'

  // Init serial monitoring
  Serial.begin(76800);
  Serial.printf("\nCarNode version: %d.%d.%d\n", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);

  // EEPROM
  EEPROM.begin(sizeof(steeringTrim));

  // Init WiFi
  WiFi.disconnect();
  delay(10);

  WiFi.macAddress(mac);
  // Start WiFi server and wait for connection
  String hostname = String("CarNode-") + String(mac[5], HEX) + String(mac[4], HEX) + String(mac[3], HEX) + String(mac[2], HEX) + String(mac[1], HEX) + String(mac[0], HEX);
  WiFi.hostname(hostname);
  WiFi.begin(ssid, password);
  Serial.printf("WiFi: Hostname: %s\n", WiFi.hostname().c_str());

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.printf("WiFi: Connecting... (status: %d): ", WiFi.status());
    print_wifi_status(WiFi.status());
    Serial.print("\n");
  }

  Serial.printf("WiFi: Connected to SSID: \"%s\" with IP: %s\n", ssid, WiFi.localIP().toString().c_str());

  // MDNS
  MDNS.begin(hostname);

  //Init servos
  steeringServo.attach(STEERING_SERVO_PIN);
  throttleServo.attach(THROTTLE_SERVO_PIN);

  steeringServo.writeMicroseconds(1500);
  throttleServo.writeMicroseconds(1500);

  EEPROM.get(0, steeringTrim);
  computeSteeringLimits();
  setSteering(0);

  Udp.begin(localUdpPort);
  Serial.printf("UDP: Listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);

  // IR
  irrecv.enableIRIn(); // Start the receiver

  // Throttle command watchdog
  throttleWatchdog.attach_ms(200, []() {
    if(throttleWatchdogBone) {
      throttleWatchdogBone = false;
    } else {
      throttleServo.writeMicroseconds(1500);
    }
  });

  // OTA
  ota.begin(hostname);
}

void loop() {
  if (cisClient.connected()) {
    processTcp();
    processUdp();
    processIr();
    sendSensorsData();
  } else {
    ota.process();
    searchCisServer();
  }

  // Give some CPU to process internal things...
  // UDP is instable without this...
  yield();
}

void searchCisServer() {
  int n = MDNS.queryService("cisserver", "tcp");
  if (n) {
    // Connect to the first available CisServer
    cisServerIpAddress = MDNS.IP(0);
    Serial.printf("MDNS: CIS server discovered: %s\n", cisServerIpAddress.toString().c_str());
    if (!cisClient.connect(cisServerIpAddress, MDNS.port(0))) {
      Serial.println("TCP: Connection failed!");
    } else {
      Serial.println("TCP: Connection established.\n");
    }
  } else {
    Serial.printf("MDNS: Waiting for CIS server...\n");
  }

  if (cisClient.connected()) {
    frontHeadlights.enable(true);
    RgbColor color(127, 0, 0);
    strip.SetPixelColor(0, color);
    strip.Show();
  } else {
    static bool enable = false;
    enable = !enable;
    frontHeadlights.enable(enable);
    if(enable) {
      RgbColor color(0, 127, 0);
      strip.SetPixelColor(0, color);
    } else {
      RgbColor color(0, 0, 127);
      strip.SetPixelColor(0, color);
    }
    strip.Show();
  }
}

#define PING            0x00
#define VERSION         0x01
#define VIDEO_CHANNEL   0x05
#define TRIM_STEERING   0x20
#define WRITE_SETTINGS  0xf0
#define INVALID_COMMAND 0xff

byte incomingTcpFrame[64];
byte* incomingTcpFramePtr = incomingTcpFrame;

void processTcp() {
  static int remainingBytesForCommand = -1;
  static byte command;

  while (int availableBytes = cisClient.available()) {
    if (remainingBytesForCommand == -1) { // We are waiting for a command
      command = cisClient.read();
      incomingTcpFramePtr = incomingTcpFrame;
      switch (command) {
      case PING: {
        cisClient.write(PING);
        break;
      }
      case VERSION: {
        outgoingPacket[0] = VERSION;       // repeat command code
        outgoingPacket[1] = NODE_TYPE_CAR; // says im a car node
        outgoingPacket[2] = VERSION_MAJOR; // says my firmware version
        outgoingPacket[3] = VERSION_MINOR;
        outgoingPacket[4] = VERSION_PATCH;
        Serial.printf("Replied to VERSION request\n");
        cisClient.write(outgoingPacket, 5);
        break;
      }
      case VIDEO_CHANNEL: {
        remainingBytesForCommand = 1;
        break;
      }
      case TRIM_STEERING: {
        remainingBytesForCommand = 1;
        break;
      }
      case WRITE_SETTINGS: {
        EEPROM.commit();
        break;
      }
      default: {
        // Unknown command
        Serial.printf("TCP: Unknown command: 0x%02x (%d available bytes)\n", command, availableBytes);
        remainingBytesForCommand = availableBytes - 1;
        command = INVALID_COMMAND;
      }
      }
    } else {
      *incomingTcpFramePtr = cisClient.read();
      remainingBytesForCommand--;

      if (command == INVALID_COMMAND) {
        // Trash any byte
        incomingTcpFramePtr = incomingTcpFrame;
      } else {
        ++incomingTcpFramePtr;
      }

      if (remainingBytesForCommand == 0) {
        switch (command) {
        case VIDEO_CHANNEL: {
          uint8_t* uint8_value = (uint8_t*)&incomingTcpFrame[0];
          // FIXME Set VTX channel
          break;
        }
        case TRIM_STEERING: {
          int8_t* int8_value = (int8_t*)&incomingTcpFrame[0];
          steeringTrim += *int8_value;
          EEPROM.put(0, steeringTrim);
          computeSteeringLimits();
        }
        default: {
          // Nothing to do with invalid command
        }
        }
        remainingBytesForCommand = -1;
      }
    }
  }
}

void processUdp() {
  // UDP: receive incoming packets
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    // Serial.printf("UDP: Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    int len = Udp.read(incomingPacket, 255);
    // Serial.printf("UDP packet contents: %s\n", incomingPacket);
    processIncomingPackets(len);
  }
}

void processIr() {
  // IR: Grab IR decoded results
  if (irrecv.decode(&results)) {
    serialPrintUint64(results.value, HEX);
    Serial.println("");

    sendUdpSensorData(SENSOR_IR, &(results.value), sizeof(results.value));

    irrecv.resume(); // Receive the next value
  }
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

#define STEERING 0x10
#define THROTTLE 0x11

void processIncomingPackets(const int len) {
  int16_t* int_value = (int16_t*)&incomingPacket[1];
  switch (incomingPacket[0]) {
  case STEERING:
    setSteering(*int_value);
    break;
  case THROTTLE:
    setThrottle(*int_value);
    break;
  default:
    Serial.printf("UDP Unknown command: 0x%02x\n", incomingPacket[0]);
  }
}

void setSteering(int value) {
  value = map(value, -32768, 32767, steeringLimitLeft, steeringLimitRight);
  steeringServo.writeMicroseconds(value);
}

void setThrottle(int value) {
  if (value == 0) {
    value = 1500;
  } else if (value > 0) {
    value = map(value, 0, 32767, 1576, 2000);
  } else if (value < 0) {
    value = map(value, -32768, 0, 1000, 1423);
  }
  throttleServo.writeMicroseconds(value);
  throttleWatchdogBone = true;
}

void sendUdpPacket(const int len) {
  Udp.beginPacket(cisServerIpAddress, 4200);
  Udp.write(outgoingPacket, len);
  Udp.endPacket();
  // Give some CPU to process internal things...
  yield();
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
