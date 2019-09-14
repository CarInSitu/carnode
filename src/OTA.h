#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>

class OTA {
public:
  OTA(){};

  void begin() {
    // Port defaults to 8266
    // ArduinoOTA.setPort(8266);

    // Hostname defaults to esp8266-[ChipID]
    // ArduinoOTA.setHostname("myesp8266");

    // No authentication by default
    // ArduinoOTA.setPassword("admin");

    // Password can be set with it's md5 value as well
    // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
    // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

    ArduinoOTA.onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using
      // SPIFFS.end()
      Serial.println("OTA: Start updating " + type);
    });
    ArduinoOTA.onEnd([]() { Serial.println("\nEnd"); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("OTA: Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("OTA: Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR)
        Serial.println("OTA: Auth Failed");
      else if (error == OTA_BEGIN_ERROR)
        Serial.println("OTA: Begin Failed");
      else if (error == OTA_CONNECT_ERROR)
        Serial.println("OTA: Connect Failed");
      else if (error == OTA_RECEIVE_ERROR)
        Serial.println("OTA: Receive Failed");
      else if (error == OTA_END_ERROR)
        Serial.println("OTA: End Failed");
    });
    ArduinoOTA.begin();
    Serial.println("OTA: Ready");
  };

  void process() { ArduinoOTA.handle(); };
};
