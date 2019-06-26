#ifndef __SMARTAUDIO_H__
#define __SMARTAUDIO_H__

#include "Arduino.h"
#include <SoftwareSerial.h>

class SmartAudio {
public:
  SmartAudio(const int rxPin, const int txPin);
  void begin();

  void send(uint8_t* buf, const uint8_t len);
  void debugRx();

  void setChannel(const uint8_t channel);
  void setMode(const uint8_t mode);
  void getSettings();

private:
  int _rxPin;
  int _txPin;
  SoftwareSerial _softSerial;

  uint8_t crc8(const uint8_t* ptr, uint8_t len);
};

#endif // __SMARTAUDIO_H__
