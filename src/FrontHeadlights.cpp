#include "FrontHeadlights.h"

#include <Arduino.h>

FrontHeadlights::FrontHeadlights(const int pin, const bool inverted) : _pin(pin), _inverted(inverted) {
  pinMode(_pin, OUTPUT);
};

void FrontHeadlights::turn(const bool on) {
  bool pinState = _inverted ? !on : on;
  digitalWrite(_pin, pinState);
};
