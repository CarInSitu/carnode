#ifndef __FRONT_HEADLIGHTS_H__
#define __FRONT_HEADLIGHTS_H__

class FrontHeadlights {
public:
  FrontHeadlights(const int pin, const bool inverted);

  void turn(const bool on);

private:
  int _pin;
  bool _inverted;
};
#endif
