#ifndef __FRONT_HEADLIGHTS_H__
#define __FRONT_HEADLIGHTS_H__

class FrontHeadlights {
public:
  FrontHeadlights(const int pin);

  void turn(int on);

private:
  int _pin;
};
#endif
