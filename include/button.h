#pragma once
#include "Arduino.h"
class Button
{
private:
  uint8_t pin;
  bool currentState;
  bool previousState;
  bool lastReading;
  unsigned long lastDebounceTime;
  unsigned long debounceDelay;

public:
  Button(uint8_t buttonPin, unsigned long debounce = 50);
  void update();
  bool isPressed();
  bool isReleased();
  bool wasPressed();
  bool wasReleased();
  bool getState();
};