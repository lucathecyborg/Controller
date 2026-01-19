#pragma once

class Button
{
private:
  uint8_t pin;
  bool currentState;
  bool previousState;
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