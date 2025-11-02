#include <Arduino.h>



// Button.h
class Button {
private:
  uint8_t pin;
  uint8_t state;
  uint8_t lastReading;
  uint8_t lastState;
  unsigned long lastDebounceTime;
  unsigned long debounceDelay;
  bool pullup;

public:
  Button(uint8_t p, unsigned long debounce = 50, bool enablePullup = true) {
    pin = p;
    debounceDelay = debounce;
    pullup = enablePullup;
    lastReading = HIGH;
    lastState = HIGH;
    lastDebounceTime = 0;
    
    pinMode(pin, pullup ? INPUT_PULLUP : INPUT);
    state = digitalRead(pin);
  }

  void update() {
    uint8_t reading = digitalRead(pin);
    
    if (reading != lastReading) {
      lastDebounceTime = millis();
    }
    
    if ((millis() - lastDebounceTime) > debounceDelay) {
      if (reading != state) {
        lastState = state;
        state = reading;
      }
    }
    
    lastReading = reading;
  }

  bool isPressed() {
    return pullup ? (state == LOW) : (state == HIGH);
  }

  bool isReleased() {
    return !isPressed();
  }

  bool read() {
    return state;
  }

  bool wasPressed() {
    bool pressed = (pullup ? (state == LOW) : (state == HIGH)) && 
                   (pullup ? (lastState == HIGH) : (lastState == LOW));
    return pressed;
  }

  bool wasReleased() {
    bool released = (pullup ? (state == HIGH) : (state == LOW)) && 
                    (pullup ? (lastState == LOW) : (lastState == HIGH));
    return released;
  }
};