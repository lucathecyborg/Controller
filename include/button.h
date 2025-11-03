#include <Arduino.h>

// Button.h
class Button {
private:
  uint8_t pin;
  uint8_t state;
  uint8_t lastReading;
  unsigned long lastDebounceTime;
  unsigned long debounceDelay;
  bool pullup;
  bool pressedFlag;  // Flag to track if press was already consumed
  bool releasedFlag; // Flag to track if release was already consumed

public:
  Button(uint8_t p, unsigned long debounce = 50, bool enablePullup = true) {
    pin = p;
    debounceDelay = debounce;
    pullup = enablePullup;
    lastReading = HIGH;
    lastDebounceTime = 0;
    pressedFlag = false;
    releasedFlag = false;
    
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
        uint8_t oldState = state;
        state = reading;
        
        // Set flags when button state changes
        if (pullup) {
          if (state == LOW && oldState == HIGH) {
            pressedFlag = true;
          } else if (state == HIGH && oldState == LOW) {
            releasedFlag = true;
          }
        } else {
          if (state == HIGH && oldState == LOW) {
            pressedFlag = true;
          } else if (state == LOW && oldState == HIGH) {
            releasedFlag = true;
          }
        }
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
    // Return the flag and clear it
    if (pressedFlag) {
      pressedFlag = false;
      return true;
    }
    return false;
  }

  bool wasReleased() {
    // Return the flag and clear it
    if (releasedFlag) {
      releasedFlag = false;
      return true;
    }
    return false;
  }
};