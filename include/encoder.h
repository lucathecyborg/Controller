#include <Arduino.h>

// RotaryEncoder.h
class RotaryEncoder {
private:
  uint8_t pinCLK;
  uint8_t pinDT;
  uint8_t pinSW;
  
  int position;
  int lastPosition;
  uint8_t lastCLK;
  uint8_t lastDT;
  
  // Button debouncing
  uint8_t btnState;
  uint8_t btnLastState;
  uint8_t btnLastReading;
  unsigned long btnLastDebounceTime;
  unsigned long btnDebounceDelay;

public:
  RotaryEncoder(uint8_t clk, uint8_t dt, uint8_t sw = 255, unsigned long debounce = 50) {
    pinCLK = clk;
    pinDT = dt;
    pinSW = sw;
    btnDebounceDelay = debounce;
    
    pinMode(pinCLK, INPUT_PULLUP);
    pinMode(pinDT, INPUT_PULLUP);
    if (pinSW != 255) {
      pinMode(pinSW, INPUT_PULLUP);
    }
    
    position = 0;
    lastPosition = 0;
    lastCLK = digitalRead(pinCLK);
    lastDT = digitalRead(pinDT);
    
    btnState = HIGH;
    btnLastState = HIGH;
    btnLastReading = HIGH;
    btnLastDebounceTime = 0;
  }

  void update() {
    // Read encoder rotation
    uint8_t clk = digitalRead(pinCLK);
    uint8_t dt = digitalRead(pinDT);
    
    // Check for rotation on falling edge of CLK
    if (clk != lastCLK && clk == LOW) {
      if (dt == HIGH) {
        position++;  // Clockwise
      } else {
        position--;  // Counter-clockwise
      }
    }
    
    lastCLK = clk;
    lastDT = dt;
    
    // Update button if present
    if (pinSW != 255) {
      uint8_t reading = digitalRead(pinSW);
      
      if (reading != btnLastReading) {
        btnLastDebounceTime = millis();
      }
      
      if ((millis() - btnLastDebounceTime) > btnDebounceDelay) {
        if (reading != btnState) {
          btnLastState = btnState;
          btnState = reading;
        }
      }
      
      btnLastReading = reading;
    }
  }

  int getPosition() {
    return position;
  }

  void setPosition(int pos) {
    position = pos;
    lastPosition = pos;
  }

  int getDelta() {
    int delta = position - lastPosition;
    lastPosition = position;
    return delta;
  }

  bool hasChanged() {
    return position != lastPosition;
  }

  bool buttonPressed() {
    return (pinSW != 255) && (btnState == LOW);
  }

  bool buttonWasPressed() {
    return (pinSW != 255) && (btnState == LOW) && (btnLastState == HIGH);
  }

  bool buttonWasReleased() {
    return (pinSW != 255) && (btnState == HIGH) && (btnLastState == LOW);
  }
};

// Example usage:
RotaryEncoder encoder(2, 3, 4);  // CLK=2, DT=3, SW=4
int volume = 50;  // Volume level 0-100

