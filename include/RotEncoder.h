#include <Arduino.h>

class RotaryEncoder {
private:
    uint8_t clkPin;
    uint8_t dtPin;
    uint8_t swPin;

    int position;
    int lastPosition;
    uint8_t lastEncoded;
    
    // Button debouncing
    bool buttonPressed;
    uint8_t lastButtonReading;
    uint8_t stableButtonState;
    unsigned long lastDebounceTime;
    const unsigned long debounceDelay = 50;

public:
    RotaryEncoder(uint8_t clk, uint8_t dt, uint8_t sw)
        : clkPin(clk), dtPin(dt), swPin(sw),
          position(0), lastPosition(0), lastEncoded(0),
          buttonPressed(false), lastButtonReading(HIGH), stableButtonState(HIGH),
          lastDebounceTime(0) {}

    void begin() {
        pinMode(clkPin, INPUT_PULLUP);
        pinMode(dtPin, INPUT_PULLUP);
        pinMode(swPin, INPUT_PULLUP);
        
        // Read initial encoder state
        uint8_t MSB = digitalRead(clkPin);
        uint8_t LSB = digitalRead(dtPin);
        lastEncoded = (MSB << 1) | LSB;
        
        lastButtonReading = digitalRead(swPin);
        stableButtonState = lastButtonReading;
    }

    void update() {
        // --- Handle rotation with detent counting ---
        uint8_t MSB = digitalRead(clkPin);
        uint8_t LSB = digitalRead(dtPin);
        uint8_t encoded = (MSB << 1) | LSB;

        // Only trigger on one specific transition per detent
        // This happens when both pins are HIGH (encoded = 3)
        if (encoded == 3 && lastEncoded != 3) {
            // Check which direction we came from
            if (lastEncoded == 1) {  // Came from DT low, CLK high -> CW
                position++;
            }
            else if (lastEncoded == 2) {  // Came from CLK low, DT high -> CCW
                position--;
            }
        }

        lastEncoded = encoded;

        // --- Handle button ---
        uint8_t reading = digitalRead(swPin);
        
        // Detect any change in button reading
        if (reading != lastButtonReading) {
            lastDebounceTime = millis();
        }

        // After debounce delay, check for press
        if ((millis() - lastDebounceTime) > debounceDelay) {
            if (reading != stableButtonState) {
                // State has changed and is stable
                if (stableButtonState == HIGH && reading == LOW) {
                    // Button was pressed
                    buttonPressed = true;
                }
                stableButtonState = reading;
            }
        }
        
        lastButtonReading = reading;
    }

    int getDelta() {
        int delta = position - lastPosition;
        lastPosition = position;
        return delta;
    }

    bool buttonWasPressed() {
        if (buttonPressed) {
            buttonPressed = false;
            return true;
        }
        return false;
    }
};