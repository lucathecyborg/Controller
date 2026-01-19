#include <Arduino.h>

class joystick
{
private:
    uint16_t x;
    uint16_t y;
    bool buttonState;
    bool previousButtonState;
    int X_PIN;
    int Y_PIN;
    int BUTTON_PIN;

public:
    joystick(int X_PIN1, int Y_PIN1, int BUTTON_PIN1);
    void readData();
    bool wasPressed();
    uint16_t getX() { return x; }
    uint16_t getY() { return y; }
};
