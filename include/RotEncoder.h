#include <Arduino.h>

class Encoder
{
private:
    int aLastState;
    int aState;
    int bState;
    int clk;
    int dt;
    int sw;
    bool buttonLastState;
    bool buttonState;
    unsigned long lastDebounceTime;
    unsigned long lastButtonDebounceTime;
    unsigned long debounceDelay = 35; // 5ms debounce time
    unsigned long buttonDebounceDelay = 50;

public:
    Encoder(int clk1, int dt1, int sw1);
    void readStates();
    void resetStates();
    int compare();
    bool readButton();
};