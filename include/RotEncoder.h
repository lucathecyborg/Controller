#include <Arduino.h>
#define outputA 6
#define outputB 7
#define buttonOutput 5

class Encoder
{
private:
    int aLastState;
    int aState;
    int bState;
    bool buttonLastState;
    bool buttonState;
    unsigned long lastDebounceTime;
    unsigned long lastButtonDebounceTime;
    unsigned long debounceDelay = 35; // 5ms debounce time
    unsigned long buttonDebounceDelay = 50;

public:
    Encoder();
    void readStates();
    void resetStates();
    int compare();
    bool readButton();
};