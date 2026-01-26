#include <Arduino.h>
#pragma once

class toggleSwitch
{
    int pin1;
    int pin2;

public:
    toggleSwitch(int p1, int p2);
    int readOutput();
};

toggleSwitch::toggleSwitch(int p1, int p2)
{
    pin1 = p1;
    pin2 = p2;
    pinMode(pin1, INPUT_PULLUP);
    pinMode(pin2, INPUT_PULLUP);
}

int toggleSwitch::readOutput()
{
    if (digitalRead(pin1) && digitalRead(pin2))
    {
        return 0;
    }
    else if (!digitalRead(pin1))
    {
        return 1;
    }
    else
    {
        return 2;
    }
}
