#include <Arduino.h>
#include "RotEncoder.h"

Encoder::Encoder(int clk1, int dt1, int sw1)
{
    clk = clk1;
    dt = dt1;
    sw = sw1;
    pinMode(clk, INPUT);
    pinMode(dt, INPUT);
    pinMode(sw, INPUT_PULLUP);

    aLastState = digitalRead(clk);
    buttonLastState = digitalRead(sw);
    bState = digitalRead(dt);
    lastDebounceTime = 0;
    lastButtonDebounceTime = 0; // Initialize
}

int Encoder::compare()
{
    if (aState != aLastState && aState == HIGH)
    {
        unsigned long currentTime = millis();
        if ((currentTime - lastDebounceTime) > debounceDelay)
        {
            lastDebounceTime = currentTime;

            // Read bState RIGHT NOW instead of using the stored value
            int currentB = digitalRead(dt);

            if (currentB != aState)
            {
                return +1;
            }
            else
            {
                return -1;
            }
        }
    }
    return 0;
}
void Encoder::readStates()
{
    aState = digitalRead(clk);
    bState = digitalRead(dt);
    buttonState = digitalRead(sw);
}

void Encoder::resetStates()
{
    aLastState = aState;
    buttonLastState = buttonState;
}

bool Encoder::readButton()
{
    if (buttonState == LOW && buttonLastState == HIGH)
    {
        unsigned long currentTime = millis();
        if ((currentTime - lastButtonDebounceTime) > buttonDebounceDelay)
        {
            lastButtonDebounceTime = currentTime; // Update debounce time
            return true;
        }
    }
    return false;
}
