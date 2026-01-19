#include <Arduino.h>
#include "RotEncoder.h"

Encoder::Encoder()
{
    pinMode(outputA, INPUT);
    pinMode(outputB, INPUT);
    pinMode(buttonOutput, INPUT_PULLUP);

    aLastState = digitalRead(outputA);
    buttonLastState = digitalRead(buttonOutput);
    bState = digitalRead(outputB);
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
            int currentB = digitalRead(outputB);

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
    aState = digitalRead(outputA);
    bState = digitalRead(outputB);
    buttonState = digitalRead(buttonOutput);
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
