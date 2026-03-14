#include "joystick.h"

joystick::joystick(int X_PIN1, int Y_PIN1, int BUTTON_PIN1)
{
    X_PIN = X_PIN1;
    Y_PIN = Y_PIN1;
    BUTTON_PIN = BUTTON_PIN1;
    pinMode(X_PIN, INPUT);
    pinMode(Y_PIN, INPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    previousButtonState = HIGH;
    buttonState = false;
}

void joystick::readData()
{
    x = map(analogRead(X_PIN), 1023, 0, 0, 1023);
    y = map(analogRead(Y_PIN), 1023, 0, 0, 1023);

    /*  bool currentButton = (digitalRead(BUTTON_PIN) == LOW);

      // True only on press event (falling edge with INPUT_PULLUP)
      buttonState = (currentButton && !previousButtonState);

      previousButtonState = currentButton; */
}

bool joystick::wasPressed()
{
    bool pressed = buttonState;
    buttonState = false;
    return pressed;
}