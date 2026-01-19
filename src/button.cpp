#include "button.h"
#include <Arduino.h>

Button::Button(uint8_t buttonPin, unsigned long debounce)
{
  pin = buttonPin;
  debounceDelay = debounce;
  currentState = false;
  previousState = false;
  lastDebounceTime = 0;
  pinMode(pin, INPUT_PULLUP);
  currentState = !digitalRead(pin); // Invert for pullup (pressed = LOW)
  previousState = currentState;
}

void Button::update()
{
  // Invert reading for pullup (pressed = LOW)
  bool reading = !digitalRead(pin);

  // Check if the button state has changed
  if (reading != currentState)
  {
    lastDebounceTime = millis();
  }

  // Only update state if debounce delay has passed
  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    if (reading != currentState)
    {
      previousState = currentState;
      currentState = reading;
    }
  }
}

bool Button::isPressed()
{
  return currentState;
}

bool Button::isReleased()
{
  return !currentState;
}

bool Button::wasPressed()
{
  return currentState && !previousState;
}

bool Button::wasReleased()
{
  return !currentState && previousState;
}

bool Button::getState()
{
  return currentState;
}
