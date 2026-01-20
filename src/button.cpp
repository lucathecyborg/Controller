#include "button.h"
#include <Arduino.h>

Button::Button(uint8_t buttonPin, unsigned long debounce)
{
  pin = buttonPin;
  debounceDelay = debounce;
  lastDebounceTime = 0;
  pinMode(pin, INPUT_PULLUP);
  currentState = !digitalRead(pin); // Invert for pullup (pressed = LOW)
  previousState = currentState;
  lastReading = currentState;
}

void Button::update()
{
  // Always update previousState to currentState at the start
  // This ensures wasPressed/wasReleased only return true for one update cycle
  previousState = currentState;

  // Invert reading for pullup (pressed = LOW)
  bool reading = !digitalRead(pin);

  // Reset debounce timer if reading changed
  if (reading != lastReading)
  {
    lastDebounceTime = millis();
    lastReading = reading;
  }

  // Only update state if debounce delay has passed
  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    currentState = reading;
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
