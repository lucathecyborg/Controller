#include <Arduino.h>
struct joystickValues
{
  uint16_t x;
  uint16_t y;
  bool button;
};


class joystick{
    private:
    uint16_t x;
    uint16_t y;
    bool button;
    int pinX,pinY,pinButton;
    public:
      joystick(int pX, int pY, int pButton){
        pinX=pX;
        pinY=pY;
        pinButton=pButton;
        pinMode(pinX, INPUT);
        pinMode(pinY, INPUT);
        pinMode(pinButton, INPUT_PULLUP);
    }
    joystickValues getValues() {
             x = analogRead(pinX);
             y = analogRead(pinY);
             button = !digitalRead(pinButton);
             joystickValues values = {x, y, button};
             return values;
    }
    int getX(){
        return analogRead(pinX);
    };

    int getY(){
        return analogRead(pinY);
    };

    bool getButton(){
        return !digitalRead(pinButton);
    };

};