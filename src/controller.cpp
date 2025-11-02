#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#include "bitmaps.h"
#include "joystick.h"


RF24 radio(9, 10); // CE, CSN


joystick joystickL(A1,A2,22);
joystick joystickR(A3,A4,23);


const byte address[6] = "NODE1";
uint16_t DroneBattery;
uint16_t ControllerBattery = 100; 
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

struct message
{
  uint16_t pot1;
  joystickValues joystickL;
  joystickValues joystickR;
};

message Data;
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void drawDisplay(bool Light, int power)
{
  static int lastPowerPercent = -100; // last shown % on screen
  int powerPercent = map(power, 0, 1020, 0, 100);

  int DroneBatteryIndex = map(DroneBattery, 0, 100, 0, 6);
  DroneBatteryIndex = constrain(DroneBatteryIndex, 0, 6);

  int ControllerBatteryIndex = map(ControllerBattery, 0, 100, 0, 6);
  ControllerBatteryIndex = constrain(ControllerBatteryIndex, 0, 6);

  display.clearDisplay();
  // Only update if the change is meaningful (≥2%)
  if (abs(powerPercent - lastPowerPercent) >= 2)
  {
    lastPowerPercent = powerPercent;
    display.setCursor(87, 37);
    display.print("Power");
    display.setCursor(89, 51);
    display.print(powerPercent);
    display.print('%');
  }
  else
  {
    display.setCursor(87, 37);
    display.print("Power");

    display.setCursor(89, 51);
    display.print(lastPowerPercent);
    display.print('%');
  }
  if(Light==0){
    display.drawBitmap(24, 46, lightOFF, 13, 14, 1);
  }
  else{
    display.drawBitmap(24, 46, lightON, 13, 14, 1);
  }

  display.setTextColor(1);
  display.setTextWrap(false);
  display.setCursor(5, 4);
  display.print("Controller");

  display.setCursor(87, 4);
  display.print("Drone");

  display.drawLine(0, 32, 126, 32, 1);


  display.setCursor(14, 37);
  display.print("Lights");

  const unsigned char *droneIMG;
  memcpy_P(&droneIMG, &batteryList[DroneBatteryIndex], sizeof(droneIMG));

  const unsigned char *controllerIMG;
  memcpy_P(&controllerIMG, &batteryList[ControllerBatteryIndex], sizeof(controllerIMG));

  display.drawBitmap(18, 15, controllerIMG, 24, 16, 1);
  display.drawBitmap(88, 14, droneIMG, 24, 16, 1);
  display.display();
}




void setup()
{
  Serial.begin(9600);
  if (!radio.begin())
  {
    Serial.println("NRF24L01 not responding");
    while (1)
      ;
  }
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(address);
  radio.stopListening();
  display.begin(0x3C, true);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  radio.enableAckPayload();

  pinMode(A0, INPUT);
  
}

void loop()
{


 
  Data.joystickL = joystickL.getValues();
  Data.joystickR = joystickR.getValues();

  Serial.print("X:  ");
  Serial.print(Data.joystickL.x);
  Serial.print("    Y:  ");
  Serial.println(Data.joystickL.y);

  Data.pot1 = analogRead(A0);

  radio.write(&Data, sizeof(Data));
  if (radio.isAckPayloadAvailable())
  {
    radio.read(&DroneBattery, sizeof(DroneBattery));
  }

  Serial.println(Data.pot1);
  drawDisplay(1,Data.pot1);
  delay(100);
}
