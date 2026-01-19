#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#include "communication.h"
#include "RotEncoder.h"
#include "joystick.h"
#include "bitmaps.h"

#define THROTTLE_PIN A0
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Timing
unsigned long lastTransmitTime = 0;
const unsigned long TRANSMIT_INTERVAL = 100; // Send every 100ms (10Hz)

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

joystick joystickL(A1, A2, 22);
joystick joystickR(A3, A4, 23);
Encoder PidValueSelector(26, 27, 28);

uint16_t DroneBattery;
int ControllerBattery;

void initDisplay()
{
  display.begin(0x3c, true);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.drawBitmap(18, 0, drone_init_screen, 90, 60, 1);
  display.display();
}

void drawDisplay(int power, bool Light)
{

  static int lastPowerPercent = -100; // last shown % on screen
  int powerPercent = map(power, 0, 1023, 0, 100);

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

  if (Light == 0)
  {
    display.drawBitmap(24, 46, lightOFF, 13, 14, 1);
  }
  else
  {
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
  delay(100);
  Serial.println("Contoller starting...");
  initDisplay();
  delay(1000);
  pinMode(THROTTLE_PIN, INPUT);
  if (!initRadio())
  {
    display.clearDisplay();
    display.drawBitmap(2, 9, radio_failed_screen, 123, 46, 1);
    display.display();
    while (1)
    {
      delay(1000);
    }
  }
}

void loop()
{
  unsigned long now = millis();

  // Read all inputs

  // Transmit at regular intervals
  if (now - lastTransmitTime >= TRANSMIT_INTERVAL)
  {
    lastTransmitTime = now;

    // Transmit data
    bool success = transmitData();
    if (success && radio.isAckPayloadAvailable())
    {
      radio.read(&DroneBattery, sizeof(DroneBattery));
      updateCommStats(success, true);
    }
    else
    {
      updateCommStats(success, false);
    }

    // Update statistics

    // Print data periodically
    if (now - lastPrintTime >= 200)
    {
      lastPrintTime = now;
      printTransmittedData();
    }
  }
}