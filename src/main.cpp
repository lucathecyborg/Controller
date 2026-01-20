#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#include "communication.h"
#include "RotEncoder.h"
#include "joystick.h"
#include "button.h"
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
Button pidAxisSelector(25);
Button calibrationToggle(24);

uint8_t ControllerBattery;

float roll_kp = 0.8, roll_ki = 0.02, roll_kd = 0.4;
float pitch_kp = 0.8, pitch_ki = 0.02, pitch_kd = 0.4;
float yaw_kp = 1.5, yaw_ki = 0.01, yaw_kd = 0.05;

float *pid_values[3][3] = {
    {&pitch_kp, &pitch_ki, &pitch_kd}, // axis 0: Pitch
    {&roll_kp, &roll_ki, &roll_kd},    // axis 1: Roll
    {&yaw_kp, &yaw_ki, &yaw_kd}        // axis 2: Yaw
};

float multipliers[3][3] = {
    {0.5, 0.005, 0.05}, // Pitch P, I, D multipliers
    {0.5, 0.005, 0.05}, // Roll P, I, D multipliers
    {0.5, 0.002, 0.01}  // Yaw P, I, D multipliers
};

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

  int DroneBatteryIndex = map(rxBattery, 0, 100, 0, 6);
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

void setPidSelectorValues(uint8_t axis, float p, float i, float d)
{
  txData.kd = d;
  txData.kp = p;
  txData.ki = i;
  txData.pidAxis = axis;
}

void drawPID(int axis, int variable, float kp, float ki, float kd)
{

  display.clearDisplay();
  display.setTextColor(1);
  display.setTextSize(2);
  display.setTextWrap(false);
  display.setCursor(17, 24);
  display.print("P");

  display.setCursor(58, 23);
  display.print("I");

  display.setCursor(99, 25);
  display.print("D");

  display.setTextSize(1);

  display.setCursor(10, 44);
  display.print(kp);

  display.setCursor(51, 45);
  display.print(ki);

  display.setCursor(92, 46);
  display.print(kd);

  switch (variable)
  {
  case 0:
    display.drawRect(5, 41, 33, 13, 1);
    break;
  case 1:
    display.drawRect(46, 42, 33, 13, 1);
    break;
  case 2:
    display.drawRect(86, 43, 33, 13, 1);
    break;
  }

  display.setTextSize(1);
  display.setCursor(37, 6);
  switch (axis)
  {
  case 0:
    display.print("Pitch PID");
    break;
  case 1:
    display.print("Roll PID");
    break;
  case 2:
    display.print("Yaw PID");
    break;
  }

  display.display();
}

void PIDCalibration()
{
  uint8_t axis = 0;
  int value = 0;
  Serial.println("calibration");

  // Initial draw
  drawPID(axis, value, *pid_values[axis][0], *pid_values[axis][1], *pid_values[axis][2]);

  while (true)
  {
    pidAxisSelector.update();
    calibrationToggle.update();
    PidValueSelector.readStates();

    // Check encoder button press to change selected variable (P/I/D)
    if (PidValueSelector.readButton())
    {
      value++;
      if (value > 2)
      {
        value = 0;
      }
      drawPID(axis, value, *pid_values[axis][0], *pid_values[axis][1], *pid_values[axis][2]);
      PidValueSelector.resetStates();
      delay(50); // Debounce
      continue;
    }

    // Check rotation to modify the selected value
    int rotation = PidValueSelector.compare();
    if (rotation != 0)
    {
      *pid_values[axis][value] = max(0.0f, *pid_values[axis][value] + rotation * multipliers[axis][value]);
      setPidSelectorValues(axis, *pid_values[axis][0], *pid_values[axis][1], *pid_values[axis][2]);
      drawPID(axis, value, *pid_values[axis][0], *pid_values[axis][1], *pid_values[axis][2]);
      transmitData();
    }

    // Check axis selector button to change axis (Pitch/Roll/Yaw)
    if (pidAxisSelector.wasPressed())
    {
      axis++;
      if (axis > 2)
      {
        axis = 0;
      }
      drawPID(axis, value, *pid_values[axis][0], *pid_values[axis][1], *pid_values[axis][2]);
    }

    PidValueSelector.resetStates();
    delay(100);

    if (calibrationToggle.wasReleased())
    {
      return;
    }
  }
}

void readInputs()
{
  joystickL.readData();
  joystickR.readData();
  txData.leftX = joystickL.getX();
  txData.leftY = joystickL.getY();
  txData.leftButton = joystickL.wasPressed();

  txData.rightX = joystickR.getX();
  txData.rightY = joystickR.getY();
  txData.rightButton = joystickR.wasPressed();

  txData.throttle = analogRead(THROTTLE_PIN);
}

void setup()
{
  pinMode(CSN_PIN, OUTPUT);
  digitalWrite(CSN_PIN, HIGH);
  SPI.begin();
  Serial.begin(9600);

  delay(200);
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

  calibrationToggle.update();

  if (calibrationToggle.wasReleased())
  {
    PIDCalibration();
  }
  // Transmit at regular intervals
  if (now - lastTransmitTime >= TRANSMIT_INTERVAL)
  {
    lastTransmitTime = now;

    // Transmit data
    bool success = transmitData();
    updateCommStats(success, radio.isAckPayloadAvailable());

    // Update statistics

    // Print data periodically
    if (now - lastPrintTime >= 200)
    {
      lastPrintTime = now;
      //  printTransmittedData();
    }

    drawDisplay(txData.throttle, 1);
  }
}