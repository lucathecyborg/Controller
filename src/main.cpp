#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <RotaryEncoder.h>

#include "communication.h"
#include "joystick.h"
#include "button.h"
#include "toggle_switch.h"
#include "bitmaps.h"
#include "buzzer.h"

#define THROTTLE_PIN A8
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// flags
#define FLAG_ARMED (1 << 0)          // bit 0: motors armed
#define FLAG_ALT_HOLD (1 << 1)       // bit 1: altitude hold enabled
#define FLAG_RETURN_TO_HOME (1 << 2) // enable return to home
#define FLAG_SAFE_LANDING (1 << 3)   // enable safe landing
#define FLAG_SET_HOME (1 << 4)       // set GPS home location
#define FLAG_FREEZE (1 << 5)         // freeze input from controller

#define LED_GREEN 27
#define LED_YELLOW 28
#define LED_RED 29

// Timing
unsigned long lastTransmitTime = 0;
const unsigned long TRANSMIT_INTERVAL = 100;

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

joystick joystickL(A9, A10, 22);
joystick joystickR(A11, A12, 23);
toggleSwitch flagSwitch(30, 31);
toggleSwitch altitudeHold(32, 33);

// Encoder setup - same as working example
RotaryEncoder pidEncoder(24, 25, RotaryEncoder::LatchMode::FOUR3);
Button pidEncoderButton(26);
Button pidAxisSelector(34);
Button calibrationToggle(35);

uint8_t ControllerBattery;

float roll_kp = 0.8, roll_ki = 0.02, roll_kd = 0.4;
float pitch_kp = 0.8, pitch_ki = 0.02, pitch_kd = 0.4;
float yaw_kp = 1.5, yaw_ki = 0.01, yaw_kd = 0.05;

float *pid_values[3][3] = {
    {&pitch_kp, &pitch_ki, &pitch_kd},
    {&roll_kp, &roll_ki, &roll_kd},
    {&yaw_kp, &yaw_ki, &yaw_kd}};

float multipliers[3][3] = {
    {0.5, 0.005, 0.05},
    {0.5, 0.005, 0.05},
    {0.5, 0.002, 0.01}};

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
  static int lastPowerPercent = -100;
  int powerPercent = map(power, 0, 1023, 0, 100);

  int DroneBatteryIndex = map(rxBattery, 0, 100, 0, 6);
  DroneBatteryIndex = constrain(DroneBatteryIndex, 0, 6);

  int ControllerBatteryIndex = map(ControllerBattery, 0, 100, 0, 6);
  ControllerBatteryIndex = constrain(ControllerBatteryIndex, 0, 6);

  display.clearDisplay();

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
  unsigned long lastPidTransmitTime = 0;

  // Track encoder position - same pattern as working example
  static int pos = 0;

  Serial.println("=== PID Calibration Started ===");

  // Reset encoder position
  pidEncoder.setPosition(0);
  pos = 0;

  // Initial draw
  drawPID(axis, value, *pid_values[axis][0], *pid_values[axis][1], *pid_values[axis][2]);

  while (true)
  {
    unsigned long now = millis();

    // CRITICAL: tick() must be called as frequently as possible
    pidEncoder.tick();

    pidAxisSelector.update();
    calibrationToggle.update();
    pidEncoderButton.update();

    // Check encoder button press to change selected variable (P/I/D)
    if (pidEncoderButton.wasPressed())
    {
      value++;
      if (value > 2)
      {
        value = 0;
      }
      Serial.print("Button pressed - selecting: ");
      Serial.println(value == 0 ? "P" : (value == 1 ? "I" : "D"));
      drawPID(axis, value, *pid_values[axis][0], *pid_values[axis][1], *pid_values[axis][2]);
      continue;
    }

    // Check rotation - exact same pattern as working example
    int newPos = pidEncoder.getPosition();
    if (pos != newPos)
    {
      int direction = (int)(pidEncoder.getDirection());
      int delta = newPos - pos;

      // Debug output
      Serial.print("pos:");
      Serial.print(newPos);
      Serial.print(" dir:");
      Serial.print(direction);
      Serial.print(" delta:");
      Serial.println(delta);

      // Apply the change
      *pid_values[axis][value] = max(0.0f, *pid_values[axis][value] + delta * multipliers[axis][value]);
      setPidSelectorValues(axis, *pid_values[axis][0], *pid_values[axis][1], *pid_values[axis][2]);
      drawPID(axis, value, *pid_values[axis][0], *pid_values[axis][1], *pid_values[axis][2]);

      pos = newPos;
    }

    // Check axis selector button to change axis (Pitch/Roll/Yaw)
    if (pidAxisSelector.wasPressed())
    {
      axis++;
      if (axis > 2)
      {
        axis = 0;
      }
      Serial.print("Axis changed to: ");
      Serial.println(axis == 0 ? "Pitch" : (axis == 1 ? "Roll" : "Yaw"));
      drawPID(axis, value, *pid_values[axis][0], *pid_values[axis][1], *pid_values[axis][2]);
    }

    if (now - lastPidTransmitTime >= 100)
    {
      lastPidTransmitTime = now;
      transmitData();
    }

    if (calibrationToggle.wasReleased())
    {
      Serial.println("=== PID Calibration Ended ===");
      return;
    }
  }
}

// Track previous armed state for melody logic
bool prevArmed = false;
void setLED(int led)
{
  switch (led)
  {
  case 0:
    digitalWrite(LED_RED, 1);
    digitalWrite(LED_YELLOW, 0);
    digitalWrite(LED_GREEN, 0);
    break;
  case 1:
    digitalWrite(LED_RED, 0);
    digitalWrite(LED_YELLOW, 1);
    digitalWrite(LED_GREEN, 0);
    break;
  case 2:
    digitalWrite(LED_RED, 0);
    digitalWrite(LED_YELLOW, 0);
    digitalWrite(LED_GREEN, 1);
    break;
  default:
    digitalWrite(LED_RED, 0);
    digitalWrite(LED_YELLOW, 0);
    digitalWrite(LED_GREEN, 0);
    break;
  }
}
void readInputs()
{
  joystickL.readData();
  joystickR.readData();
  txData.leftX = joystickL.getX();
  txData.leftY = joystickL.getY();

  txData.rightX = joystickR.getX();
  txData.rightY = joystickR.getY();

  txData.throttle = analogRead(THROTTLE_PIN);

  txData.flags = 0;
  switch (flagSwitch.readOutput())
  {
  case 1:
    txData.flags |= FLAG_ARMED;

    break;
  case 2:
    txData.flags |= FLAG_FREEZE;
    break;
  }

  switch (altitudeHold.readOutput())
  {
  case 1:
    txData.flags |= FLAG_ALT_HOLD;
    break;
  case 2:
    txData.flags |= FLAG_RETURN_TO_HOME;
    break;
  }

  // Melody logic for armed/disarmed
  bool nowArmed = (txData.flags & FLAG_ARMED);
  if (nowArmed && !prevArmed)
  {
    playMelody(MELODY_ARMED, DURATION_ARMED_DISARMED, 2);
    setLED(2);
  }
  else if (!nowArmed && prevArmed)
  {
    playMelody(MELODY_DISARMED, DURATION_ARMED_DISARMED, 2);
    setLED(1);
  }
  prevArmed = nowArmed;
}

void setup()
{
  pinMode(CSN_PIN, OUTPUT);
  digitalWrite(CSN_PIN, HIGH);
  SPI.begin();
  Serial.begin(9600);
  while (!Serial)
    ; // Wait for serial - same as working example

  delay(200);
  Serial.println("Controller starting...");
  initDisplay();

  delay(1000);
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  playMelody(MELODY_BOOT, DURATION_BOOT, 3);

  if (!initRadio())
  {
    display.clearDisplay();
    display.drawBitmap(2, 9, radio_failed_screen, 123, 46, 1);
    setLED(0);
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

  // Always tick encoder in main loop
  pidEncoder.tick();

  calibrationToggle.update();

  if (calibrationToggle.wasReleased())
  {
    PIDCalibration();
  }

  if (now - lastTransmitTime >= TRANSMIT_INTERVAL)
  {
    lastTransmitTime = now;

    bool success = transmitData();
    updateCommStats(success, radio.isAckPayloadAvailable());

    if (now - lastPrintTime >= 200)
    {
      lastPrintTime = now;
    }
    updateBuzzer();
    drawDisplay(txData.throttle, 1);
  }
}
