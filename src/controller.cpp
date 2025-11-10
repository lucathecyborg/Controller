#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#include "RotEncoder.h"
#include "bitmaps.h"
#include "joystick.h"
#include "button.h"

//Size 27 bytes (max 32)
struct message
{
  uint16_t pot1; //2 bytes
  joystickValues joystickL; // 6 bytes
  joystickValues joystickR ; // 6 bytes
  uint8_t PidAxis;    // 1 byte
  float kp=1.5, ki=0.05, kd=0.8; // 12 bytes
};



  float roll_kp = 1.5, roll_ki = 0.05, roll_kd = 0.8;  
  float pitch_kp = 1.5, pitch_ki = 0.05, pitch_kd = 0.8;
  float yaw_kp = 1.5, yaw_ki = 0.05, yaw_kd = 0.8; 


const byte address[6] = "NODE1";
RF24 radio(9, 10); // CE, CSN

joystick joystickL(A1,A2,22);
joystick joystickR(A3,A4,23);
Button calibrationButton(24,500,true);
Button pidSelector(25,50,true);
RotaryEncoder PidValue(26,27,28);


message Data;


uint16_t DroneBattery;
uint16_t ControllerBattery = 100; 
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64



Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


bool CalibratePID = false;


// Display useful information during normal operation
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
  radio.enableAckPayload();

  display.begin(0x3C, true);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  

  pinMode(A0, INPUT);
  
}


//Display PID values during calibration
void drawPID(int axis, int variable, float kp, float ki, float kd){

  display.clearDisplay();
  display.setTextColor(1);
  display.setTextSize(2);
  display.setTextWrap(false);
  display.setCursor(17,24);
  display.print("P");

  display.setCursor(58,23);
  display.print("I");

  display.setCursor(99,25);
  display.print("D");

  display.setTextSize(1);

  display.setCursor(10,44);
  display.print(kp);

  display.setCursor(51,45);
  display.print(ki);

  display.setCursor(92,46);
  display.print(kd);

  switch (variable){
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
  display.setCursor(37,6);
  switch (axis){
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


//Send data to the drone
void sendData(){

 /* Data.joystickL = joystickL.getValues();
  Data.joystickR = joystickR.getValues();  */
    Data.joystickL.x = 512;
  Data.joystickL.y = 512;
  Data.joystickL.button = 0;

  Data.joystickR.x = 512;
  Data.joystickR.y = 512;
  Data.joystickR.button = 0;
  Data.pot1 = analogRead(A0);
  Serial.println(Data.pot1);
  /*Serial.print("X:  ");
  Serial.print(Data.joystickL.x);
  Serial.print("    Y:  ");
  Serial.println(Data.joystickL.y);

  */

  radio.write(&Data, sizeof(Data));
  if (radio.isAckPayloadAvailable())
  {
    radio.read(&DroneBattery, sizeof(DroneBattery));
  }

  
}



// Update the PID values in the Data struct
void setPidSelectorValues(uint8_t axis, float p, float i, float d){
      Data.kd=d;
      Data.kp=p;
      Data.ki=i;
      Data.PidAxis=axis; 
 
}



// PID Calibration function
void Calibration(){
  int axis = 0;
  int value = 0;

  while(true){
    
    pidSelector.update(); 

    // Axis selector
    if(pidSelector.wasPressed()){
      axis++;
      if(axis > 2){
        axis = 0;
      }
      Serial.println("pid Selector was pressed");
    }

    PidValue.update();

    // Variable selector
    if(PidValue.buttonWasPressed()){
      value++;
      if(value > 2){
        value = 0;
      }
      Serial.println("pid value was pressed");
    }

    int delta = PidValue.getDelta();
Serial.print("Button pin reading: ");
Serial.println(digitalRead(28));

    switch (value){
      case 0: // P
        switch (axis)
        {
        case 0: // Pitch
          pitch_kp += delta * 0.5;
          drawPID(0, 0, pitch_kp, pitch_ki, pitch_kd);
          setPidSelectorValues(0, pitch_kp, pitch_ki, pitch_kd);
          break;
        
        case 1: // Roll
          roll_kp += delta * 0.5;
          drawPID(1, 0, roll_kp, roll_ki, roll_kd);
          setPidSelectorValues(1, roll_kp, roll_ki, roll_kd);
          break;

        case 2: // Yaw
          yaw_kp += delta * 0.5;
          drawPID(2, 0, yaw_kp, yaw_ki, yaw_kd);
          setPidSelectorValues(2, yaw_kp, yaw_ki, yaw_kd);
          break;
        }
        break;

      case 1: // I
        switch (axis)
        {
        case 0: // Pitch
          pitch_ki += delta * 0.05;
          drawPID(0, 1, pitch_kp, pitch_ki, pitch_kd);
          setPidSelectorValues(0, pitch_kp, pitch_ki, pitch_kd);
          break;
        
        case 1: // Roll
          roll_ki += delta * 0.05;
          drawPID(1, 1, roll_kp, roll_ki, roll_kd);
          setPidSelectorValues(1, roll_kp, roll_ki, roll_kd);
          break;

        case 2: // Yaw
          yaw_ki += delta * 0.05;
          drawPID(2, 1, yaw_kp, yaw_ki, yaw_kd);
          setPidSelectorValues(2, yaw_kp, yaw_ki, yaw_kd);
          break;
        }
        break;

      case 2: // D
        switch (axis)
        {
        case 0: // Pitch
          pitch_kd += delta * 0.2;
          drawPID(0, 2, pitch_kp, pitch_ki, pitch_kd);
          setPidSelectorValues(0, pitch_kp, pitch_ki, pitch_kd);
          break;
        
        case 1: // Roll
          roll_kd += delta * 0.2;
          drawPID(1, 2, roll_kp, roll_ki, roll_kd);
          setPidSelectorValues(1, roll_kp, roll_ki, roll_kd);
          break;

        case 2: // Yaw
          yaw_kd += delta * 0.02;
          drawPID(2, 2, yaw_kp, yaw_ki, yaw_kd);
          setPidSelectorValues(2, yaw_kp, yaw_ki, yaw_kd);
          break;
        }
        break;
    }
 
    calibrationButton.update();
    if(calibrationButton.wasPressed()){
      CalibratePID = false;
      return;
    }
    
    sendData();
    delay(100);
  }
}







void loop()
{
  calibrationButton.update();
  if (calibrationButton.wasPressed()) {
    CalibratePID = !CalibratePID;
    Serial.println("entering calibration");
  }

  if(CalibratePID){
   Calibration();
  } 


 
  sendData();
  drawDisplay(1, Data.pot1);
  delay(100);
}