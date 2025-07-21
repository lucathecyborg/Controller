#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Arduino.h>

RF24 radio(9, 10); // CE, CSN

const byte address[6] = "NODE1";

struct joystick
{
  uint16_t x;
  uint16_t y;
  bool button;
};

struct message
{
  uint16_t pot1;
  joystick joystickL;
  joystick joystickR;
  bool button1;
};

message Data;

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
}

void loop()
{

  Data.joystickL.x = analogRead(A1);
  Data.joystickL.y = analogRead(A2);
  Data.joystickL.button = 0;

  Data.joystickR.x = 0;
  Data.joystickR.y = 0;
  Data.joystickR.button = 0;
  Serial.print("X:  ");
  Serial.print(Data.joystickL.x);
  Serial.print("    Y:  ");
  Serial.println(Data.joystickL.y);

  Data.pot1 = analogRead(A0);
  Data.button1 = digitalRead(7);
  bool ok = radio.write(&Data, sizeof(Data));
  // Serial.print("Sending data: ");
  if (ok)
  {
    /*Serial.print("Sent: ");
    Serial.println(Data.pot1); */
  }
  else
  {
    Serial.println("Failed to send");
  }
  delay(100);
}
