#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

RF24 radio(9, 10); // CE, CSN

const byte address[6] = "NODE1";

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

static const unsigned char PROGMEM image_battery_10_bits[] = {0x00, 0x00, 0x00, 0x0f, 0xff, 0xfe, 0x10, 0x00, 0x01, 0x10, 0x00, 0x05, 0x70, 0x00, 0x05, 0x80, 0x00, 0x05, 0x80, 0x00, 0x05, 0x80, 0x00, 0x05, 0x80, 0x00, 0x05, 0x80, 0x00, 0x05, 0x70, 0x00, 0x05, 0x10, 0x00, 0x05, 0x10, 0x00, 0x01, 0x0f, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static const unsigned char PROGMEM image_download_bits[] = {0x3f, 0xe0, 0x00, 0x00, 0xa2, 0x28, 0x92, 0x48, 0x87, 0x08, 0x88, 0x88, 0xb8, 0xe8, 0x88, 0x88, 0x87, 0x08, 0x92, 0x48, 0xa2, 0x28, 0x80, 0x08, 0x40, 0x10, 0x3f, 0xe0};

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
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void drawDisplay(int batteryC, int batteryD, bool Light, int power)
{
  static int lastPowerPercent = -1; // last shown % on screen
  int powerPercent = map(power, 0, 1020, 0, 100);

  // Only update if the change is meaningful (≥2%)
  if (abs(powerPercent - lastPowerPercent) >= 2)
  {
    lastPowerPercent = powerPercent;

    display.clearDisplay();

    display.drawBitmap(18, 15, image_battery_10_bits, 24, 16, 1);
    display.drawBitmap(88, 14, image_battery_10_bits, 24, 16, 1);

    display.setTextColor(1);
    display.setTextWrap(false);
    display.setCursor(5, 4);
    display.print("Controller");

    display.setCursor(87, 4);
    display.print("Drone");

    display.drawLine(0, 32, 126, 32, 1);
    display.drawBitmap(24, 46, image_download_bits, 13, 14, 1);

    display.setCursor(14, 37);
    display.print("Lights");

    display.setCursor(87, 37);
    display.print("Power");

    display.setCursor(89, 51);
    display.print(powerPercent);
    display.print('%');

    display.display();
  }
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
  Serial.println(Data.pot1);
  drawDisplay(1,1,1, Data.pot1);
  delay(100);
}
