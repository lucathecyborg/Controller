#pragma once
#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Pin definitions for Arduino
// Standard Arduino SPI pins: SCK=52, MISO=50, MOSI=51
#define CE_PIN 8
#define CSN_PIN 9

extern RF24 radio;
extern const byte address[6];

// Message structure - MUST match drone side exactly
#pragma pack(push, 1)
struct message
{
    uint16_t leftX;
    uint16_t leftY;

    uint16_t rightX;
    uint16_t rightY;

    uint16_t throttle;

    uint8_t pidAxis;
    float kp, ki, kd;

    uint8_t flags;
    uint8_t checksum;
};
#pragma pack(pop)

extern message txData;
extern uint16_t rxBattery;
extern int lastBatteryUpdate;

struct CommStats
{
    uint32_t packetsSent;
    uint32_t acksReceived;
    uint32_t transmitFailures;
    unsigned long lastAckReceived;
    unsigned long maxGap;
};
extern CommStats commStats;

// Forward declarations
bool initRadio();
uint8_t calculateChecksum(message *msg);
void addChecksum(message *msg);
bool transmitData();
void updateCommStats(bool success, bool ackReceived);
void printCommStats();
void printTransmittedData();

#endif