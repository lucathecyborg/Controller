#include "Communication.h"

// Define global variables declared in Communication.h
RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "NODE1";
message txData;
uint16_t rxBattery = 0;
int lastBatteryUpdate = 0;
CommStats commStats = {0, 0, 0, 0, 0};
unsigned long noAckSince = 0;
bool initRadio()
{
    if (!radio.begin())
    {
        Serial.println("Radio hardware not responding!");
        return false;
    }
    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(125); // Avoid WiFi interference
    radio.enableAckPayload();
    radio.openWritingPipe(address);
    radio.stopListening(); // Controller is transmitter

    Serial.println("Radio initialized successfully");
    return true;
}

uint8_t calculateChecksum(message *msg)
{
    uint8_t sum = 0;
    const uint8_t *data = (const uint8_t *)msg;

    // Calculate checksum over all bytes except the checksum field itself
    for (size_t i = 0; i < sizeof(message) - 1; i++)
    {
        sum += data[i];
    }
    return sum;
}

void addChecksum(message *msg)
{
    msg->checksum = calculateChecksum(msg);
}

void readInputs();
void setLED(int led);

bool transmitData()
{

    readInputs();
    // Add checksum before transmitting
    addChecksum(&txData);

    // Attempt to send the data
    bool txSuccess = radio.write(&txData, sizeof(txData));

    if (txSuccess)
    {
        if (radio.isAckPayloadAvailable())
        {
            radio.read(&rxBattery, sizeof(rxBattery));
            noAckSince = 0; // reset timer
        }
        else
        {
            Serial.println("ACK payload not available");
        }
    }
    else
    {
        Serial.println("Transmission failed");
    }

    return txSuccess;
}

void updateCommStats(bool success, bool ackReceived)
{
    if (success)
    {
        commStats.packetsSent++;

        if (ackReceived)
        {
            unsigned long now = millis();

            if (commStats.acksReceived > 0)
            {
                unsigned long gap = now - commStats.lastAckReceived;
                if (gap > commStats.maxGap)
                {
                    commStats.maxGap = gap;
                }
            }

            commStats.acksReceived++;
            commStats.lastAckReceived = now;
        }
    }
    else
    {
        commStats.transmitFailures++;
    }
}

void printCommStats()
{
    unsigned long timeSinceLastAck = millis() - commStats.lastAckReceived;

    Serial.println("\n--- Communication Statistics ---");
    Serial.print("Packets Sent: ");
    Serial.println(commStats.packetsSent);
    Serial.print("ACKs Received: ");
    Serial.println(commStats.acksReceived);
    Serial.print("Transmit Failures: ");
    Serial.println(commStats.transmitFailures);

    if (commStats.packetsSent > 0)
    {
        float successRate = (commStats.acksReceived * 100.0) / commStats.packetsSent;
        Serial.print("Success Rate: ");
        Serial.print(successRate, 1);
        Serial.println("%");
    }

    Serial.print("Max Gap: ");
    Serial.print(commStats.maxGap);
    Serial.println("ms");

    Serial.print("Last ACK: ");
    Serial.print(timeSinceLastAck);
    Serial.println("ms ago");

    if (commStats.acksReceived > 0)
    {
        Serial.println("Status: CONNECTED");
    }
    else
    {
        Serial.println("Status: NO RESPONSE");
    }

    Serial.print("Drone Battery: ");
    Serial.print(rxBattery);
    Serial.println("%");

    Serial.println("--------------------------------\n");
}

void printTransmittedData()
{
    Serial.print("TX > ");
    Serial.print("throttle:");
    Serial.print(txData.throttle);
    Serial.print(" L:");
    Serial.print(txData.leftX);
    Serial.print(",");
    Serial.print(txData.leftY);
    Serial.print(" R:");
    Serial.print(txData.rightX);
    Serial.print(",");
    Serial.print(txData.rightY);

    if (txData.pidAxis < 3)
    {
        Serial.print(" PID[");
        Serial.print(txData.pidAxis);
        Serial.print("]: P=");
        Serial.print(txData.kp, 3);
        Serial.print(" I=");
        Serial.print(txData.ki, 3);
        Serial.print(" D=");
        Serial.print(txData.kd, 3);
    }

    Serial.print(" Batt:");
    Serial.print(rxBattery);
    Serial.println("%");
}