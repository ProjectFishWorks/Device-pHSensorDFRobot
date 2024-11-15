/*!
 * @file DFRobot_PH_Test.h
 * @brief This is the sample code for Gravity: Analog pH Sensor / Meter Kit V2, SKU:SEN0161-V2.
 * @n In order to guarantee precision, a temperature sensor such as DS18B20 is needed, to execute automatic temperature compensation.
 * @n You can send commands in the serial monitor to execute the calibration.
 * @n Serial Commands:
 * @n    enterph -> enter the calibration mode
 * @n    calph   -> calibrate with the standard buffer solution, two buffer solutions(4.0 and 7.0) will be automaticlly recognized
 * @n    exitph  -> save the calibrated parameters and exit from calibration mode
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [Jiawei Zhang](jiawei.zhang@dfrobot.com)
 * @version  V1.0
 * @date  2018-11-06
 * @url https://github.com/DFRobot/DFRobot_PH
 */

#include "DFRobot_PH.h"
#include <EEPROM.h>
#include <NodeControllerCore.h>
#include <Arduino.h>

#define debuging

#define sendPhMessageFreq 4000 // frequency of sending LED intensity messages to the App
#define updatePHvalues 1000    // delay time in milliseconds to update the pH value

#define NODE_ID 165                       // Node ID of the device
#define SEND_PH_MESSAGE_ID 0x0A00         // 2560
#define LOW_PH_ALARM_MESSAGE_ID 0x0A01    // 2561
#define HIGH_PH_ALARM_MESSAGE_ID 0x0A02   // 2562
#define PH_ALARM_ON_OFF_MESSAGE_ID 0x0A03 // 2563
#define GET_TEMP_MESSAGE_ID 0x0A04        // 2564

#define ALARM_MESSAGE_ID 901 // ID in hex is 0x385

#define PH_PIN 4 // pH sensor analog pin
bool pHAlarmOnOff;
uint8_t hasSentPHAlarm = 0;
uint8_t hasSentNoPHAlarm = 0;

uint64_t lowPHalarmValue = 0;
uint64_t highPHalarmValue = 0;
uint64_t errorData1 = 1;
uint64_t errorData0 = 0;

float voltage;
float phValue;
float temperature = 20;

// PH sensor object
DFRobot_PH DFRobot_PH_sensor;
// Node controller core object
NodeControllerCore core;

//--------------------------------------------- put function declarations here:----------------------------------------------------

// Callback function for received messages from the CAN bus
void receive_message(uint8_t nodeID, uint16_t messageID, uint64_t data);

// Function to update the pH value
float updatePH();

// Function to send the pH value to the CAN bus
void sendPHMessage();

//-------------------------------------------------------Setup-------------------------------------------------------------------------

void setup()
{
    Serial.begin(115200);
    DFRobot_PH_sensor.begin();

    // Create the node controller core object

    core = NodeControllerCore();

    Serial.println("test");
    // Initialize the node controller core object
    if (core.Init(receive_message, NODE_ID))
    {
        Serial.println("Driver device initialized");
    }
    else
    {
        Serial.println("Failed to initialize driver");
    }
}

//-------------------------------------------------------Loop-------------------------------------------------------------------------

void loop()
{
    updatePH();

    sendPHMessage();
}

////////////////////////////////////////////// put function definitions here:  /////////////////////////////////////////////////////////

float updatePH()
{
    static unsigned long timepoint = millis();
    if (millis() - timepoint > 1000U)
    { // time interval: 1s
        timepoint = millis();
        // temperature = readTemperature();         // read your temperature sensor to execute temperature compensation
        // voltage = analogRead(PH_PIN)/1024.0*5000;  // read the voltage
        voltage = (analogReadMilliVolts(PH_PIN)/1000.0); // read the voltage
        //phValue = random(0, 14);                       // random pH value for testing
        phValue = DFRobot_PH_sensor.readPH(voltage, temperature); // convert voltage to pH with temperature compensation

        // DFRobot_PH_sensor.calibration(voltage,temperature);           // calibration process by Serail CMD

#ifdef debuging
        
        Serial.println("voltage = " + String(voltage));
        Serial.print("temperature:");
        Serial.print(temperature, 1);
        Serial.print("^C  pH:");
        Serial.println(phValue, 2);
#endif
    }
    delay(updatePHvalues);
    return phValue;
}

// Callback function for received messages from the CAN bus
void receive_message(uint8_t nodeID, uint16_t messageID, uint64_t data)
{
    Serial.println("Message received callback");

    // Check if the message is for this node
    if (nodeID == NODE_ID)
    {
        Serial.println("Message received to self");
        // Check the message ID for the LED control messages
        switch (messageID)
        {
        case PH_ALARM_ON_OFF_MESSAGE_ID:
            pHAlarmOnOff = data;
            Serial.println("pH alarm on off value is " + String(data));
            break;
        case LOW_PH_ALARM_MESSAGE_ID:
            lowPHalarmValue = data;
            Serial.println("Low pH alarm value is " + String(data));
            hasSentPHAlarm = 0;
            hasSentNoPHAlarm = 0;
            break;

        case HIGH_PH_ALARM_MESSAGE_ID:
            highPHalarmValue = data;
            Serial.println("High pH alarm value is " + String(data));
            hasSentPHAlarm = 0;
            hasSentNoPHAlarm = 0;
            break;

        case GET_TEMP_MESSAGE_ID:
            temperature = data;
            Serial.println("Temperature value is " + String(data));
            break;

        default:
            break;
        }
    }
}

void sendPHMessage()
{
    // Send the pH value to the CAN bus
    uint64_t PHvalue = updatePH();
    core.sendMessage(SEND_PH_MESSAGE_ID, &PHvalue);

    if (pHAlarmOnOff == 1)
    {
#ifdef debuging
        Serial.println("pH AlarmOnOff= " + String(pHAlarmOnOff));
#endif
        if (phValue >= highPHalarmValue || phValue <= lowPHalarmValue)
        {
#ifdef debuging
            Serial.println("------------pH Message triggered------------");
            Serial.println("Has sent alarm is = " + String(hasSentPHAlarm));
#endif
            if (hasSentPHAlarm == 1)
            {
                core.sendMessage(ALARM_MESSAGE_ID, &errorData1);
                hasSentPHAlarm = 1;
                hasSentNoPHAlarm = 0;
                Serial.println("-----------Not Can Temp Alarm  = " + String(hasSentPHAlarm));
            }
        }
        else
        {
            if (hasSentNoPHAlarm == 0)
            {
                core.sendMessage(ALARM_MESSAGE_ID, &errorData0);
                hasSentPHAlarm = 0;
                hasSentNoPHAlarm = 1;
#ifdef debuging
                Serial.println("Canopy Temperture no alarm");
                Serial.println("Canopy Temperture = " + String(phValue));
#endif
            }
        }
    }
    delay(sendPhMessageFreq);
}

/*
float readTemperature()
{
  //add your code here to get the temperature from your temperature sensor
}
*/