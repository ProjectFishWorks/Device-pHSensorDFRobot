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

#define sendPhMessageFreq 30000 // frequency of sending LED intensity messages to the App
#define updatePHvalues 1000    // delay time in milliseconds to update the pH value

#define avgCount 100 // number of samples to average
#define timeBetweenPhSamples 500 // time between samples in milliseconds
unsigned long startAvg = millis(); // start time for averaging
float runningTotalPhReads = 0; // running total of pH reads
float sumOfAvgPhReads = 0; // sum of the average

#define NODE_ID 165                       // Node ID of the device
#define SEND_PH_MESSAGE_ID 0x0A00         // 2560
#define LOW_PH_ALARM_MESSAGE_ID 0x0A01    // 2561
#define HIGH_PH_ALARM_MESSAGE_ID 0x0A02   // 2562
#define PH_ALARM_ON_OFF_MESSAGE_ID 0x0A03 // 2563
#define GET_TEMP_MESSAGE_ID 2572        // 2564

#define ALARM_MESSAGE_ID 901 // ID in hex is 0x385

#define PH_PIN 4 // pH sensor analog pin
bool pHAlarmOnOff;
uint8_t hasSentPHAlarm = 0;
uint8_t hasSentNoPHAlarm = 0;

float lowPHalarmValue = 0;
float highPHalarmValue = 0;
uint64_t errorData1 = 1;
uint64_t errorData0 = 0;

float voltage;
float temperature = 20;     // temperature value updated from the CAN bus

// PH sensor object
DFRobot_PH DFRobot_PH_sensor;
// Node controller core object
NodeControllerCore core;

//--------------------------------------------- put function declarations here:----------------------------------------------------

// Callback function for received messages from the CAN bus
void receive_message(uint8_t nodeID, uint16_t messageID, uint64_t data);

// Function to update the pH value
void avgPhValue(void *parameters);

// Function to send the pH value to the CAN bus
void sendPHMessage(void *parameters);

//-------------------------------------------------------Setup-------------------------------------------------------------------------

void setup()
{
    Serial.begin(115200);
    DFRobot_PH_sensor.begin();
    DFRobot_PH_sensor.calibration(voltage,temperature);           // calibration process by Serail CMD

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
    xTaskCreate(sendPHMessage, "sendPHMessage", 10000, NULL, 1, NULL);
    xTaskCreate(avgPhValue, "avgPhValue", 10000, NULL, 1, NULL);
}

//-------------------------------------------------------Loop-------------------------------------------------------------------------

void loop()
{
    //updatePH();

    //sendPHMessage();

    DFRobot_PH_sensor.calibration(voltage,temperature);           // calibration process by Serail CMD

    delay(10);
}

////////////////////////////////////////////// put function definitions here:  /////////////////////////////////////////////////////////

void avgPhValue(void *parameters)
{
    while(1){
       // temperature = readTemperature();         // read your temperature sensor to execute temperature compensation
        // voltage = analogRead(PH_PIN)/1024.0*5000;  // read the voltage
        voltage = (analogReadMilliVolts(PH_PIN)); // read the voltage
        //phValue = random(0, 14);                       // random pH value for testing
        float phValue = DFRobot_PH_sensor.readPH(voltage, temperature); // convert voltage to pH with temperature compensation
        sumOfAvgPhReads += phValue;
        runningTotalPhReads++;

        #ifdef debuging
        
            Serial.println("voltage = " + String(voltage));
            Serial.print("temperature:");
            Serial.print(temperature, 1);
            Serial.print("^C  pH:");
            Serial.println(phValue, 2);
        #endif
        delay(timeBetweenPhSamples);
    }
 
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
            memccpy(&lowPHalarmValue, &data, sizeof(data), sizeof(lowPHalarmValue));
            Serial.println("Low pH alarm value is " + String(lowPHalarmValue));
            hasSentPHAlarm = 0;
            hasSentNoPHAlarm = 0;
            break;

        case HIGH_PH_ALARM_MESSAGE_ID:
            memccpy(&highPHalarmValue, &data, sizeof(data), sizeof(highPHalarmValue));
            Serial.println("High pH alarm value is " + String(highPHalarmValue));
            hasSentPHAlarm = 0;
            hasSentNoPHAlarm = 0;
            break;

        default:
            break;
        }
    }else if(nodeID == 162 && messageID == GET_TEMP_MESSAGE_ID){
        Serial.println("Temperature raw value is " + String(data));
        memccpy(&temperature, &data, sizeof(data), sizeof(temperature));
        Serial.println("Temperature float value is " + String(temperature));
    }
}

void sendPHMessage(void *parameters)
{
    while (1)
    {
        // Send the pH value to the CAN bus
    float phValue = sumOfAvgPhReads / runningTotalPhReads;

    Serial.println("low pH alarm value is " + String(lowPHalarmValue));
    Serial.println("high pH alarm value is " + String(highPHalarmValue));

    Serial.println("pH average value is " + String(phValue));   

    core.sendMessage(SEND_PH_MESSAGE_ID, &phValue);

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
            if (hasSentPHAlarm == 0)
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
    sumOfAvgPhReads = 0;
    runningTotalPhReads = 0;
    delay(sendPhMessageFreq);
    }
    
}

/*
float readTemperature()
{
  //add your code here to get the temperature from your temperature sensor
}
*/