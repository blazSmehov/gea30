#ifndef VARIABLES_H
#define VARIABLES_H
#include <ModbusMaster.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <RTCZero.h>
#define EXTERN extern
#include "constants.h"

//
// class variables
//

EXTERN RTCZero rtc;
EXTERN ModbusMaster modbus;

//
// variables for PUMP
//

EXTERN volatile bool pumpOn;

EXTERN volatile int pumpState;
EXTERN volatile int lastPumpState;

//
// variables for OXYGEN
//

EXTERN volatile bool oxygenEnabled;
EXTERN volatile bool oxygenOnDelay;
EXTERN volatile bool oxygenAfterDelayOn;
EXTERN volatile bool oxygenOnFirstTime;

EXTERN volatile uint16_t oxygenDelay; 

EXTERN unsigned long startOxygenOnDelay; 
    
EXTERN volatile int oxygenState;
EXTERN volatile int lastOxygenState;

//
// variables for OZONE
//

EXTERN volatile int ozoneInterval;  
EXTERN volatile int ozoneState; 
EXTERN volatile int prevOzoneState;
EXTERN volatile int ozonatorState;
EXTERN volatile int lastOzoneState;
             
EXTERN uint8_t ozoneDelay;

EXTERN volatile unsigned long previousOzoneMillis;  
EXTERN volatile unsigned long startOzoneOnDelay;

EXTERN volatile bool ozoneEnabled;              
EXTERN volatile bool ozoneOnDelay;
EXTERN volatile bool ozoneAfterDelayOn;
EXTERN volatile bool ozoneOnFirstTime;   

//
// variables for DO regulation
//

EXTERN volatile bool reg_enable;
EXTERN volatile bool potek_dol;
EXTERN volatile bool potek_gor;
EXTERN volatile bool rezim_dol1;
EXTERN volatile bool potek_gor;
EXTERN volatile bool potek_dol;
EXTERN volatile bool casovnik;
EXTERN volatile bool casovnik_off;
EXTERN volatile bool Pumpa_ugas;
EXTERN volatile bool prva_iteracija;

EXTERN volatile double regulacija_max;
EXTERN volatile double regulacija_min;

EXTERN volatile int stevec_pumpa;
EXTERN volatile int stevec_off;

//
// variables for wifi connection
//

EXTERN volatile bool startTimer;

EXTERN char server[];   
EXTERN char ssid[];
EXTERN char pass[];

EXTERN volatile int ReCnctFlag;  
EXTERN volatile int ReCnctCount;  
EXTERN int port;
EXTERN volatile int stevec_wifi;

//EXTERN const int RSSI_MAX;    // define maximum straighten of signal in dBm
//EXTERN const int RSSI_MIN;   // define minimum strength of signal in dBm

//
// variables for timers
//

EXTERN volatile unsigned long epochBlynk;   // Epoch of time from Blynk server
EXTERN volatile unsigned int currentTime;   // Current calculated time from epoch value
EXTERN volatile unsigned int t1StartTime;   // Timer 1 start time
EXTERN volatile unsigned int t1StopTime;    // Timer 1 stop time
EXTERN volatile unsigned int t2StartTime;   // Timer 1 start time
EXTERN volatile unsigned int t2StopTime;    // Timer 1 stop time
EXTERN volatile unsigned int t3StartTime;   // Timer 1 start time
EXTERN volatile unsigned int t3StopTime;    // Timer 1 stop time
EXTERN volatile unsigned int t4StartTime;   // Timer 1 start time
EXTERN volatile unsigned int t4StopTime;    // Timer 1 stop time
EXTERN volatile unsigned int t5StartTime;   // Timer 1 start time
EXTERN volatile unsigned int t5StopTime;    // Timer 1 stop time

EXTERN volatile int timer1En;           // Timer 1 enabled
EXTERN volatile int timer2En;           // Timer 2 enabled
EXTERN volatile int timer3En;           // Timer 3 enabled
EXTERN volatile int timer4En;           // Timer 4 enabled
EXTERN volatile int timer5En;           // Timer 5 enabled

EXTERN volatile int machOn;             // Combined variable to turn machine ON by all timers
EXTERN volatile int prevMachOn;         // Previous machine state
EXTERN volatile int mOnT1;              // Turn machine ON by timer 1
EXTERN volatile int mOnT2;              // Turn machine ON by timer 2
EXTERN volatile int mOnT3;              // Turn machine ON by timer 3
EXTERN volatile int mOnT4;              // Turn machine ON by timer 4
EXTERN volatile int mOnT5;              // Turn machine ON by timer 5

//
// variables for temperature sensors
//

EXTERN uint8_t tempAdrPump[8];  // address of pump temp sensor
EXTERN uint8_t tempAdrOzone[8]; // address of ozone temp sensor
EXTERN uint8_t tempAdrIoT[8];   // address of IoT temp sensor

EXTERN volatile float tempPump;   // Pump temp
EXTERN volatile float tempCompressor;  // Ozonator temp tempCompressor
EXTERN volatile float tempCabinet;    // IoT temp

//
// variables for safety functions
//

EXTERN volatile unsigned long startTimeMaxPower;
EXTERN volatile unsigned long currentTimeMaxPower;
EXTERN volatile unsigned long periodMaxPower;  // Max power until shutdown in minutes
EXTERN volatile unsigned long startTimeMinPower;
EXTERN volatile unsigned long currentTimeMinPower;
EXTERN volatile unsigned long periodMinPower;  // Max power until shutdown in minutes

EXTERN volatile boolean timingMaxPower;
EXTERN volatile boolean timingMinPower;

EXTERN volatile unsigned int minPower; // Set the minimum power allowed

EXTERN volatile float acVoltage;

EXTERN volatile int machineMode;

//
// variables for machine operations
//

EXTERN volatile int appMode;          // Mode selected in app: 0-ALWAYS ON, 1-OFF, 2-AUTO
EXTERN volatile int prevMode;         // Last mode of AUTO/OFF/MAN Switch on the machine
EXTERN volatile int autoCombination;  // When selected AUTO: 0-pump+O3, 1-pump+O2, 2-pump+O3+O2
EXTERN volatile int blynkConnected;
EXTERN int orpArray[ArrayLenth];
EXTERN volatile int orpArrayIndex;

EXTERN String statusSwitch;      // Status as shown in app (AUTO - IN OPERATION, AUTO - STAND BY etc.)

EXTERN volatile unsigned long previousMillisDo;
EXTERN volatile unsigned long millisDoInterval;

EXTERN volatile float offsetVoltage;        //zero drift voltage

EXTERN volatile double orpValue;                // ORP VALUE in mV (-2000mV to 2000mV)

//
// constant variables
//

EXTERN volatile float maxTempPump;
EXTERN volatile float maxTempCompressor;
EXTERN volatile float maxtempCabinet;

EXTERN volatile unsigned int maxPower; // Set the maximum power allowed
EXTERN volatile unsigned int powerLimitTemp; // Temperature where we change min power becaue power of compressor goes down as it's temp raises

EXTERN volatile int RSSI_MAX;    // define maximum straighten of signal in dBm
EXTERN volatile int RSSI_MIN;   // define minimum strength of signal in dBm

#endif
