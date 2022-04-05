#include "variables.h"
#define EXTERN extern
// #include "constants.h"

//
// arduino class variables
//

RTCZero rtc;
ModbusMaster modbus;

//
// variables for PUMP
//

volatile bool pumpOn;

volatile int pumpState = 0;
volatile int lastPumpState = 0;
volatile int pressure_status = 0;

volatile bool pressure_reset = false;
volatile bool timingMinPressure = false;
volatile bool timingMaxPressure = false;

volatile unsigned long pressureMinTimer;
volatile unsigned long pressureMaxTimer;
volatile unsigned long pressureTimerMax;
volatile unsigned long pressureTimerMin;

//
// variables for OXYGEN
//

volatile bool oxygenEnabled;
volatile bool oxygenOnDelay = false;
volatile bool oxygenAfterDelayOn = false;
volatile bool oxygenOnFirstTime = true;

volatile uint16_t oxygenDelay = 30; 
// volatile int oxygenDelay = 30; 

unsigned long startOxygenOnDelay; 
    
volatile int oxygenState;
volatile int lastOxygenState = 0;

//
// variables for OZONE
//

volatile int ozoneInterval = 10;  
volatile int ozoneState; 
volatile int prevOzoneState;
volatile int ozonatorState;
volatile int lastOzoneState = 0;
             
uint8_t ozoneDelay = 10;

volatile unsigned long previousOzoneMillis = 0;  
volatile unsigned long startOzoneOnDelay;

volatile bool ozoneEnabled = false;              
volatile bool ozoneOnDelay = false;
volatile bool ozoneAfterDelayOn = false;
volatile bool ozoneOnFirstTime = true;           

//
// variables for DO regulation
//

volatile bool reg_enable;
volatile bool potek_dol;
volatile bool potek_gor;
volatile bool rezim_dol1;
volatile bool casovnik;
volatile bool casovnik_off;
volatile bool Pumpa_ugas;
volatile bool prva_iteracija;

volatile double regulacija_max;
volatile double regulacija_min;

volatile int stevec_pumpa;
volatile int stevec_off;

// //
// // variables for ORP regulation
// //

// volatile bool reg_enable_ORP;
// volatile bool potek_dol_ORP;
// volatile bool potek_gor_ORP;
// volatile bool rezim_dol1_ORP;
// volatile bool casovnik_ORP;
// volatile bool casovnik_off_ORP;
// volatile bool Pumpa_ugas_ORP;
// volatile bool prva_iteracija_ORP;

// volatile double regulacija_max_ORP;
// volatile double regulacija_min_ORP;

// volatile int stevec_pumpa_ORP;
// volatile int stevec_off_ORP;

//
// variables for wifi connection
//

volatile bool startTimer = false;

volatile int ReCnctFlag;  
volatile int ReCnctCount = 0;  
volatile int stevec_wifi = 0;


//
// variables for timers
//

volatile unsigned long epochBlynk;   // Epoch of time from Blynk server
volatile unsigned int currentTime;   // Current calculated time from epoch value
volatile unsigned int t1StartTime;   // Timer 1 start time
volatile unsigned int t1StopTime;    // Timer 1 stop time
volatile unsigned int t2StartTime;   // Timer 1 start time
volatile unsigned int t2StopTime;    // Timer 1 stop time
volatile unsigned int t3StartTime;   // Timer 1 start time
volatile unsigned int t3StopTime;    // Timer 1 stop time
volatile unsigned int t4StartTime;   // Timer 1 start time
volatile unsigned int t4StopTime;    // Timer 1 stop time
volatile unsigned int t5StartTime;   // Timer 1 start time
volatile unsigned int t5StopTime;    // Timer 1 stop time

volatile int timer1En = 0;           // Timer 1 enabled
volatile int timer2En = 0;           // Timer 2 enabled
volatile int timer3En = 0;           // Timer 3 enabled
volatile int timer4En = 0;           // Timer 4 enabled
volatile int timer5En = 0;           // Timer 5 enabled

volatile int machOn = 0;             // Combined variable to turn machine ON by all timers
volatile int prevMachOn = 0;         // Previous machine state
volatile int mOnT1 = 0;              // Turn machine ON by timer 1
volatile int mOnT2 = 0;              // Turn machine ON by timer 2
volatile int mOnT3 = 0;              // Turn machine ON by timer 3
volatile int mOnT4 = 0;              // Turn machine ON by timer 4
volatile int mOnT5 = 0;              // Turn machine ON by timer 5

// //
// // variables for temperature sensors
// //
// uint8_t tempAdrPump[8] = { 0x28, 0x42, 0xAA, 0xC7, 0x0B, 0x00, 0x00, 0xF5 };  // address of pump temp sensor
// uint8_t tempAdrOzone[8] = { 0x28, 0x42, 0xAA, 0xC7, 0x0B, 0x00, 0x00, 0xF5 }; // address of ozone temp sensor
// uint8_t tempAdrIoT[8] = { 0x28, 0x42, 0xAA, 0xC7, 0x0B, 0x00, 0x00, 0xF5 };  // address of IoT temp sensor

// volatile float tempPump;   // Pump temp
// volatile float tempCompressor;  // Ozonator temp tempCompressor
// volatile float tempCabinet;    // IoT temp

//
// variables for safety functions
//

volatile unsigned long startTimeMaxPower;
volatile unsigned long currentTimeMaxPower;
volatile unsigned long periodMaxPower = 3;  // Max power until shutdown in minutes
volatile unsigned long startTimeMinPower;
volatile unsigned long currentTimeMinPower;
volatile unsigned long periodMinPower = 3;  // Max power until shutdown in minutes

volatile boolean timingMaxPower = false;
volatile boolean timingMinPower = false;

volatile unsigned int minPower = 950; // Set the minimum power allowed

volatile float acVoltage = 230.0;

volatile int machineMode;

//
// variables for machine operations
//

volatile int appMode = 1;          // Mode selected in app: 0-ALWAYS ON, 1-OFF, 2-AUTO
volatile int prevMode = 0;         // Last mode of AUTO/OFF/MAN Switch on the machine
volatile int autoCombination = 2;  // When selected AUTO: 0-pump+O3, 1-pump+O2, 2-pump+O3+O2
volatile int blynkConnected = 0;
int orpArray[ArrayLenth];
volatile int orpArrayIndex=0;

String statusSwitch;      // Status as shown in app (AUTO - IN OPERATION, AUTO - STAND BY etc.)

volatile unsigned long previousMillisDo = 0;
volatile unsigned long millisDoInterval = 5000;

volatile float offsetVoltage = 0;        //zero drift voltage

volatile double orpValue;                // ORP VALUE in mV (-2000mV to 2000mV)

//
// constant variables
//

volatile float maxTempPump = 80.0;
volatile float maxTempCompressor = 95.0;
volatile float maxtempCabinet = 80.0;

volatile unsigned int maxPower = 3500; // Set the maximum power allowed
volatile unsigned int powerLimitTemp = 66; // Temperature where we change min power becaue power of compressor goes down as it's temp raises

volatile int RSSI_MAX =-50;    // define maximum straighten of signal in dBm
volatile int RSSI_MIN =-100;   // define minimum strength of signal in dBm

//
// FLAG for operating
//

volatile int machineOperating = 1;
