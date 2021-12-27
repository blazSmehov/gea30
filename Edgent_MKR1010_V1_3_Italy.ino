#include <OneWire.h>
#include <DallasTemperature.h>
#include <RTCZero.h>
// Fill-in information from your Blynk Template here
#define BLYNK_TEMPLATE_ID "TMPLqzMT0mMJ"
#define BLYNK_DEVICE_NAME "SmartWater"

#define BLYNK_FIRMWARE_VERSION        "0.1.10"

#define BLYNK_PRINT Serial
//#define BLYNK_DEBUG

#define APP_DEBUG

#include "BlynkEdgent.h"

/* HARDWARE DEFINES - START */
#define ONE_WIRE_BUS 0

#define AUTO 18
#define MAN 19
#define FLOAT_SWITCH 20
#define BOARD_BUTTON 6

#define AC_PIN A1
#define ACTectionRange 20;  //set Non-invasive AC Current Sensor tection range (5A,10A,20A)
#define VREF 3.3

#define PUMP_CONTACTOR 3
#define OXYGEN_CONTACTOR 2
#define OZONE_CONTACTOR 1
#define RESERVE_CONTACTOR 4

#define ONE_WIRE_BUS 0 // Data wire is plugged into digital pin 0 on the Arduino
/* HARDWARE DEFINES - END */
// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire(ONE_WIRE_BUS);  
// Pass oneWire reference to DallasTemperature library
DallasTemperature sensors(&oneWire);

int machineMode;
String statusSwitch;
int prevMode = 0;         // Last mode of AUTO/OFF/MAN Switch on the machine
int appMode = 1;          // Mode selected in app: 0-ALWAYS ON, 1-OFF, 2-AUTO
int autoCombination = 2;  // When selected AUTO: 0-pump+O3, 1-pump+O2, 2-pump+O3+O2


bool pumpOn = false;
float acVoltage = 230.0;

// Ozone variables
int ozoneInterval = 10;                 // Interval of ozonator ON/OFF in minutes
uint8_t ozoneDelay = 10;
unsigned long previousOzoneMillis = 0;  // Previous ozone millis
bool ozoneEnabled = false;              // Ozone generator is enabled
int ozoneState;                         // Ozone generator in cycle is off/on - used in function ozonatorCycle
int prevOzoneState;                     // Last state in ozone cycle
bool ozoneOnDelay = false;
bool ozoneAfterDelayOn = false;
unsigned long startOzoneOnDelay;
bool ozoneOnFirstTime = true;           // Ozone ON for the firs time

// Oxygen variables
uint16_t oxygenDelay = 30;        // The delay for oxygen generator to turn ON
bool oxygenEnabled = false;       // Ozone generator is enabled
bool oxygenOnDelay = false;       // Start of delay
bool oxygenAfterDelayOn = false;  // Delay has finished - oxygen on
unsigned long startOxygenOnDelay; // Millis of current time
bool oxygenOnFirstTime = true;    // Oxygen ON for the first time
int ozonatorState;                // Ozone generator is off/on
int pumpState = 0;                // Pump is off/on
int oxygenState;                  // Oxygen generator is off/on

// Previous States
int lastOzoneState = 0;
int lastOxygenState = 0;
int lastPumpState = 0;

// Temperature sensor variables
uint8_t tempAdrPump[8] = { 0x28, 0xF1, 0x3E, 0xC7, 0x0B, 0x00, 0x00, 0x03 };  // address of pump temp sensor
uint8_t tempAdrOzone[8] = { 0x28, 0x89, 0xC5, 0xCA, 0x0B, 0x00, 0x00, 0x71 }; // address of ozone temp sensor
uint8_t tempAdrIoT[8] = { 0x28, 0x19, 0xD2, 0xC7, 0x0B, 0x00, 0x00, 0x70 };   // address of IoT temp sensor
float tempPump;   // Pump temp
float tempOzone;  // Ozonator temp
float tempIoT;    // IoT temp

// Timer variables
unsigned long epochBlynk;   // Epoch of time from Blynk server
unsigned int currentTime;   // Current calculated time from epoch value
unsigned int t1StartTime;   // Timer 1 start time
unsigned int t1StopTime;    // Timer 1 stop time
unsigned int t2StartTime;   // Timer 1 start time
unsigned int t2StopTime;    // Timer 1 stop time
unsigned int t3StartTime;   // Timer 1 start time
unsigned int t3StopTime;    // Timer 1 stop time
unsigned int t4StartTime;   // Timer 1 start time
unsigned int t4StopTime;    // Timer 1 stop time
unsigned int t5StartTime;   // Timer 1 start time
unsigned int t5StopTime;    // Timer 1 stop time

int timer1En = 0;           // Timer 1 enabled
int timer2En = 0;           // Timer 2 enabled
int timer3En = 0;           // Timer 3 enabled
int timer4En = 0;           // Timer 4 enabled
int timer5En = 0;           // Timer 5 enabled

int machOn = 0;             // Combined variable to turn machine ON by all timers
int prevMachOn = 0;         // Previous machine state
int mOnT1 = 0;              // Turn machine ON by timer 1
int mOnT2 = 0;              // Turn machine ON by timer 2
int mOnT3 = 0;              // Turn machine ON by timer 3
int mOnT4 = 0;              // Turn machine ON by timer 4
int mOnT5 = 0;              // Turn machine ON by timer 5


// Wifi variables
const int RSSI_MAX =-50;    // define maximum straighten of signal in dBm
const int RSSI_MIN =-100;   // define minimum strength of signal in dBm

//#define PRESSURE
// Water pressure sensor
//#ifdef PRESSURE
#define PRESSURE_PIN A0
//#endif

// RTC object
RTCZero rtc;

/* BLYNK TIMERS */
BlynkTimer sensorTimer;
//BlynkTimer modeTimer;

// ON/OFF button in app for Pump 
BLYNK_WRITE(V0)
{
     
  if (param.asInt() == 1 && machineMode == 1 && appMode == 0)
  {
    pumpOn = true;
    digitalWrite(PUMP_CONTACTOR, HIGH);
    Serial.println("Pump ON!");
  }
  else if (param.asInt() == 0 && machineMode == 1 && appMode == 0)
  {
    machineShutdown();
  }
  else
  {
    Blynk.virtualWrite(0, 0);
  }
  
}

// ON/OFF button in app for oxygen enabled/disabled    
BLYNK_WRITE(V1)
{
  
  // If pump is working, you can turn O and O3 generators ON
  if (pumpOn == true && machineMode == 1 && appMode == 0)
  {
  

    if (param.asInt() == 1 && !oxygenAfterDelayOn)
    {
      oxygenEnabled = true;
      Blynk.virtualWrite(17, 1); // Oxygen enabled
      Serial.println("Oxygen MAN ON!");
      
    }
    else if (param.asInt() == 1 && oxygenAfterDelayOn)
    {
      // Turn oxygen generator right back on if pump wasn't turned on
      Blynk.virtualWrite(17, 1); // Oxygen enabled
      digitalWrite(OXYGEN_CONTACTOR, HIGH);
    }
    else
    {
      Blynk.virtualWrite(17, 0); // Oxygen disabled
      digitalWrite(OXYGEN_CONTACTOR, LOW);
      Serial.println("Oxygen OFF");
      oxygenEnabled = false;
    }
  }
  else if (param.asInt() == 0 && machineMode == 1 && appMode == 0)
  {
    digitalWrite(OXYGEN_CONTACTOR, LOW);
    Blynk.virtualWrite(1, 0);
    Blynk.virtualWrite(17, 0); // Oxygen disabled     
  }
  else
  {
    Blynk.virtualWrite(1, 0);
  }
  
}
    
// ON/OFF button in app for ozone enabled/disabled    
BLYNK_WRITE(V2)
{  
  
  if (pumpOn == true && machineMode == 1 && appMode == 0)
  {
      if (param.asInt() == 1)
      {
        ozoneEnabled = true;
        Blynk.virtualWrite(18, 1); // Ozone enabled
        Serial.println("Ozone ON!");
      }
      else
      {
        ozoneEnabled = false;
        previousOzoneMillis = 0;
        Blynk.virtualWrite(18, 0); // Ozone disabled
        Serial.println("Ozone OFF");
      }
      
  }
  else if (param.asInt() == 0 && machineMode == 1 && appMode == 0)
  {
    ozoneEnabled = false;
    Blynk.virtualWrite(2, 0);   // Ozone button OFF
    Blynk.virtualWrite(18, 0); // Ozone disabled
  }
  else
  {
    Blynk.virtualWrite(2, 0);
  }
}

// Get device status from the APP
BLYNK_WRITE(V3)
{
  statusSwitch = param.asString();
}

// Slider widget for ozone generator interval setup
BLYNK_WRITE(V7)
{
  ozoneInterval = param.asInt();
  //Serial.print("Ozone interval set to: ");
  //Serial.print(ozoneInterval);
  //Serial.println(" min");
}

BLYNK_WRITE(V12) {
  switch (param.asInt()) {
    case 0:
      Serial.println("Mode selection: ON");
      Blynk.syncVirtual(V7);
      appMode = 0;
      machineOn();
      break;
    case 1: {
      Serial.println("Mode selection: OFF");
      appMode = 1;
      machineShutdown();
      break;
    }
    case 2: {
      Serial.println("Mode selection: TIMER");
      appMode = 2;
      machineShutdown(); 
      break;
    }    
  }
}

// Turn on machine by Timers
void onByTimers(){
    if (appMode == 2 && machOn == 1 && autoCombination == 0)
    {
      pumpOn = false;
      Blynk.logEvent("machine_on_o3_and_o2");
      machineOnAuto();
    }
    else if (appMode == 2 && machOn == 1 && autoCombination == 1)
    {
      pumpOn = false;
      Blynk.logEvent("machine_on_o3");
      machineOnAutoOzone();
      
    }
    else if (appMode == 2 && machOn == 1 && autoCombination == 2)
    {
      pumpOn = false;
      Blynk.logEvent("machine_on_o2");
      machineOnAutoOxygen();
      
    }
    else if (appMode == 2 && machOn == 0)
    {
      Blynk.logEvent("machine_off");
      machineShutdown();
    }
}

// Combination when machine is working with timers
BLYNK_WRITE(V16){
  switch (param.asInt()) {
    case 0:
      Serial.println("Auto combination: Pump + O3 + O2");
      autoCombination = 0; // Pump + O3 + O2
      break;
    case 1: {
      Serial.println("Auto combination: Pump + O3");
      autoCombination = 1; // Pump + O3
      break;
    }
    case 2: {
      Serial.println("Auto combination: Pump + O2");
      autoCombination = 2; // Pump + O2
      break;
    }    
  }
}

// Oxygen delay set in seconds
BLYNK_WRITE(V19){
  oxygenDelay = param.asInt();
}

// Timer 1
BLYNK_WRITE(V20)
{
  TimeInputParam t1(param);
  // Process start time
  if (t1.hasStartTime())
  {
    int Start1_hour = t1.getStartHour();
    int Start1_minute = t1.getStartMinute();
    int Start1_second = t1.getStartSecond();
    
    t1StartTime = Start1_hour*60*60 + Start1_minute*60 + Start1_second;
    Serial.print("Start time 1: ");
    Serial.print(Start1_hour);
    Serial.print(":");
    Serial.print(Start1_minute);
    Serial.print(":");
    Serial.print(Start1_second);
    Serial.println();
  }
  // Process stop time
  if (t1.hasStopTime())
  {
    int Stop1_hour = t1.getStopHour() ;
    int Stop1_minute = t1.getStopMinute() ;
    int Stop1_second = t1.getStopSecond();

   
    t1StopTime = Stop1_hour*60*60 + Stop1_minute*60 + Stop1_second;
    Serial.print("Stop time 1: ");
    Serial.print(Stop1_hour);
    Serial.print(":");
    Serial.print(Stop1_minute);
    Serial.print(":");
    Serial.print(Stop1_second);
    Serial.println();
  }
}

// Timer 2
BLYNK_WRITE(V21)
{
  TimeInputParam t(param);
  // Process start time
  if (t.hasStartTime())
  {
    int Start_hour = t.getStartHour();
    int Start_minute = t.getStartMinute();
    int Start_second = t.getStartSecond();
    
    t2StartTime = Start_hour*60*60 + Start_minute*60 + Start_second;
    Serial.print("Start time 2: ");
    Serial.print(Start_hour);
    Serial.print(":");
    Serial.print(Start_minute);
    Serial.print(":");
    Serial.print(Start_second);
    Serial.println();
  }
  // Process stop time
  if (t.hasStopTime())
  {
    int Stop_hour = t.getStopHour() ;
    int Stop_minute = t.getStopMinute() ;
    int Stop_second = t.getStopSecond();
    
    t2StopTime = Stop_hour*60*60 + Stop_minute*60 + Stop_second;
    Serial.print("Stop time 2: ");
    Serial.print(Stop_hour);
    Serial.print(":");
    Serial.print(Stop_minute);
    Serial.print(":");
    Serial.print(Stop_second);
    Serial.println();
  }
}

// Timer 3
BLYNK_WRITE(V22)
{
  TimeInputParam t(param);
  // Process start time
  if (t.hasStartTime())
  {
    int Start_hour = t.getStartHour();
    int Start_minute = t.getStartMinute();
    int Start_second = t.getStartSecond();
    
    t3StartTime = Start_hour*60*60 + Start_minute*60 + Start_second;
    Serial.print("Start time 3: ");
    Serial.print(Start_hour);
    Serial.print(":");
    Serial.print(Start_minute);
    Serial.print(":");
    Serial.print(Start_second);
    Serial.println();
  }
  // Process stop time
  if (t.hasStopTime())
  {
    int Stop_hour = t.getStopHour() ;
    int Stop_minute = t.getStopMinute() ;
    int Stop_second = t.getStopSecond();
    
    t3StopTime = Stop_hour*60*60 + Stop_minute*60 + Stop_second;
    Serial.print("Stop time 3: ");
    Serial.print(Stop_hour);
    Serial.print(":");
    Serial.print(Stop_minute);
    Serial.print(":");
    Serial.print(Stop_second);
    Serial.println();
  }
}

// Timer 4
BLYNK_WRITE(V23)
{
  TimeInputParam t(param);
  // Process start time
  if (t.hasStartTime())
  {
    int Start_hour = t.getStartHour();
    int Start_minute = t.getStartMinute();
    int Start_second = t.getStartSecond();
    
    t4StartTime = Start_hour*60*60 + Start_minute*60 + Start_second;
    Serial.print("Start time 4: ");
    Serial.print(Start_hour);
    Serial.print(":");
    Serial.print(Start_minute);
    Serial.print(":");
    Serial.print(Start_second);
    Serial.println();
  }
  // Process stop time
  if (t.hasStopTime())
  {
    int Stop_hour = t.getStopHour() ;
    int Stop_minute = t.getStopMinute() ;
    int Stop_second = t.getStopSecond();
    
    t4StopTime = Stop_hour*60*60 + Stop_minute*60 + Stop_second;
    Serial.print("Stop time 4: ");
    Serial.print(Stop_hour);
    Serial.print(":");
    Serial.print(Stop_minute);
    Serial.print(":");
    Serial.print(Stop_second);
    Serial.println();
  }
}

// Timer 5
BLYNK_WRITE(V24)
{
  TimeInputParam t(param);
  // Process start time
  if (t.hasStartTime())
  {
    int Start_hour = t.getStartHour();
    int Start_minute = t.getStartMinute();
    int Start_second = t.getStartSecond();
    
    t5StartTime = Start_hour*60*60 + Start_minute*60 + Start_second;
    Serial.print("Start time 5: ");
    Serial.print(Start_hour);
    Serial.print(":");
    Serial.print(Start_minute);
    Serial.print(":");
    Serial.print(Start_second);
    Serial.println();
  }
  // Process stop time
  if (t.hasStopTime())
  {
    int Stop_hour = t.getStopHour() ;
    int Stop_minute = t.getStopMinute() ;
    int Stop_second = t.getStopSecond();
    
    t5StopTime = Stop_hour*60*60 + Stop_minute*60 + Stop_second;
    Serial.print("Stop time 5: ");
    Serial.print(Stop_hour);
    Serial.print(":");
    Serial.print(Stop_minute);
    Serial.print(":");
    Serial.print(Stop_second);
    Serial.println();
  }
}

// Enable timer 1
BLYNK_WRITE(V25){
  timer1En = param.asInt();
  Serial.print("Timer 1: ");
  Serial.println(timer1En);
}
// Enable timer 2
BLYNK_WRITE(V26){
  timer2En = param.asInt();
  Serial.print("Timer 2: ");
  Serial.println(timer2En);
}
// Enable timer 3
BLYNK_WRITE(V27){
  timer3En = param.asInt();
  Serial.print("Timer 3: ");
  Serial.println(timer3En);
}
// Enable timer 4
BLYNK_WRITE(V28){
  timer4En = param.asInt();
  Serial.print("Timer 4: ");
  Serial.println(timer4En);
}
// Enable timer 5
BLYNK_WRITE(V29){
  timer5En = param.asInt();
  Serial.print("Timer 5: ");
  Serial.println(timer5En);
}

BLYNK_WRITE(InternalPinRTC) {   //check the value of InternalPinRTC  
  epochBlynk = param.asLong();      //store time in t variable
  rtc.setEpoch(epochBlynk);
  
}



BLYNK_CONNECTED()
{
  Blynk.sendInternal("rtc", "sync");
  rtc.begin();
  Blynk.syncAll();
  
  autoManMode();
  checkContactorState();
  sensorTimer.setInterval(2000L, checkTime);
}


void setup()
{
  prevMode = 0; // Previous mode of AUTO/OFF/MAN switch on machine
  sensors.begin();  // Start up the DS18B20 library
  //sensors.setWaitForConversion(false);
  
  pinMode(PUMP_CONTACTOR, OUTPUT);
  pinMode(OXYGEN_CONTACTOR, OUTPUT);
  pinMode(OZONE_CONTACTOR, OUTPUT);
  


  pinMode(AUTO, INPUT);
  pinMode(MAN, INPUT);
  
  
  Serial.begin(115200);
  delay(2000);

  BlynkEdgent.begin();

  sensorTimer.setInterval(5000L, sendSensorValues);
  sensorTimer.setInterval(1000L, autoManMode);
  sensorTimer.setInterval(1000L, checkContactorState);
  sensorTimer.setInterval(5000L, temperatureProtection);
  sensorTimer.setInterval(5000L, powerProtection);
  //sensorTimer.setInterval(1000L, onByTimers);
  //checkContactorState();
  //sensorTimer.setInterval(2000L, requestTemperature);

  Blynk.virtualWrite(10, 0);
  Blynk.virtualWrite(9, 0);
  Blynk.virtualWrite(8, 0);
}

void loop() {
  BlynkEdgent.run();
  sensorTimer.run();
  ozonatorCycle();
  oxygenWithDelay();
  
}

void printTime()
{
  print2digits(rtc.getHours());
  Serial.print(":");
  print2digits(rtc.getMinutes());
  Serial.print(":");
  print2digits(rtc.getSeconds());
  Serial.println();
}

void print2digits(int number) {
  if (number < 10) {
    Serial.print("0");
  }
  Serial.print(number);
}

void checkTime()
{
  currentTime = rtc.getHours()*60*60 + rtc.getMinutes()*60 + rtc.getSeconds();
  if (timer1En == 1)
  {
    if (currentTime >= t1StartTime && currentTime < t1StopTime && mOnT1 == 0)
    {
      // TURN MACHINE ON (based on combination selected)
      // Use separate variable
      mOnT1 = 1;
      Serial.print("Machine ON timer 1: ");
      Serial.println(mOnT1);
    }
    else if (currentTime > t1StopTime)
    {
      mOnT1 = 0;
    }
  }
  else if (timer1En == 0)
  {
    mOnT1 = 0;
  }

  if (timer2En == 1)
  {
    if (currentTime >= t2StartTime && currentTime < t2StopTime && mOnT2 == 0)
    {
      // TURN MACHINE ON (based on combination selected)
      // Use separate variable
      mOnT2 = 1;
      Serial.print("Machine ON timer 2: ");
      Serial.println(mOnT2);
    }
    else if (currentTime > t2StopTime)
    {
      mOnT2 = 0;
    }
  }
  else if (timer2En == 0)
  {
    mOnT2 = 0;
  }

  if (timer3En == 1)
  {
    if (currentTime >= t3StartTime && currentTime < t3StopTime && mOnT3 == 0)
    {
      // TURN MACHINE ON (based on combination selected)
      // Use separate variable
      mOnT3 = 1;
      Serial.print("Machine ON timer 3: ");
      Serial.println(mOnT3);
    }
    else if (currentTime > t3StopTime)
    {
     mOnT3 = 0;
    }
  }
  else if (timer3En == 0)
  {
    mOnT3 = 0;
  }

  if (timer4En == 1)
  {
    if (currentTime >= t4StartTime && currentTime < t4StopTime && mOnT4 == 0)
    {
      // TURN MACHINE ON (based on combination selected)
      // Use separate variable
      mOnT4 = 1;
      Serial.print("Machine ON timer 4: ");
      Serial.println(mOnT4);
    }
    else if (currentTime > t4StopTime)
    {
      mOnT4 = 0;
    }
  }
  else if (timer4En == 0)
  {
    mOnT4 = 0;
  }

  if (timer5En == 1)
  {
    if (currentTime >= t5StartTime && currentTime < t5StopTime && mOnT5 == 0)
    {
      // TURN MACHINE ON (based on combination selected)
      // Use separate variable
      mOnT5 = 1;
      Serial.print("Machine ON timer 5: ");
      Serial.println(mOnT5);
    }
    else if (currentTime > t5StopTime)
    {
      mOnT5 = 0;
    }
  }
  else if (timer5En == 0)
  {
    mOnT5 = 0;
  }
    machOn = mOnT1 || mOnT2|| mOnT3|| mOnT4|| mOnT5; // Machine ON variables to turn machine on by timers

    if (machOn != prevMachOn)
    {
      Serial.print("Machine state change (timer): ");
      Serial.println(machOn);
      Serial.print("Current time: ");
      printTime();
      onByTimers(); // Call a function to start/stop the machine
      prevMachOn = machOn;
    }
}

void autoManMode(){
  Blynk.syncVirtual(V3);
  // POPRAVI TA IF stavek kot je bil PREJ!!! poglej komentar  
  if (digitalRead(AUTO) && !digitalRead(MAN)){ // if (digitalRead(AUTO) && !digitalRead(MAN)){ --> POPRAVI POTEM NAZAJ! SAMO ZA TESTIRANJE
    machineMode = 1;
    
    if (pumpState == 1 && appMode == 0 && (statusSwitch != "** AUTO - IN OPERATION **") )
    {
      Serial.println("IN OPERATION");
      Blynk.virtualWrite(3, "** AUTO - IN OPERATION **");
      prevMode = machineMode;
    }
    else if (pumpState == 1 && appMode == 2 && autoCombination == 0 && (statusSwitch != "AUTO - BUBBLING O3+O2") && (statusSwitch == "*** AUTO - STAND BY ***"))
    {
      Blynk.virtualWrite(3, "AUTO - BUBBLING O3+O2");
    }
    else if (pumpState == 1 && appMode == 2 && autoCombination == 1 && (statusSwitch != "** AUTO - BUBBLING O3 **")&& (statusSwitch == "*** AUTO - STAND BY ***"))
    {
      Blynk.virtualWrite(3, "** AUTO - BUBBLING O3 **");
    }
    else if (pumpState == 1 && appMode == 2 && autoCombination == 2 && (statusSwitch != "** AUTO - BUBBLING O2 **")&& (statusSwitch == "*** AUTO - STAND BY ***"))
    {
      Blynk.virtualWrite(3, "** AUTO - BUBBLING O2 **");
    }
    else if (pumpState == 0 && (statusSwitch != "*** AUTO - STAND BY ***"))
    {
      Serial.println("STAND-BY");
      Blynk.virtualWrite(3, "*** AUTO - STAND BY ***");
      prevMode = machineMode;
    }
    
    if (machineMode != prevMode){
      Serial.println("MACHINE AUTO STATUS");
      //Blynk.virtualWrite(3, "***** AUTO *****");
      prevMode = machineMode;
    }
    
  }
  else if (!digitalRead(AUTO) && digitalRead(MAN ))
  {
    //Serial.println("MAN STATUS");
    
    machineMode = 2;
    
    if ((machineMode != prevMode) || statusSwitch != "***** MANUAL *****"){
    Blynk.virtualWrite(3, "***** MANUAL *****");
    prevMode = machineMode;
    }
    
  }
  else if (!digitalRead(AUTO) && !digitalRead(MAN))
  {
    //Serial.println("MACHINE OFF STATUS");
    
    machineMode = 3;
    
    if ((machineMode != prevMode) || statusSwitch != "***** OFF *****"){
    Blynk.virtualWrite(3, "***** OFF *****");
    Blynk.virtualWrite(12, 1);
    prevMode = machineMode;
    }
  }
  else if (digitalRead(AUTO) && digitalRead(MAN))
  {
    //Serial.println("MODE ERROR STATUS");
    
    machineMode = 4;
    if ((machineMode != prevMode) || statusSwitch != "***** ERROR *****"){
    Blynk.virtualWrite(3, "***** ERROR *****");
    prevMode = machineMode;
    }
  }

  if (machineMode != 1)
  {
    pumpOn = false;
    machineShutdown(); 
  }
  //Serial.print("Pump state: ");
  //Serial.println(pumpState);
  //Serial.print("Ozonator state: ");
  //Serial.println(ozonatorState);
}


float readACCurrentValue()
{
  float ACCurrtntValue = 0;
  float peakVoltage = 0;  
  float voltageVirtualValue = 0;  //Vrms
  for (int i = 0; i < 20; i++)
  {
    peakVoltage += analogRead(AC_PIN);   //read peak voltage
    delay(1);
  }
  peakVoltage = peakVoltage / 20;  // 20 values so we measure whole period 50Hz 
  voltageVirtualValue = peakVoltage * 0.707;    //change the peak voltage to the Virtual Value of voltage

  /*The circuit is amplified by 2 times, so it is divided by 2.*/
  voltageVirtualValue = (voltageVirtualValue / 1024 * VREF ) / 2;  

  ACCurrtntValue = voltageVirtualValue * ACTectionRange;

  return ACCurrtntValue;
}

float calculatePower(double acVoltage){
  float amps = readACCurrentValue(); //read AC Current Value
  float totalPower = amps*acVoltage;

  return totalPower;
  
}





void requestTemperature(){
  
  sensors.requestTemperatures();
 
  tempPump = sensors.getTempC(tempAdrPump);
  tempOzone = sensors.getTempC(tempAdrOzone);
  tempIoT = sensors.getTempC(tempAdrIoT);
}

void checkContactorState()
{
  if (digitalRead(PUMP_CONTACTOR) != lastPumpState)
  {
    pumpState = digitalRead(PUMP_CONTACTOR);
    Blynk.virtualWrite(9, pumpState);
    lastPumpState = pumpState;
  }

  if (digitalRead(OXYGEN_CONTACTOR) != lastOxygenState)
  {
    oxygenState = digitalRead(OXYGEN_CONTACTOR);
    Blynk.virtualWrite(10, oxygenState);
    lastOxygenState = oxygenState;
  }

  if (digitalRead(OZONE_CONTACTOR) != lastOzoneState)
  {
    ozonatorState = digitalRead(OZONE_CONTACTOR);
    Blynk.virtualWrite(8, ozonatorState);
    lastOzoneState = ozonatorState;
  }

}


void sendSensorValues(){
  requestTemperature();
  wifiStrength(); 
  float totalPower = calculatePower(acVoltage);

  Blynk.virtualWrite(4, totalPower);
  Blynk.virtualWrite(5, tempPump);
  Blynk.virtualWrite(6, tempOzone);
  Blynk.virtualWrite(14, tempIoT);

}

void ozonatorCycle()
{
  if (ozoneEnabled == true)
  { 
    unsigned long currentMillis = millis();

    // When you turn on Ozone generator there is some delay (cca 10s)
    if (!ozoneOnDelay && !ozoneAfterDelayOn){
      Serial.println("----Ozone 10s counter started----");
      startOzoneOnDelay = millis();
      ozoneOnDelay = true;
    }
    else if ((currentMillis - startOzoneOnDelay >= ozoneDelay*1000) && ozoneOnDelay)
    {
      ozoneOnDelay = false;
      Serial.println("Ozone ON after delay.");
      ozoneAfterDelayOn = true;
      
    }
      //Serial.println("--------------");
      //Serial.print("currentMillis - previousOzoneMillis: ");
      //Serial.println(currentMillis - previousOzoneMillis);

      //Serial.print("ozoneInterval*1000*60: ");
      //Serial.println(ozoneInterval*1000*60);
      //Serial.println("--------------");
      if (((currentMillis - previousOzoneMillis >= ozoneInterval*1000*60)|| (ozoneOnFirstTime)) && ozoneAfterDelayOn  )
      {

        if (ozoneOnFirstTime){
          ozoneOnFirstTime = false;
        }
      
        // if the ozone generator is off turn it on and vice-versa:
        if (ozoneState == LOW) 
        {
          prevOzoneState = ozoneState;
          ozoneState = HIGH;
          Serial.println("Generating ozone");
        } 
        else if (ozoneState == HIGH) 
        {
          prevOzoneState = ozoneState;
          ozoneState = LOW;
          Serial.println("Not generating O3 - stand by...");
        }
        
        if (ozoneState != prevOzoneState)
        {
          Serial.print("Ozonator switched state after: ");
          Serial.print((currentMillis - previousOzoneMillis)/(1000*60));
          Serial.println(" min");
          Serial.print("Ozone State: ");
          Serial.println(ozoneState);
          digitalWrite(OZONE_CONTACTOR, ozoneState);
        }
  
      previousOzoneMillis = currentMillis;
      
      }

  
  }
  else if (ozoneEnabled == false)
  {
    if (digitalRead(OZONE_CONTACTOR) == HIGH){
      digitalWrite(OZONE_CONTACTOR, LOW); 
      
    }
    ozoneAfterDelayOn = false;
    ozoneOnDelay = false;
    previousOzoneMillis = 0;
    ozoneState = LOW;
    ozoneOnFirstTime = true;
    
  }
}

// Oxygen generator ON with delay
void oxygenWithDelay()
{
  if ( oxygenEnabled == true )
  {
    unsigned long currentMillis = millis();

    // When you turn on Oxygen generator there is some delay (cca 1min)
    if (!oxygenOnDelay && !oxygenAfterDelayOn){
      Serial.println("----Oxygen counter started----");
      startOxygenOnDelay = millis();
      oxygenOnDelay = true;
    }
    else if ((currentMillis - startOxygenOnDelay >= oxygenDelay*1000) && oxygenOnDelay)
    {
      oxygenOnDelay = false;
      Serial.println("Oxygen ON after delay.");
      oxygenAfterDelayOn = true;
      // Turn the oxygen generator ON
      digitalWrite(OXYGEN_CONTACTOR, HIGH); 
    }
    
  }
  else if ( oxygenEnabled == false )
  {
    if (digitalRead(OXYGEN_CONTACTOR) == HIGH)
    {
      digitalWrite(OXYGEN_CONTACTOR, LOW);
        
    }
    oxygenOnDelay = false;
    oxygenAfterDelayOn = false;
    
    
  }
  
}

/* SAFTEY FUNCTIONS */

unsigned long startTimeMaxPower;
unsigned long currentTimeMaxPower;
unsigned long periodMaxPower = 3;  // Max power until shutdown in minutes
boolean timingMaxPower = false;

boolean timingMinPower = false;
unsigned long startTimeMinPower;
unsigned long currentTimeMinPower;
unsigned long periodMinPower = 3;  // Max power until shutdown in minutes
const unsigned int maxPower = 2500; // Set the maximum power allowed
const unsigned int minPower = 1150; // Set the minimum power allowed

void powerProtection()
{
  float currentPower = calculatePower(acVoltage);
  // If power too high
  if (currentPower > maxPower)
  {
    
    
    currentTimeMaxPower = millis(); // Current time in case of power too high

    if (!timingMaxPower)
    {
      // Power too high - SEND NOTIFICATION TO BLYNK
      if (machineMode == 1)
      {
        Blynk.logEvent("power_too_high");
      }
      else
      {
        Blynk.logEvent("power_too_high_man");
      }
      
      Serial.println("Power too HIGH - send notification.");
      startTimeMaxPower = millis();
      timingMaxPower = true;
    }
    else if ( currentTimeMaxPower - startTimeMaxPower >= (periodMaxPower*60*1000))
    {
      // Power too high for too long - machine shutdown - SEND NOTIFICATION TO BLYNK
      machineSafetyShutdown();
      Blynk.logEvent("power_failure");
      Serial.println("Power failure (too HIGH) - Machine safety shutdown.");
      timingMaxPower = false;
    }
    
    
    
  }
  // If power too low
  else if ( (currentPower < minPower) && digitalRead(PUMP_CONTACTOR) && digitalRead(OZONE_CONTACTOR) && digitalRead(OXYGEN_CONTACTOR)  )
  {
    
    currentTimeMinPower = millis(); // Current time in case of power too low

    if (!timingMinPower){
      if (machineMode == 1)
      {
        Blynk.logEvent("power_too_low");
      }
      else
      {
        Blynk.logEvent("power_too_low_man");
      }
      Blynk.logEvent("power_too_low");
      Serial.println("Power too LOW - send notification.");
      startTimeMinPower = millis();
      timingMinPower = true;
    }
    else if ( (currentTimeMinPower - startTimeMinPower >= (periodMinPower*60*1000)) && (machineMode == 1) )
    {
      // Power too low for too long - machine shutdown - SEND NOTIFICATION TO BLYNK
      machineSafetyShutdown();
      Blynk.logEvent("power_failure_low");
      Serial.println("Power failure (too LOW) - Machine safety shutdown.");
      timingMinPower = false;
    }
    
  }
  else
  {
    // If in both cases power goes back to normal counting stops
    timingMaxPower = false;
    timingMinPower = false;
    
  }
}

const float maxTempPump = 75.0;
const float maxTempOzone = 60.0;
const float maxTempIot = 50;
// If temp of pump, oxygen generator or ozone generator exceedes some value - set warning
void temperatureProtection()
{
  if (tempPump > maxTempPump)
  {
    Serial.println("Pump temperature too high");
    Blynk.logEvent("pump_temperature_high");
  }

  if (tempOzone > maxTempOzone)
  {
    Serial.println("Ozone generator temperature too high");
    Blynk.logEvent("ozone_generator_temperature_high");
  }
  if (tempOzone > maxTempOzone)
  {
    Serial.println("Ozone generator temperature too high");
    Blynk.logEvent("ozone_generator_temperature_high");
  }
  if (tempIoT > maxTempIot)
  {
    Serial.println("IoT cabinet temperature too high");
    Blynk.logEvent("iot_temperature_too_high");
  }
  
}


// Machine shutdown function
void machineShutdown(){
    ozoneEnabled = false;
    oxygenEnabled = false;
    pumpOn = false;
    mOnT1 = 0;
    mOnT2 = 0;
    mOnT3 = 0;
    mOnT4 = 0;
    mOnT5 = 0;
    machOn = 0;
    prevMachOn = 0;
    
    digitalWrite(PUMP_CONTACTOR, LOW);
    digitalWrite(OZONE_CONTACTOR, LOW);
    digitalWrite(OXYGEN_CONTACTOR, LOW);
    Blynk.virtualWrite(0, 0);
    Blynk.virtualWrite(1, 0);
    Blynk.virtualWrite(2, 0);
    Blynk.virtualWrite(17, 0); // Oxygen enabled LED
    Blynk.virtualWrite(18, 0); // Ozone enabled LED
    
}

// Safety shutdown
void machineSafetyShutdown(){
    ozoneEnabled = false;
    oxygenEnabled = false;
    pumpOn = false;
    digitalWrite(PUMP_CONTACTOR, LOW);
    digitalWrite(OZONE_CONTACTOR, LOW);
    digitalWrite(OXYGEN_CONTACTOR, LOW);
    Blynk.virtualWrite(0, 0);
    Blynk.virtualWrite(1, 0);
    Blynk.virtualWrite(2, 0);
    Blynk.virtualWrite(12, 1); // Mode = OFF
    Blynk.virtualWrite(17, 0); // Oxygen enabled LED
    Blynk.virtualWrite(18, 0); // Ozone enabled LED
    
}


// Function to turn machine ON using Automations in pump+O3+O2 MODE
void machineOnAuto()
{
  if (machineMode == 1 )
  {
    digitalWrite(PUMP_CONTACTOR, HIGH);
    ozoneEnabled = true;
    oxygenEnabled = true;
    Blynk.virtualWrite(17, 1); // Oxygen enabled LED
    Blynk.virtualWrite(18, 1); // Ozone enabled LED
  }
    
}

// Function to turn machine ON using Automations in pump+O3 MODE
void machineOnAutoOzone()
{
  if (machineMode == 1)
  {
    oxygenEnabled = false;
    ozoneEnabled = true;
    digitalWrite(OXYGEN_CONTACTOR, LOW);
    digitalWrite(PUMP_CONTACTOR, HIGH);
    Blynk.virtualWrite(17, 0); // Oxygen enabled LED
    Blynk.virtualWrite(18, 1); // Ozone enabled LED
    
  }
    
}

// Function to turn machine ON using Automations in pump+O2 MODE
void machineOnAutoOxygen()
{
  if (machineMode == 1)
  {
    ozoneEnabled = false;
    oxygenEnabled = true;
    digitalWrite(PUMP_CONTACTOR, HIGH);
    Blynk.virtualWrite(17, 1); // Oxygen enabled LED
    Blynk.virtualWrite(18, 0); // Ozone enabled LED
  }   
}

void machineOn(){
  if (machineMode == 1)
  {
    pumpOn = true;
    ozoneEnabled = true;
    oxygenEnabled = true;
    
    digitalWrite(PUMP_CONTACTOR, HIGH);
    
    Blynk.virtualWrite(0, 1);
    Blynk.virtualWrite(1, 1);
    Blynk.virtualWrite(2, 1);
    Blynk.virtualWrite(17, 1); // Oxygen enabled LED
    Blynk.virtualWrite(18, 1); // Ozone enabled LED
  }
    
}

/* WIFI STRENGTH FUNCTIONS */
long dBmtoPercentage(long dBm)
{
  long quality;
    if(dBm <= RSSI_MIN)
    {
        quality = 0;
    }
    else if(dBm >= RSSI_MAX)
    {  
        quality = 100;
    }
    else
    {
        quality = 2 * (dBm + 100);
   }

     return quality;
}//dBmtoPercentage 

void wifiStrength(){
  long wifiSignal = dBmtoPercentage(WiFi.RSSI());
  //Serial.print("signal strength (%):");
  //Serial.println(wifiSignal);
  Blynk.virtualWrite(15, wifiSignal);
}

/* EXTRA FUNCTIONS */
void pSensor(){
  const float  OffSet = 0.2997;
  float V, P;
  V = (analogRead(PRESSURE_PIN)*3.3)/1024;     //Sensor output voltage V = analogRead(17) * 5.00 / 1024;
  P = ((V - OffSet)*606.06)/100;             //Calculate water pressure

  Serial.print("Voltage:");
  Serial.print(V,4);
  Serial.print("V ");

  
  Serial.print("Pressure:");
  Serial.print(P, 1);
  Serial.println(" Bar");
  Serial.println();
  
}
