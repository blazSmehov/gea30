#define BLYNK_TEMPLATE_ID "TMPL60vVWn3M"
#define BLYNK_TEMPLATE_NAME "GEASmartCopy"
#define BLYNK_DEVICE_NAME "GEA 30 wbstg0262024"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RTCZero.h>
#include <ModbusMaster.h>
#include "DO_reg_func.h"
#include "machine_func.h"
#include "Modbus_func.h"
#include <ArduinoRS485.h> 
#include <ArduinoModbus.h>
#define EXTERN extern
#include "variables.h"
#include "constants.h"

#define BLYNK_FIRMWARE_VERSION        "0.1.30"

#define BLYNK_PRINT Serial
#define APP_DEBUG

#include "BlynkEdgent.h"

// tega ne brisi 

#define ONE_WIRE_BUS 0

#define ONE_WIRE_BUS 0 // Data wire is plugged into digital pin 0 on the Arduino

/* HARDWARE DEFINES - END */
// Setup a oneWire instance to communicate with any OneWire device

OneWire oneWire(ONE_WIRE_BUS);  
// Pass oneWire reference to DallasTemperature library
DallasTemperature sensors(&oneWire);

unsigned long previousTime1 = 0;

//
// variables for temperature sensors
uint8_t tempAdrPump[8] = { 0x28, 0x56, 0x6D, 0xC9, 0x0B, 0x00, 0x00, 0x14 };  // address of pump temp sensor
uint8_t tempAdrOzone[8] = { 0x28, 0x37, 0xDD, 0xC8, 0x0B, 0x00, 0x00, 0x20 }; // address of ozone temp sensor
uint8_t tempAdrIoT[8] = { 0x28, 0xEF, 0x1E, 0xC9, 0x0B, 0x00, 0x00, 0xA6 };   // address of IoT temp sensor

float tempPump;   // Pump temp
float tempCompressor;  // Ozonator temp tempCompressor
float tempCabinet;    // IoT temp

//git comment

void machineShutdown();
void machineOn();
void callDoSensor();
void sendSensorValues();
void temperatureProtection();
void autoManMode();
void checkTime();
void checkContactorState();
void powerProtection();
void outputsWhenConnected();
void pressureProtection();
void regulacija_ORP();
void ozonatorCycle();
void printTime();
void readHoldingRegisterValues1();
// void writeHoldingRegisterValue();

// declarations, to zbriši pol

bool blynk_prva_iteracija = true;
bool blynk_druga_iteracija = false;

/* BLYNK TIMERS */
BlynkTimer sensorTimer;
//BlynkTimer modeTimer;

// ON/OFF button in app for Pump 
BLYNK_WRITE(V0)
{
  if( machineMode == 1 && appMode == 0)
  {
    if (param.asInt() == 1)
    {
      pumpOn = true;
      digitalWrite(PUMP_CONTACTOR, HIGH);
      // Serial.println("Pump ON!");
    }
    else if (param.asInt() == 0)
    {
      machineShutdown();
      // Serial.println("Problem1");
    }
  }
  else
  {
    pumpOn = false;
    Blynk.virtualWrite(0, 0);
  }
}

// global flag

bool oxygenAndOzoneEnabled = false;
bool oxygenOnEnabled = false;

// ON/OFF button in app for oxygen enabled/disabled    
BLYNK_WRITE(V1)
{
  // If pump is working, you can turn O and O3 generators ON
  if (pumpOn == true && machineMode == 1 && appMode == 0)
  {
    
    if (oxygenAndOzoneEnabled){
      return;
    }
    else if (param.asInt() == 1 )
    {
      oxygenEnabled = true;
      Blynk.virtualWrite(17, 1); // Oxygen enabled
      oxygenOnEnabled = true;
      // Serial.println("Oxygen MAN ON!");
      
    }
    else if (param.asInt() == 0)
    {
      oxygenEnabled = false; // ???
      Blynk.virtualWrite(17, 0); // Oxygen disabled
      digitalWrite(OXYGEN_CONTACTOR, LOW);
      oxygenOnEnabled = false;
      // Serial.println("Oxygen OFF");
      
    }
  }
  else if (pumpOn == false && machineMode == 1 && appMode == 0)
  {
    oxygenEnabled = false;
    digitalWrite(OXYGEN_CONTACTOR, LOW);
    Blynk.virtualWrite(1, 0);
    Blynk.virtualWrite(17, 0); // Oxygen disabled   
  }
  else
  {
    Blynk.virtualWrite(1, 0);
  }
}
    
// ON/OFF button in app for ozone + oxygen enabled/disabled    
BLYNK_WRITE(V2)
{  
  
  if (pumpOn == true && machineMode == 1 && appMode == 0)
  {
    if (oxygenOnEnabled) {
      return;
    }
    else if (param.asInt() == 1)
    {
      ozoneEnabled = true;
      oxygenEnabled = true;
      oxygenAndOzoneEnabled = true;
      Blynk.virtualWrite(18, 1); // Ozone enabled
      Blynk.virtualWrite(17, 1); // Oxygen enabled
    }
    else if (param.asInt() == 0)
    {
      ozoneEnabled = false;
      oxygenEnabled = false;
      oxygenAndOzoneEnabled = false;
      previousOzoneMillis = 0;
      Blynk.virtualWrite(2, 0);   // Ozone button OFF
      Blynk.virtualWrite(18, 0);  // Ozone disabled
      digitalWrite(OXYGEN_CONTACTOR, LOW);
      Blynk.virtualWrite(1, 0);
      Blynk.virtualWrite(17, 0); // Oxygen disabled   
    }
  }
  else
  {
    Blynk.virtualWrite(2, 0);
    Blynk.virtualWrite(1, 0);
  }
}

// Get device status from the APP
BLYNK_WRITE(V3)
{
  statusSwitch = param.asString();
}

BLYNK_WRITE(V12) {
  switch (param.asInt()) {
    case 0:
      // Serial.println("Mode selection: ON");
      Blynk.syncVirtual(V7);
      appMode = 0;
      reg_enable = false;
      machineOn();
      rezim_dol1 = false;
      potek_dol = false;
      potek_gor = false;

      break;
    case 1: {
      // Serial.println("Mode selection: OFF");
      appMode = 1;
      machineShutdown();
      rezim_dol1 = false;
      reg_enable = false;
      potek_dol = false;
      potek_gor = false;

      break;
    }
    case 2: {
      // Serial.println("Mode selection: TIMER");
      appMode = 2;
      machineShutdown(); 
      rezim_dol1 = false; 
      reg_enable = false;
      potek_dol = false;
      potek_gor = false;

      break;
    }    
    case 3: {
      // Serial.println("Mode selection: REG");
      
      appMode = 3;

      reg_enable = true;
      Pumpa_ugas = true;

      break;
    }
  }
}

// GEA devices have autoCombinations 0 and 1 (O3 + O2 and O2)

// Combination when machine is working with timers
BLYNK_WRITE(V16){
  switch (param.asInt()) {
    case 0:
      //Serial.println("Auto combination: Pump + O3 + O2");
      autoCombination = 0; // Pump + O3 + O2
    
      break;
    case 1: {
      //Serial.println("Auto combination: Pump + O2");
      autoCombination = 1; // Pump + O3
      
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
  }
  // Process stop time
  if (t1.hasStopTime())
  {
    int Stop1_hour = t1.getStopHour() ;
    int Stop1_minute = t1.getStopMinute() ;
    int Stop1_second = t1.getStopSecond();

   
    t1StopTime = Stop1_hour*60*60 + Stop1_minute*60 + Stop1_second;
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
  }
  // Process stop time
  if (t.hasStopTime())
  {
    int Stop_hour = t.getStopHour() ;
    int Stop_minute = t.getStopMinute() ;
    int Stop_second = t.getStopSecond();
    
    t2StopTime = Stop_hour*60*60 + Stop_minute*60 + Stop_second;
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
  }
  // Process stop time
  if (t.hasStopTime())
  {
    int Stop_hour = t.getStopHour() ;
    int Stop_minute = t.getStopMinute() ;
    int Stop_second = t.getStopSecond();
    
    t3StopTime = Stop_hour*60*60 + Stop_minute*60 + Stop_second;
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
  }
  // Process stop time
  if (t.hasStopTime())
  {
    int Stop_hour = t.getStopHour() ;
    int Stop_minute = t.getStopMinute() ;
    int Stop_second = t.getStopSecond();
    
    t4StopTime = Stop_hour*60*60 + Stop_minute*60 + Stop_second;
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
  }
  // Process stop time
  if (t.hasStopTime())
  {
    int Stop_hour = t.getStopHour() ;
    int Stop_minute = t.getStopMinute() ;
    int Stop_second = t.getStopSecond();
    
    t5StopTime = Stop_hour*60*60 + Stop_minute*60 + Stop_second;
  }
}

// Enable timer 1
BLYNK_WRITE(V25){
  timer1En = param.asInt();
}
// Enable timer 2
BLYNK_WRITE(V26){
  timer2En = param.asInt();
}
// Enable timer 3
BLYNK_WRITE(V27){
  timer3En = param.asInt();
}
// Enable timer 4
BLYNK_WRITE(V28){
  timer4En = param.asInt();
}
// Enable timer 5
BLYNK_WRITE(V29){
  timer5En = param.asInt();
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
  blynkConnected = 1;
  blynk_druga_iteracija = true;
  Blynk.syncVirtual(V35);
  Blynk.syncVirtual(V36);
  outputsWhenConnected();
  checkContactorState();
  sensorTimer.setInterval(711L, autoManMode);
  sensorTimer.setInterval(1023L, checkContactorState);
  sensorTimer.setInterval(1235L, oxygenWithDelay);
  sensorTimer.setInterval(6359L, callDoSensor);
  sensorTimer.setInterval(1000L, regulacija_O2);
  sensorTimer.setInterval(1522L, ozonatorCycle);
  sensorTimer.setInterval(3109L, checkTime);
  sensorTimer.setInterval(10307L, sendSensorValues);
  sensorTimer.setInterval(12018L, powerProtection);
  sensorTimer.setInterval(15099L, temperatureProtection);
  sensorTimer.setInterval(1000L, pressureProtection);
  sensorTimer.setInterval(6331L, readHoldingRegisterValues1);
}

void setup()
{
  prevMode = 0; // Previous mode of AUTO/OFF/MAN switch on machine
  sensors.begin();  // Start up the DS18B20 library
  
  pinMode(PUMP_CONTACTOR, OUTPUT);
  pinMode(OXYGEN_CONTACTOR, OUTPUT);
  pinMode(OZONE_CONTACTOR, OUTPUT);
  
  // RS_EN defined as output - Modbus enable
  pinMode(RS_EN, OUTPUT);
  
  pinMode(AUTO, INPUT);
  pinMode(MAN, INPUT);
  
  Serial.begin(115200);
  delay(2000);

  Serial1.begin(9600);
  delay(1000);

  // modbus.begin(Slave_ID, Serial1);
  // modbus.idle(yield);
  if (!ModbusRTUClient.begin(9600)) {
    Serial.println("Failed to start Modbus RTU Client!");
    while (1);
  }

  // Postavimo RS_EN na 0
  modbus.preTransmission(preTransmission);
  modbus.postTransmission(postTransmission);

  BlynkEdgent.begin();

  Blynk.virtualWrite(10, 0);
  Blynk.virtualWrite(9, 0);
  Blynk.virtualWrite(8, 0);

}

float vrednostORP = 0;

// int id = 0;

void readHoldingRegisterValues1() {
  // Serial.print("Reading Input Register values ... ");
  // Serial.print("im am trying this id: ");
  // Serial.println(id);

  // read 10 Input Register values from (slave) id 42, address 0x00
  if (!ModbusRTUClient.requestFrom(40, HOLDING_REGISTERS, 0x09, 1)) {
    Serial.print("failed! ");
    Serial.println(ModbusRTUClient.lastError());
  } else {
    // Serial.println("success");

    while (ModbusRTUClient.available()) {
      vrednostORP = 0.1 * ModbusRTUClient.read();
      // Serial.print("vrednost orp: ");
      // Serial.println(vrednostORP);
      }
    // Serial.println();
  }
  // id++;
}

void loop() {
  
  sensorTimer.run();

  if (blynk_prva_iteracija == true){
    BlynkEdgent.run();
    // Serial.println("zacetek :(");
  }

  if ((WiFi.status() == WL_CONNECTED)&&(blynk_druga_iteracija == true)){
    // Serial.println("povezano :O");
    BlynkEdgent.run();
    blynk_prva_iteracija = false;
  }

  else if(blynk_druga_iteracija == true) {
    WiFi.begin(configStore.wifiSSID, configStore.wifiPass);
    Blynk.config(configStore.cloudToken, configStore.cloudHost, configStore.cloudPort);
    Blynk.disconnect();
    Blynk.connect(0);
  }
  
  if (blynkConnected == 1){
    orpSensor();
  }
  
  else if ((WiFi.status() != WL_CONNECTED)&& blynkConnected == 1){
    //Serial.println("Blynk diconnected");
    blynkConnected = 0;
  }
}

/* -- --*/

BLYNK_WRITE(V35){
  regulacija_max = param.asDouble();
} 
BLYNK_WRITE(V36){
  regulacija_min = param.asDouble();
}

void requestTemperature(){

  Serial.println("appMode: ");
  Serial.println(appMode);
  
  sensors.requestTemperatures();
 
  tempPump = sensors.getTempC(tempAdrPump);
  tempCompressor = sensors.getTempC(tempAdrOzone);
  tempCabinet = sensors.getTempC(tempAdrIoT);
}

void sendSensorValues(){
  requestTemperature();
  wifiStrength(); 
  float pressure = pSensor();
  float totalPower = calculatePower(acVoltage);

  

  Blynk.virtualWrite(4, totalPower);
  Blynk.virtualWrite(5, tempPump);
  // should be virtual write 6, but is changed for device FAUNA
  Blynk.virtualWrite(6, tempCompressor); // Prev. tempCompressor
  Blynk.virtualWrite(14, tempCabinet);
  Blynk.virtualWrite(33, pressure);
  Blynk.virtualWrite(34, vrednostORP);
}

void callDoSensor(){
  float doValue, doSat, doTemp;
  if ( blynkConnected == 1){
    readDO(doValue, doSat, doTemp);
    // Serial.println("doValue");
    // Serial.println(doValue);
    // Serial.println(doTemp);
    
    Blynk.virtualWrite(30, doValue);
    Blynk.virtualWrite(31, doSat);
    Blynk.virtualWrite(32, doTemp);
  }
}

/* -- NI V UPORABI -- */
void doLoop(){
  unsigned long currentMillis = millis();
 
  if(currentMillis - previousMillisDo >  millisDoInterval) {
    //Serial.println("doLoop interval reached");
    callDoSensor();
    previousMillisDo = currentMillis;   
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
      Blynk.logEvent("machine_on_o2");
      machineOnAutoOxygen();
      
    }
    else if (appMode == 2 && machOn == 0)
    {
      Blynk.logEvent("machine_off");
      machineShutdown();
      //Serial.println("Problem4");
    }
}

void checkTime()
{
  currentTime = rtc.getHours()*60*60 + rtc.getMinutes()*60 + rtc.getSeconds();
  if (timer1En == 1)
  {
    if (currentTime >= t1StartTime && currentTime < t1StopTime && mOnT1 == 0)
    {
      mOnT1 = 1;
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
      mOnT2 = 1;
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
      mOnT3 = 1;
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
      mOnT4 = 1;
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
      mOnT5 = 1;
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
      printTime();
      onByTimers(); // Call a function to start/stop the machine
      prevMachOn = machOn;
    }
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
    //Serial.print("0");
  }
  //Serial.print(number);
}

void outputsWhenConnected()
{
  /* Send contactor values when Blynk is connected */
  Blynk.virtualWrite(9, digitalRead(PUMP_CONTACTOR));
  Blynk.virtualWrite(10, digitalRead(OXYGEN_CONTACTOR));
  Blynk.virtualWrite(8, digitalRead(OZONE_CONTACTOR));
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

void wifiStrength(){
  long wifiSignal = dBmtoPercentage(WiFi.RSSI());
  //Serial.print("signal strength (%):");
  //Serial.println(wifiSignal);
  Blynk.virtualWrite(15, wifiSignal);
}

void autoManMode(){
  Blynk.syncVirtual(V3);
  bool automatic = digitalRead(AUTO);
  bool manual = digitalRead(MAN);  
  
  if (automatic && !manual){
    machineMode = 1;
   
    
    if (pumpState == 1 && appMode == 0 && (statusSwitch != "** AUTO - IN OPERATION **"))
    {
      //Serial.println("IN OPERATION");
      Blynk.virtualWrite(3, "** AUTO - IN OPERATION **");
      prevMode = machineMode;
    }
    else if (pumpState == 1 && appMode == 2 && autoCombination == 0 && (statusSwitch != "AUTO - BUBBLING O3+O2") && (statusSwitch == "*** AUTO - STAND BY ***"))
    {
      Blynk.virtualWrite(3, "AUTO - BUBBLING O3+O2");
    }
    else if (pumpState == 1 && appMode == 2 && autoCombination == 1 && (statusSwitch != "** AUTO - BUBBLING O2 **")&& (statusSwitch == "*** AUTO - STAND BY ***"))
    {
      Blynk.virtualWrite(3, "** AUTO - BUBBLING O2 **");
    }
    else if(appMode == 3 && (statusSwitch != "** DO REGULATION **"))
    {
      Blynk.virtualWrite(3, "** DO REGULATION **");
    }
    else if (pumpState == 0 && (statusSwitch != "*** AUTO - STAND BY ***") && appMode != 3)
    {
      //Serial.println("STAND-BY");
      Blynk.virtualWrite(3, "*** AUTO - STAND BY ***");
      prevMode = machineMode;
    }
    
    if (machineMode != prevMode){
      //Serial.println("MACHINE AUTO STATUS");
      //Blynk.virtualWrite(3, "***** AUTO *****");
      prevMode = machineMode;
    }
    
  }
  else if (!automatic && manual)
  {
    //Serial.println("SWITCH ON MAN");
    
    machineMode = 2;
    
    if ((machineMode != prevMode) || statusSwitch != "***** MANUAL *****"){
    Blynk.virtualWrite(3, "***** MANUAL *****");
    prevMode = machineMode;
    }
    
  }
  else if (!automatic && !manual)
  {
    //Serial.println("SWITCH OFF");
    machineMode = 3;
    if ((machineMode != prevMode) || statusSwitch != "***** OFF *****"){
    Blynk.virtualWrite(3, "***** OFF *****");
    Blynk.virtualWrite(12, 1);
    prevMode = machineMode;
    }
  }
  else if (automatic && manual)
  {
    //Serial.println("MODE SWITCH ERROR");
    
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
}

void powerProtection()
{

  if (tempCabinet <= powerLimitTemp)
  {
    minPower = 950;
  }
  else if (tempCabinet > powerLimitTemp)
  {
    minPower = 850;
  }
  
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
      
      //Serial.println("Power too HIGH - send notification.");
      startTimeMaxPower = millis();
      timingMaxPower = true;
    }
    else if ( currentTimeMaxPower - startTimeMaxPower >= (periodMaxPower*60*1000))
    {
      // Power too high for too long - machine shutdown - SEND NOTIFICATION TO BLYNK
      machineSafetyShutdown();
      Blynk.logEvent("power_failure");
      //Serial.println("Power failure (too HIGH) - Machine safety shutdown.");
      timingMaxPower = false;
    }
    
    
    
  }
  // If power too low
  else if ( (currentPower < minPower) && digitalRead(PUMP_CONTACTOR) && digitalRead(OZONE_CONTACTOR) && digitalRead(OXYGEN_CONTACTOR))
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
      //Blynk.logEvent("power_too_low");
      //Serial.println("Power too LOW - send notification.");
      startTimeMinPower = millis();
      timingMinPower = true;
    }
    else if ( (currentTimeMinPower - startTimeMinPower >= (periodMinPower*60*1000)) && (machineMode == 1) )
    {
      // Power too low for too long - machine shutdown - SEND NOTIFICATION TO BLYNK
      machineSafetyShutdown();
      Blynk.logEvent("power_failure_low");
      //Serial.println("Power failure (too LOW) - Machine safety shutdown.");
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

void pressureProtection(){

  float pressure = pSensor();

  if((pumpOn == true)||(machineMode == 2)){
    
    if (pressure >= 3){
      pressureTimerMax = millis();
      
      if (!timingMaxPressure){
        Blynk.logEvent("pressure_too_high");
        timingMaxPressure = true;
        pressureMaxTimer = millis();
        Serial.println("pressure too high");
      }
      else if ((pressureTimerMax - pressureMaxTimer) >= 180000){
        Blynk.logEvent("machine_shutdown_high_pressure");
        timingMaxPressure = false;
        machineSafetyShutdown();
        // appMode = 1;
        }
      }
  
    else if (pressure < 1){
      pressureTimerMin = millis();
      Serial.println("pritisk prenizek");
      
      if (!timingMinPressure){
        Blynk.logEvent("pressure_too_low");
        timingMinPressure = true;
        pressureMinTimer = millis();
        Serial.println("pressure too low");
      }

      else if ((pressureTimerMin - pressureMinTimer) >= 180000){
        Blynk.logEvent("machine_shutdown_low_pressure");
        timingMinPressure = false;
        machineSafetyShutdown();
        // appMode = 1;
        }
    }
  
    else if ((pressure >= 1)&&(pressure < 3)){
      timingMaxPressure = false;
      timingMinPressure = false;
      pressureMinTimer = millis();
      pressureMaxTimer = millis();
    }

    // Serial.println("razlika časa: ");
    // Serial.println(pressureTimerMin - pressureMinTimer);
  }
}


void temperatureProtection()
{
  if (tempPump > maxTempPump)
  {
    //Serial.println("Pump temperature too high! (over 80°C");
    Blynk.logEvent("pump_temperature_high");
  }

  if (tempCompressor > maxTempCompressor)
  {
   //Serial.println("Compressor temperature too high! (over 95°C");
    Blynk.logEvent("compressor_temperature_high");
  }
  if (tempCabinet > maxtempCabinet)
  {
    //Serial.println("Cabinet temperature too high ! (over 80°C");
    Blynk.logEvent("cabinet_temperature_high");
  }
}

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
    //Serial.println("MACHINE OFF FUNCTION");
    digitalWrite(PUMP_CONTACTOR, LOW);
    digitalWrite(OZONE_CONTACTOR, LOW);
    digitalWrite(OXYGEN_CONTACTOR, LOW);
    Blynk.virtualWrite(0, 0);
    Blynk.virtualWrite(1, 0);
    Blynk.virtualWrite(2, 0);
    Blynk.virtualWrite(17, 0); // Oxygen enabled LED
    Blynk.virtualWrite(18, 0); // Ozone enabled LED
    
}

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

// // Function to turn machine ON using Automations in pump+O3+O2 MODE
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

// // Function to turn machine ON using Automations in pump+O2 MODE
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
    
    digitalWrite(PUMP_CONTACTOR, HIGH);
    
    Blynk.virtualWrite(0, 1);
  } 
}


// /*ADDED 12_3_2022 by JS*/


int ozoneOnInterval = 10;               // Interval of ozonator ON in minutes
int ozoneOffInterval = 10;              // Interval of ozonator OFF in minutes

/* Choose right virtual pin number */
BLYNK_WRITE(V7)
{  
    
  ozoneOnInterval = param.asInt();
  ozoneOffInterval = param.asInt();
  
}
// /* Choose right virtual pin number */
// BLYNK_WRITE(V50)
// {
//   ozoneOffInterval = param.asInt();
  
// }

int ozState = 0; // State of ozonator switch case
unsigned long previousOzoneOnMillis = 0;  // Previous ozone millis when ozone generator was turned on
unsigned long previousOzoneOffMillis = 0; // Previous ozone millis when ozone generator was turned off

void ozonatorCycle()
{ 
  unsigned long currentMillis = millis();
  switch (ozState)
  {
  case 0:

    /* First step when ozonator is off */
    if (ozoneEnabled == true){
      /* If ozone is enabled go to state 10 */
      ozState = 10;
    }
    else if (ozoneEnabled == false){
      /* If ozone is disabled turn off O3 contactor */
      /* and go to step 1 */

      ozoneAfterDelayOn = false;
      ozoneOnDelay = false;
      previousOzoneOnMillis = 0;
      previousOzoneOffMillis = 0;

      ozState = 1;
    }
    break;

  case 1:

    /* Wait for ozonator to be enabled again */
    if (ozoneEnabled == true){
      ozState = 10;
      Serial.println("ozone enabled");
    }
    
    break;

  case 10:
    /* Ozone generator starting in 10 sec */
    startOzoneOnDelay = millis(); // Starting 10 sec counter
    ozState = 20;
    Serial.println("ozone timer on");
    break;

  case 20:
    /* Start ozone generator after on delay (10 sec) */
    if (currentMillis - startOzoneOnDelay >= ozoneDelay*1000){
      previousOzoneOnMillis = millis();
      ozState = 30;
      Serial.println("ozonator is on");
      Serial.println("ozone timers are: ");
      Serial.println(ozoneOnInterval);
      Serial.println(ozoneOffInterval);
    }
    else if(ozoneEnabled == false){
      ozState = 0;
    }
    break;
  
  case 30:
    /* Turn ozone generator ON for x minutes */
    if (currentMillis - previousOzoneOnMillis >= ozoneOnInterval*1000*60){
      previousOzoneOffMillis = millis();
      ozState = 40;
      Serial.println("ozonator is running");
    }
    else if(ozoneEnabled == false){
      ozState = 0;
    }
    break;

  case 40:
    /* Turn ozone generator OFF for x minutes */
    if (currentMillis - previousOzoneOffMillis >= ozoneOffInterval*1000*60){
      previousOzoneOnMillis = millis();
      ozState = 30;
      Serial.println("ozonator is turning off");
    }
    else if(ozoneEnabled == false){
      ozState = 0;
    }
    break;
  
  default:
    break;
  }
  
  if (ozState == 30){
      /* Turn on O3 generator if state is 30 */
      digitalWrite(OZONE_CONTACTOR, HIGH);
     Serial.println("ozonator deluje");
    }
    else{
      digitalWrite(OZONE_CONTACTOR, LOW);
      Serial.println("ozonator ne deluje");
    }
}
