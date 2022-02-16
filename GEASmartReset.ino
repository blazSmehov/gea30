#define BLYNK_TEMPLATE_ID "TMPL60vVWn3M"
#define BLYNK_DEVICE_NAME "GEASmartCopy"
#define BLYNK_AUTH_TOKEN "H_z1Ec4na8_arDdTANW897tIBnUaRmEn"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RTCZero.h>
#include <ModbusMaster.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <BlynkSimpleWiFiNINA.h>
BlynkWifiCommon Blynk(_blynkTransport);
#include "DO_reg_func.h"
#include "machine_func.h"
#include "Modbus_func.h"
#define EXTERN extern
#include "constants.h"
#include "variables.h"

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

unsigned long previousTime1 = 0;

//git comment

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

// ON/OFF button in app for oxygen enabled/disabled    
BLYNK_WRITE(V1)
{
  // If pump is working, you can turn O and O3 generators ON
  if (pumpOn == true && machineMode == 1 && appMode == 0)
  {
  
    if (param.asInt() == 1 )
    {
      oxygenEnabled = true;
      Blynk.virtualWrite(17, 1); // Oxygen enabled
      // Serial.println("Oxygen MAN ON!");
      
    }
    else if (param.asInt() == 0)
    {
      oxygenEnabled = false; // ???
      Blynk.virtualWrite(17, 0); // Oxygen disabled
      digitalWrite(OXYGEN_CONTACTOR, LOW);
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
    
// ON/OFF button in app for ozone enabled/disabled    
BLYNK_WRITE(V2)
{  
  
  if (pumpOn == true && machineMode == 1 && appMode == 0)
  {
    if (param.asInt() == 1)
    {
      ozoneEnabled = true;
      Blynk.virtualWrite(18, 1); // Ozone enabled
      // Serial.println("Ozone ON!");
    }
    else if (param.asInt() == 0)
    {
      ozoneEnabled = false;
      previousOzoneMillis = 0;
      Blynk.virtualWrite(2, 0);   // Ozone button OFF
      Blynk.virtualWrite(18, 0); // Ozone disabled
      // Serial.println("Ozone OFF");
    }
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
      //digitalWrite(PUMP_CONTACTOR, LOW);
      //digitalWrite(OZONE_CONTACTOR, LOW);
      //digitalWrite(OXYGEN_CONTACTOR, LOW);
      rezim_dol1 = false;
      // Serial.println("Problem2");
      reg_enable = false;
      potek_dol = false;
      potek_gor = false;
      break;
    }
    case 2: {
      // Serial.println("Mode selection: TIMER");
      appMode = 2;
      machineShutdown(); 
      // Serial.println("Problem3");
      rezim_dol1 = false; 
      reg_enable = false;
      potek_dol = false;
      potek_gor = false;
      break;
    }    
    case 3: {
      // Serial.println("Mode selection: REG");
      appMode = 3;
      // Serial.println("Problem4");
      reg_enable = true;
      Pumpa_ugas = true;
      break;
    }
  }
}

// Combination when machine is working with timers
BLYNK_WRITE(V16){
  switch (param.asInt()) {
    case 0:
      //Serial.println("Auto combination: Pump + O3 + O2");
      autoCombination = 0; // Pump + O3 + O2
      break;
    case 1: {
      //Serial.println("Auto combination: Pump + O3");
      autoCombination = 1; // Pump + O3
      break;
    }
    case 2: {
      //Serial.println("Auto combination: Pump + O2");
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
  //Blynk.syncVirtual(V3);
  //autoManMode();
  Blynk.syncVirtual(V35);
  Blynk.syncVirtual(V36);
  outputsWhenConnected();
  checkContactorState();
  sensorTimer.setInterval(711L, autoManMode);
  sensorTimer.setInterval(1023L, checkContactorState);
  sensorTimer.setInterval(1235L, oxygenWithDelay);
  sensorTimer.setInterval(1522L, ozonatorCycle);
  sensorTimer.setInterval(3109L, checkTime);
  sensorTimer.setInterval(6359L, callDoSensor);
  sensorTimer.setInterval(1000L, regulacija_O2);
  sensorTimer.setInterval(10307L, sendSensorValues);
  sensorTimer.setInterval(12018L, powerProtection);
  sensorTimer.setInterval(15099L, temperatureProtection);
//  sensorTimer.setInterval(1000L, reset_wifi);
}

void setup()
{
  bool reg_enable = true;
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

  modbus.begin(Slave_ID, Serial1);
  modbus.idle(yield);

  // Postavimo RS_EN na 0
  modbus.preTransmission(preTransmission);
  modbus.postTransmission(postTransmission);

  WiFi.begin(ssid, pass);  // Non-blocking if no WiFi available
  Blynk.config(BLYNK_AUTH_TOKEN, server, port);
  Blynk.connect();

  Blynk.virtualWrite(10, 0);
  Blynk.virtualWrite(9, 0);
  Blynk.virtualWrite(8, 0);
}

void loop() {
  
  sensorTimer.run();
  
  if (Blynk.connected()){
    Blynk.run();
    ReCnctCount = 0;
  }

//  else if (ReCnctCount < 3){
  else if (ReCnctFlag == 0) { 
  ReCnctFlag = 1;  
  Serial.println("Starting reconnection timer in 5 seconds...");
  sensorTimer.setTimeout(5000L, []() { 
    ReCnctFlag = 0; 
    ReCnctCount++;  
    Serial.print("Attempting reconnection #");
    Serial.println(ReCnctCount);
    WiFi.begin(ssid, pass);
    Blynk.config(BLYNK_AUTH_TOKEN, server, port);
    Blynk.disconnect();
    Blynk.connect();
    });
  }
//  }

//  else if (ReCnctCount == 3){
//    startTimer = true;
//  }
  
  if (blynkConnected == 1){
    orpSensor();
  }
  
  else if ((WiFi.status() != WL_CONNECTED)&& blynkConnected == 1){
    //Serial.println("Blynk diconnected");
    blynkConnected = 0;
  }

  unsigned long currentTime1 = millis();

  if((currentTime1 - previousTime1 >= 5000)&&(Blynk.connected())){
    Serial.println("povezano :)");
    previousTime1 = currentTime1;
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
  Blynk.virtualWrite(6, tempCompressor); // Prev. tempCompressor
  Blynk.virtualWrite(14, tempCabinet);
  Blynk.virtualWrite(33, pressure);
  Blynk.virtualWrite(34, orpValue);

}

void callDoSensor(){
  float doValue, doSat, doTemp;
  if ( blynkConnected == 1){
    readDO(doValue, doSat, doTemp);
    
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

//void reset_wifi(){
//
//  if (startTimer == true){
//    if(stevec_wifi == 0) Serial.println("delovanje brez wifija :(");
//    stevec_wifi = stevec_wifi + 1;
//    if (stevec_wifi > 1800){
//      ReCnctCount = 0;
//      stevec_wifi = 0;
//      startTimer = false;
//    }
//  }
//}
