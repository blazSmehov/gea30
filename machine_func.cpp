#include "Modbus_func.h"
#include "machine_func.h"
#include <Arduino.h>
#define EXTERN extern
#include "variables.h"
#include "constants.h"

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
  Serial.println("vrednost napetosti: ");
  Serial.println(peakVoltage);
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
    else if (pumpState == 1 && appMode == 2 && autoCombination == 1 && (statusSwitch != "** AUTO - BUBBLING O3 **")&& (statusSwitch == "*** AUTO - STAND BY ***"))
    {
      Blynk.virtualWrite(3, "** AUTO - BUBBLING O3 **");
    }
    else if (pumpState == 1 && appMode == 2 && autoCombination == 2 && (statusSwitch != "** AUTO - BUBBLING O2 **")&& (statusSwitch == "*** AUTO - STAND BY ***"))
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
float pSensor(){
  const float  OffSet = 0.2997;
  float V, P;
  V = (analogRead(PRESSURE_PIN)*3.3)/1024;     //Sensor output voltage V = analogRead(17) * 5.00 / 1024;
  P = ((V - OffSet)*606.06)/100;             //Calculate water pressure

  //Serial.print("Pressure:");
  //Serial.print(P, 1);
  //Serial.println(" Bar");
  //Serial.println();

  return P;
  
}

/* ORP SENSOR */

double avergearray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    //printf("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5){   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
}

// Dodaj blynk.write in posodobi template z vrednostjo ORP
void orpSensor(){
  static unsigned long orpTimer=millis();   //analog sampling interval
  static unsigned long printTime=millis();
  if(millis() >= orpTimer)
  {
    orpTimer=millis()+20;
    orpArray[orpArrayIndex++]=analogRead(ORP_PIN);    //read an analog value every 20ms
    if (orpArrayIndex==ArrayLenth) {
      orpArrayIndex=0;
    }
    orpValue=((30*(double)VOLTAGE*1000)-(75*avergearray(orpArray, ArrayLenth)*VOLTAGE*1000/1024))/75-offsetVoltage;
    //convert the analog value to orp according the circuit
  }
  if(millis() >= printTime)   //Every 1000 milliseconds, print a numerical, convert the state of the LED indicator
  {
    printTime=millis()+1000;
  }
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

void outputsWhenConnected()
{
  /* Send contactor values when Blynk is connected */
  Blynk.virtualWrite(9, digitalRead(PUMP_CONTACTOR));
  Blynk.virtualWrite(10, digitalRead(OXYGEN_CONTACTOR));
  Blynk.virtualWrite(8, digitalRead(OZONE_CONTACTOR));
}


void postTransmission()
{
  digitalWrite(RS_EN, 0);
}

// Pred oddajo Modbus senzorja postavimo RS_EN na 1
void preTransmission()
{
  digitalWrite(RS_EN, 1);
}

// Po oddaji Modbus senzorja postavimo RS_EN na 0
