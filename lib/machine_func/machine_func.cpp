#include "Modbus_func.h"
#include "machine_func.h"
// #include <Arduino.h>
#define EXTERN extern
#include "variables.h"
// #include "constants.h"

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

/* EXTRA FUNCTIONS */
float pSensor(){
  const float  OffSet = 0.2997;
  float V, P;
  V = (analogRead(PRESSURE_PIN)*3.3)/1024;     //Sensor output voltage V = analogRead(17) * 5.00 / 1024;
  P = ((V - OffSet)*606.06)/100;             //Calculate water pressure

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

void postTransmission()
{
  digitalWrite(RS_EN, 0);
}

// Pred oddajo Modbus senzorja postavimo RS_EN na 1
void preTransmission()
{
  digitalWrite(RS_EN, 1);
}

// ?Po oddaji Modbus senzorja postavimo RS_EN na 0
