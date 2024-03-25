#include "DO_reg_func.h"
#include "Modbus_func.h"
#include <ArduinoRS485.h> 
#include <ArduinoModbus.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <RTCZero.h>
#include <iostream>
#include <cmath>

#define EXTERN extern
// #include "constants.h"

// Funkcija za pridobitev podatkov iz DO senzorja
// void readDO(float &doValue, float &doSatValue, float &doTempValue){
//   //Serial.println("Reading Modbus registers");
//   uint16_t do_data_arr[8];
//   modbus.writeSingleRegister(0x0001, 31);
  
//   delay(200);
//   // Pridobimo odgovor DO senzorja
//   // Beremo registre od 0x0053, 8 registrov
//   uint8_t result = modbus.readHoldingRegisters(0x0053, 8);
//   //Serial.println(result);
//   //uint8_t result = modbus.readHoldingRegisters(164, 2);

//     // Ce je odgovor OK, zapisemo vrednosti registrov v array data[]
//     if (getResultMsg(&modbus, result)) 
//     {
//       for (uint8_t j = 0; j < 8; j++)
//       {
//         do_data_arr[j] = modbus.getResponseBuffer(j);
//       }

//     // DO Temperature
//     float resultDoTemp;
//     uint16_t regs[2] = {do_data_arr[0], do_data_arr[1]};    //reg[0] & reg[1] contain the 16 bit ints
//     long int tempDo = (((unsigned long)regs[0] << 16) | regs[1]);
//     memcpy(&resultDoTemp, &tempDo, 4); // Temp data from DO sensor
//     doTempValue = resultDoTemp;
//     Serial.print("T: ");
//     Serial.print(doTempValue);
//     Serial.println(" °C");
//     //Serial.println("Prvi del: " + String(do_data_arr[0]) + ", drugi del: " + String(do_data_arr[1]));
    

//     // DO saturation
//     float resultDoSat;
//     uint16_t regsDoSat[2] = {do_data_arr[2], do_data_arr[3]};    //reg[0] & reg[1] contain the 16 bit ints
//     long int doSat = (((unsigned long)regsDoSat[0] << 16) | regsDoSat[1]);
//     memcpy(&resultDoSat, &doSat, 4); // DO saturation data from DO sensor
//     doSatValue = resultDoSat;
//     Serial.print("DO SAT: ");
//     Serial.print(doSatValue);
//     Serial.println(" %");

//     // DO in mg/L
//     float resultDO;
//     uint16_t regsDo[2] = {do_data_arr[4], do_data_arr[5]};    //reg[0] & reg[1] contain the 16 bit ints
//     long int valueDo = (((unsigned long)regsDo[0] << 16) | regsDo[1]);
//     memcpy(&resultDO, &valueDo, 4); // DO data from DO sensor
//     doValue = resultDO;
//     Serial.print("DO: ");
//     Serial.print(doValue);
//     Serial.println(" mg/L");
//     //Serial.println();
//     }

// }

double calculateDOSaturation(double temperature) {
    // Constants for the Weiss equation
    const double A1 = -173.4292;
    const double A2 = 249.6339;
    const double A3 = 143.3483;
    const double A4 = -21.8492;

    // Convert temperature from Celsius to Kelvin
    double tempK = temperature + 273.15;

    // Calculate saturation concentration of oxygen
    return exp(A1 + A2 * (100 / tempK) + A3 * log(tempK / 100) + A4 * pow(tempK / 100, 2));
}

double array[3];

void readDO(float &doValue, float &doSatValue, float &doTempValue){
  if (!ModbusRTUClient.requestFrom(1, HOLDING_REGISTERS, 0x2001, 3)) {
    // Serial.print("failed! ");
    Serial.println(ModbusRTUClient.lastError());
  } else {
    // Serial.println("success");

    int index = 0;  // Index to track array elements

    while (ModbusRTUClient.available()) {
      double value = ModbusRTUClient.read();  // Read the value and convert it
      array[index] = value;  // Save the value into the array
      index++;  // Increment the index for the next element
    }
    doValue = array[0] * 0.01;
    doTempValue = array[2] * 0.1;
    double doSaturation = calculateDOSaturation(doTempValue);
    doSatValue = (doValue/doSaturation) * 100;
  }
}

bool firstIterationDo = true;
unsigned long currentMillisDo;
unsigned long previousMillisDo1 = 0;
int intervalDo = 1; 
bool preverjanjeDo = false;
int stevecDo = 0;
float previousValueDo = 0;
int minimumSpremembaDo;
int numberOfOverDo = 0;

void regulacija_O2(){

  float doValue, doSat, doTemp;
  readDO(doValue, doSat, doTemp);
  casovnik_pumpa();
  casovnik_ugas();
  rezim_dol();
  
  if(reg_enable == true){

    Serial.println("reg enable true");
    if((regulacija_min < doValue) && (regulacija_max > doValue) && (potek_dol == false) && (potek_gor == false)){
      rezim_dol1 = true;

      // set variables for checking doValue

      stevecDo = 0;
      preverjanjeDo = false;
      firstIterationDo = true;
    }
    if(regulacija_min > doValue){
      potek_gor = true;
      potek_dol = false;

      // set variables for checking doValue
      preverjanjeDo = true;
      firstIterationDo = false;

      if(pumpOn == false){
        pumpOn = true;
        oxygenEnabled = true;
        digitalWrite(PUMP_CONTACTOR,HIGH);
      }
      else if(pumpOn == true){
        oxygenEnabled = true;
      }

      rezim_dol1 = false;
      casovnik = false;
      casovnik_off = false;
    }
    if(regulacija_max < doValue){
      rezim_dol1 = true;
      potek_dol = true;
      potek_gor = false;

      // set variables for checking doValue
      preverjanjeDo = false;
      stevecDo = 0;
      firstIterationDo = true;
    }
  }
}

void casovnik_pumpa(){

  if(casovnik == true){
    stevec_pumpa = stevec_pumpa + 1;

    if(stevec_pumpa == 900){
      stevec_pumpa = 0;
      pumpOn = true;
      digitalWrite(PUMP_CONTACTOR,HIGH);
      casovnik_off = true;
    }
  }
  else if(casovnik == false) stevec_pumpa = 0;
}

void casovnik_ugas(){

  if(casovnik_off == true){
    stevec_off = stevec_off + 1;
  }
  else if(casovnik_off == false) stevec_off = 0;
  if(stevec_off == 60){
    stevec_off = 0;
    Pumpa_ugas = true;
    casovnik_off = false;
  }
}

void rezim_dol(){
  
  if(rezim_dol1 == true){
    oxygenEnabled = false;
    casovnik = true;
    if(prva_iteracija == true){
      Pumpa_ugas = true;
      prva_iteracija = false;
    }
    if(Pumpa_ugas == true){
      digitalWrite(PUMP_CONTACTOR,LOW);
      pumpOn = false;
      Pumpa_ugas = false;
   }
 }
  else if(rezim_dol1 == false){
    casovnik = false;
    casovnik_off = false;
    prva_iteracija = true;
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
     // Serial.println("----Oxygen counter started----");
      startOxygenOnDelay = millis();
      oxygenOnDelay = true;
    }
    else if ((currentMillis - startOxygenOnDelay >= oxygenDelay*1000) && oxygenOnDelay)
    {
      oxygenOnDelay = false;
     // Serial.println("Oxygen ON after delay.");
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