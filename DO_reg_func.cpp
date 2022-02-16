#include "DO_reg_func.h"
#include "Modbus_func.h"
#define EXTERN extern
#include "constants.h"

// Funkcija za pridobitev podatkov iz DO senzorja
void readDO(float &doValue, float &doSatValue, float &doTempValue){
  //Serial.println("Reading Modbus registers");
  uint16_t do_data_arr[8];
  modbus.writeSingleRegister(0x0001, 31);
  
  delay(200);
  // Pridobimo odgovor DO senzorja
  // Beremo registre od 0x0053, 8 registrov
  uint8_t result = modbus.readHoldingRegisters(0x0053, 8);
  //Serial.println(result);
  //uint8_t result = modbus.readHoldingRegisters(164, 2);

    // Ce je odgovor OK, zapisemo vrednosti registrov v array data[]
    if (getResultMsg(&modbus, result)) 
    {
      for (uint8_t j = 0; j < 8; j++)
      {
        do_data_arr[j] = modbus.getResponseBuffer(j);
      }

    // DO Temperature
    float resultDoTemp;
    uint16_t regs[2] = {do_data_arr[0], do_data_arr[1]};    //reg[0] & reg[1] contain the 16 bit ints
    long int tempDo = (((unsigned long)regs[0] << 16) | regs[1]);
    memcpy(&resultDoTemp, &tempDo, 4); // Temp data from DO sensor
    doTempValue = resultDoTemp;
    Serial.print("T: ");
    Serial.print(doTempValue);
    Serial.println(" °C");
    //Serial.println("Prvi del: " + String(do_data_arr[0]) + ", drugi del: " + String(do_data_arr[1]));
    

    // DO saturation
    float resultDoSat;
    uint16_t regsDoSat[2] = {do_data_arr[2], do_data_arr[3]};    //reg[0] & reg[1] contain the 16 bit ints
    long int doSat = (((unsigned long)regsDoSat[0] << 16) | regsDoSat[1]);
    memcpy(&resultDoSat, &doSat, 4); // DO saturation data from DO sensor
    doSatValue = resultDoSat;
    Serial.print("DO SAT: ");
    Serial.print(doSatValue);
    Serial.println(" %");

    // DO in mg/L
    float resultDO;
    uint16_t regsDo[2] = {do_data_arr[4], do_data_arr[5]};    //reg[0] & reg[1] contain the 16 bit ints
    long int valueDo = (((unsigned long)regsDo[0] << 16) | regsDo[1]);
    memcpy(&resultDO, &valueDo, 4); // DO data from DO sensor
    doValue = resultDO;
    Serial.print("DO: ");
    Serial.print(doValue);
    Serial.println(" mg/L");
    //Serial.println();
    }

}

void regulacija_O2(){

  casovnik_pumpa();
  casovnik_ugas();
  rezim_dol();
  
  if(reg_enable == true){

    float doValue, doSat, doTemp;
    readDO(doValue, doSat, doTemp);

    if((regulacija_min < doValue) && (regulacija_max > doValue) && (potek_dol == false) && (potek_gor == false)){
      rezim_dol1 = true;
    }
    if(regulacija_min > doValue){
      potek_gor = true;
      potek_dol = false;
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

void ozonatorCycle()
{
  if (ozoneEnabled == true)
  { 
    unsigned long currentMillis = millis();

    // When you turn on Ozone generator there is some delay (cca 10s)
    if (!ozoneOnDelay && !ozoneAfterDelayOn){
      //Serial.println("----Ozone 10s counter started----");
      startOzoneOnDelay = millis();
      ozoneOnDelay = true;
    }
    else if ((currentMillis - startOzoneOnDelay >= ozoneDelay*1000) && ozoneOnDelay)
    {
      ozoneOnDelay = false;
      //Serial.println("Ozone ON after delay.");
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
          //Serial.println("Generating ozone");
        } 
        else if (ozoneState == HIGH) 
        {
          prevOzoneState = ozoneState;
          ozoneState = LOW;
          //Serial.println("Not generating O3 - stand by...");
        }
        
        if (ozoneState != prevOzoneState)
        {
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
