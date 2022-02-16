#ifndef MACHINE_FUNC_H
#define MACHINE_FUNC_H
#include <Arduino.h>
#define EXTERN extern
#include "variables.h"
#include <BlynkSimpleWiFiNINA.h>
#include <SPI.h>

void powerProtection();
void temperatureProtection();
void machineShutdown();
void machineSafetyShutdown();
void machineOnAuto();
void machineOnAutoOzone();
void machineOnAutoOxygen();
void machineOn();
void autoManMode();
void wifiStrength();
void orpSensor();
void checkContactorState();
void outputsWhenConnected();
void postTransmission();
void preTransmission();

float calculatePower(double acVoltage);
float readACCurrentValue();
float pSensor();

long dBmtoPercentage(long dBm);

double avergearray(int* arr, int number);

#endif
