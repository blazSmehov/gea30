#ifndef MACHINE_FUNC_H
#define MACHINE_FUNC_H
#include <Arduino.h>
#define EXTERN extern
#include "variables.h"
#include <SPI.h>

void orpSensor();
void postTransmission();
void preTransmission();

float calculatePower(double acVoltage);
float readACCurrentValue();
float pSensor();

long dBmtoPercentage(long dBm);

double avergearray(int* arr, int number);

#endif
