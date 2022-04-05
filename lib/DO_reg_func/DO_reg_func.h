#ifndef DO_REG_FUNC_H
#define DO_REG_FUNC_H
#include <Arduino.h>
#define EXTERN extern
#include "variables.h"

void readDO(float &doValue, float &doSatValue, float &doTempValue);
void regulacija_O2();
void casovnik_pumpa();
void casovnik_ugas();
void rezim_dol();
// void ozonatorCycle();
void oxygenWithDelay();

#endif
