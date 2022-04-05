#ifndef MODBUS_FUNC_H
#define MODBUS_FUNC_H
#include <Arduino.h>
#define EXTERN extern
#include "variables.h"

bool getResultMsg(ModbusMaster *modbus, uint8_t result);

#endif
