#include "Modbus_func.h"
// #include <Arduino.h>
#define EXTERN extern
#include "variables.h"
// #include "constants.h"

bool getResultMsg(ModbusMaster *modbus, uint8_t result) 
{ 
  String tmpstr2 = "\r\n";
  switch (result) 
  {
  case modbus->ku8MBSuccess:
    return true;
    break;
  case modbus->ku8MBIllegalFunction:
    tmpstr2 += "Illegal Function";
    break;
  case modbus->ku8MBIllegalDataAddress:
    tmpstr2 += "Illegal Data Address";
    break;
  case modbus->ku8MBIllegalDataValue:
    tmpstr2 += "Illegal Data Value";
    break;
  case modbus->ku8MBSlaveDeviceFailure:
    tmpstr2 += "Slave Device Failure";
    break;
  case modbus->ku8MBInvalidSlaveID:
    tmpstr2 += "Invalid Slave ID";
    break;
  case modbus->ku8MBInvalidFunction:
    tmpstr2 += "Invalid Function";
    break;
  case modbus->ku8MBResponseTimedOut:
    tmpstr2 += "Response Timed Out";
    break;
  case modbus->ku8MBInvalidCRC:
    tmpstr2 += "Invalid CRC";
    break;
  default:
    tmpstr2 += "Unknown error: " + String(result);
    break;
  }

  // Print Modbus returned message
  // Serial.println(tmpstr2);
  return false;
}
