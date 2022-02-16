// Popravljeni temperaturni senzorji
// Masina IoT nova Italija z ORP
#define BLYNK_FIRMWARE_VERSION        "0.1.27"

#define BLYNK_PRINT Serial
//#define BLYNK_DEBUG

#define APP_DEBUG

#define ONE_WIRE_BUS 0

#define AUTO 18
#define MAN 19
#define FLOAT_SWITCH 20
#define BOARD_BUTTON 6

#define AC_PIN A1
#define ACTectionRange 20;  //set Non-invasive AC Current Sensor tection range (5A,10A,20A)
#define VREF 3.3

#define PUMP_CONTACTOR 3
#define OXYGEN_CONTACTOR 2
#define OZONE_CONTACTOR 1
#define RESERVE_CONTACTOR 4

#define ONE_WIRE_BUS 0 // Data wire is plugged into digital pin 0 on the Arduino
/* HARDWARE DEFINES - END */
// Setup a oneWire instance to communicate with any OneWire device

#define PRESSURE_PIN A0
//#endif

// MODBUS DO SENSOR
#define RS_RX 13
#define RS_TX 14
#define RS_EN 21

#define Slave_ID  10

#define ORP_PIN A2
#define VOLTAGE 3.3    //system voltage
#define ArrayLenth  40    //times of collection
