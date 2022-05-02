#ifndef Config_h
#define Config_h

#include <Arduino.h>
#include "./src/ArduinoLog/ArduinoLog.h"
#include "./src/mavlink/ardupilotmega/mavlink.h"

// DEBUG MODE
//#define DEBUG_MODE // Comentar para desactivar debug

// Use 0-2. Larger for more debugging messages
#define FLASH_DEBUG       0

// SERIAL BAUDS
#define SERIAL_BAUD 57600
#define SERIAL_BAUD_TELEM 57600

// COND VALUES (PARAMS)
#define COND_ALTITUDE 10 //cm
#define DISTANCE_SENSOR_FILTER_COUNT 2 // tries
#define LANDING_ALTITUDE 50 // from where conditionals works
#define ALTITUDE_DIFF 200 //cm Max difference allowed between relative altitude and distance sensor to disarm. If set to 0 relative alttitude is ignored.


// MAVLINK

// This Device
#define SYSID 1
#define COMPID 191

// Target Device
#define TARGET_SYSID 1
#define TARGET_COMPID 1

// GCS
#define GCS_SYSID 255
#define GCS_COMPID 190

// Intervals
#define HEARTBEAT_INTERVAL 1000 //ms 1 vez por segundo
#define DISTANCE_SENSOR_INTERVAL 200000
#define EXTENDED_SYS_STATE_INTERVAL 200000
#define ALTITUDE_INTERVAL 200000

// Time to lost link
#define LOST_TIME 5 //s

// Virtual EEPROM
const int WRITTEN_SIGNATURE = 0xBEEFDEED;
const int START_ADDRESS     = 0;

#endif
