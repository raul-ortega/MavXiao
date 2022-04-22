#ifndef Telem_h
#define Telem_h

#include "Config.h"

typedef struct
{
  uint32_t custom_mode;
  uint8_t base_mode;
  boolean armed;
  uint16_t distance_sensor;
  int16_t altitude;
  uint8_t landed_state; 
} APdata_t;

class Telem
{
public:
  Telem(HardwareSerial &hs);
  void begin();
  void run(void (*msgRecivedCallback)(mavlink_message_t msg));
  void armDisarm();
  void heartbeat();
  void status_text(char * text);
  void param_value(mavlink_param_value_t param_value);
  void request_data_streams(uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop);
  void request_distance_sensor();
  void request_altitude();

  // Parámetros iniciales  
  mavlink_param_value_t cond_dis_sen = { COND_ALTITUDE, 3, 0, "COND_DIST_SEN", MAV_PARAM_TYPE_REAL32};
  mavlink_param_value_t dis_sen_cont = { DISTANCE_SENSOR_FILTER_COUNT, 3, 1, "DIST_SENS_CONT", MAV_PARAM_TYPE_REAL32};
  mavlink_param_value_t landing_alt = { LANDING_ALTITUDE, 3, 2, "LANDING_ALT", MAV_PARAM_TYPE_REAL32};

  // Lista de parámetros en memoria
  mavlink_param_value_t paramsList[3];

  APdata_t APdata;
  boolean link;
  boolean led_status;

private:
  HardwareSerial *_MAVSerial;
  void check_link();

  uint8_t system_id;
  uint8_t component_id;
  uint8_t target_sysid;  // Target sysid
  uint8_t target_compid; // Target compid

  unsigned long time_heartbeat;
  unsigned long last_heatbeat;
};
#endif
