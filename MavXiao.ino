#include "./src/FlashStorage_SAMD/src/FlashStorage_SAMD.h"
#include "Config.h"
#include "Telem.h"

// Init mav
Telem mav(Serial1);

// Distance sensor counter
int ds_values_count = 0;

// Condition vars
int cond_flight_modes[] = { 
    PLANE_MODE_AUTO, 
    PLANE_MODE_RTL, 
    PLANE_MODE_QLAND, 
    PLANE_MODE_QRTL,
    PLANE_MODE_QLOITER,
    PLANE_MODE_QHOVER,
    PLANE_MODE_QSTABILIZE
};

uint8_t cond_mode = 0;
uint8_t cond_armed = 0;
uint8_t cond_alt = 0;
uint8_t cond_landed_state = 0;
unsigned long last_noticed = 0;

enum AC_STATE {
  AC_LANDED = 0,
  AC_TAKING_OFF,
  AC_FLYING,
  AC_LANDING
};

uint8_t ac_state = AC_LANDED;

int eeAddress;



void setup()
{

    // initialize digital pin LED_BUILTIN as an output.
    pinMode(LED_BUILTIN, OUTPUT);

    // Serial Monitor ang Log
    Serial.begin(SERIAL_BAUD);

    // Esperamos Serial UP
    delay(3000);

    // Log helper
#ifndef DEBUG_MODE
    Log.begin(LOG_LEVEL_SILENT, &Serial);
#else
    Log.begin(LOG_LEVEL_VERBOSE, &Serial);
#endif

    // Serial Mavlink Begin
    mav.begin();

    int signature;
    int eeAddress;

    EEPROM.setCommitASAP(false);

    EEPROM.get(START_ADDRESS, signature);

    // If the EEPROM doesn't store valid data, then no WRITTEN_SIGNATURE
    if (signature != WRITTEN_SIGNATURE) {
        Log.notice("Sin Datos en eeprom" CR);

        // Para guardar estado escritura
        EEPROM.put(START_ADDRESS, WRITTEN_SIGNATURE);

        Log.notice("Guardamos parametros por defecto en memoria" CR);

        // Nos desplazamos hasta la siguiente dirección libre de memoria
        eeAddress = START_ADDRESS + sizeof(WRITTEN_SIGNATURE);

        // Recorremos y añadimos parámetros
        for (auto& param : mav.paramsList) {
            Log.notice(F("put %s" CR), param.param_id);
            EEPROM.put(eeAddress, param);
            eeAddress += sizeof(param);
        }

        // Persistimos
        if (!EEPROM.getCommitASAP())
        {
          Serial.println("Persistiendo datos");
          EEPROM.commit();
        }

    } else {

        Log.notice("Cargamos datos desde EEPROM" CR);

        // Nos desplazamos hasta la primera posición
        eeAddress = START_ADDRESS + sizeof(WRITTEN_SIGNATURE);

        for (mavlink_param_value_t &param : mav.paramsList) {
            EEPROM.get(eeAddress, param);
            eeAddress += sizeof(param);
        }

    }
}

void loop()
{

    // Run mav
    mav.run(msgRecivedCallback);


    // EVALUATE DISARM ACTION
    if (cond_armed == 1 && cond_mode == 1 && cond_alt == 1 && cond_landed_state == 1) {
        //Disarm
        mav.armDisarm();

        cond_armed = 0;
    }

    // Máquina de estados
    if (millis() - last_noticed >= 100) {
        
        last_noticed = millis();

        if (ac_state == AC_LANDED && cond_armed == 0 && cond_alt == 1) {
          cond_landed_state = 0;
        } else if (ac_state == AC_LANDED && cond_armed == 1 && cond_alt == 0) {
          ac_state = AC_TAKING_OFF;
        } else if (ac_state == AC_TAKING_OFF && cond_armed == 1 && cond_alt == 0 ) {
          ac_state = AC_FLYING;
        } else if (ac_state == AC_FLYING && cond_armed == 1 && cond_alt == 0 && mav.APdata.distance_sensor <= int(mav.paramsList[2].param_value)) {
          ac_state = AC_LANDING;
        } else if (ac_state == AC_LANDING && cond_armed == 1 && cond_alt == 0 && mav.APdata.distance_sensor > int(mav.paramsList[2].param_value)) {
          ac_state = AC_FLYING;
        } else if (ac_state == AC_LANDING && cond_armed == 1 && cond_alt == 1) {
          ac_state = AC_LANDED;
          if  (cond_mode == 1)
            cond_landed_state = 1; // we can disarm now
        }

        Log.notice(F("ac_state: %i cond_landed_state: %i armed: %i mode: %i alt: %i rangefinder: %i" CR), ac_state, cond_landed_state, cond_armed, cond_mode, cond_alt, mav.APdata.distance_sensor);// mav.APdata.landed_state , mav.APdata.custom_mode, mav.APdata.armed, mav.APdata.distance_sensor, cond_mode, cond_armed, cond_alt, ds_values_count);

    }

    // Output
    if (mav.link) {
        //Log.notice(F("Mode: %i Armed: %T Range Finder : %icms Cond Mode: %i Cond Armed: %i Cond Range Finder: %i Filter Count: %i"CR) , mav.APdata.custom_mode, mav.APdata.armed, mav.APdata.distance_sensor, cond_mode, cond_armed, cond_alt, ds_values_count);
        //Log.notice(F("Landed State: %i " CR) , mav.APdata.landed_state);
    }
}

// --------------- Disparador segun msg recivido ---------------------- //
void msgRecivedCallback(mavlink_message_t msg)
{
    switch (msg.msgid) {

    case MAVLINK_MSG_ID_PARAM_SET: // #23: SET PARAM

        mavlink_param_set_t _param_set;
        mavlink_msg_param_set_decode(&msg, &_param_set);

        // Buscamos param a cambiar
        for (mavlink_param_value_t& param : mav.paramsList) {

            // Si lo tenemos, modificamos y respondemos
            if (String(param.param_id) == String(_param_set.param_id)) {
                // Param set
                param.param_value = _param_set.param_value;

                // Obtenemos su posición en memoria
                int eeAddress = START_ADDRESS + sizeof(WRITTEN_SIGNATURE) + (param.param_index * 28);
                
                // Actualizamos parámetro
                EEPROM.put(eeAddress, param);
                
                // Persistimos
                if (!EEPROM.getCommitASAP())
                {
                  Serial.println("Persistiendo datos");
                  EEPROM.commit();
                }                

                // Respuesta para MP
                mav.param_value(param);

                break;
            }
        }

        break;

    case MAVLINK_MSG_ID_HEARTBEAT:

        // CHECK COND Mode
        for (byte i = 0; i < sizeof(cond_flight_modes) / sizeof(cond_flight_modes[0]); i++) {
            if (cond_flight_modes[i] == mav.APdata.custom_mode) {
                cond_mode = 1;
                break;
            } else {
                cond_mode = 0;
            }
        }

        // CHECK COND Armed
        cond_armed = (mav.APdata.armed) ? 1 : 0;

        break;

    case MAVLINK_MSG_ID_DISTANCE_SENSOR:
    case MAVLINK_MSG_ID_ALTITUDE:

        // CHECK COND Alt
        if (mav.APdata.distance_sensor < int(mav.paramsList[0].param_value)) {
            if (ds_values_count >= int(mav.paramsList[1].param_value)) {
                setCondAlt(1);
                ds_values_count = int(mav.paramsList[1].param_value);
            } else {
                ds_values_count++;
            }
        } else {
            setCondAlt(0);
            ds_values_count = 0;
        }

        break;

    }
}

void setCondAlt(uint8_t value) {
  if (value == 0) 
    cond_alt = 0;
  else if(mav.paramsList[3].param_value > 0 && (mav.APdata.altitude - mav.APdata.distance_sensor >= mav.paramsList[3].param_value)) {
      cond_alt = 0;
  } else
      cond_alt = 1;
}
