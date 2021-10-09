#include "./src/FlashStorage_SAMD/src/FlashStorage_SAMD.h"
#include "Config.h"
#include "Telem.h"

// Init mav
Telem mav(Serial1);

// Distance sensor counter
int ds_values_count = 0;

// Condition vars
int cond_flight_modes[] = { PLANE_MODE_AUTO, PLANE_MODE_RTL, PLANE_MODE_QLAND, PLANE_MODE_QRTL };
uint8_t cond_mode = 0;
uint8_t cond_armed = 0;
uint8_t cond_alt = 0;

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
    if (cond_armed == 1 && cond_mode == 1 && cond_alt == 1) {
        // Disarm
        mav.armDisarm();

        cond_armed = 0;
    }

    // Output
    if (mav.link) {
        Log.notice(F("Mode: %i Armed: %T Range Finder : %icms Cond Mode: %i Cond Armed: %i Cond Range Finder: %i Filter Count: %i"CR) , mav.APdata.custom_mode, mav.APdata.armed, mav.APdata.distance_sensor, cond_mode, cond_armed, cond_alt, ds_values_count);
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

        // CHECK COND Alt
        if (mav.APdata.distance_sensor < int(mav.paramsList[0].param_value)) {
            if (ds_values_count >= int(mav.paramsList[1].param_value)) {
                cond_alt = 1;
                ds_values_count = int(mav.paramsList[1].param_value);
            } else {
                ds_values_count++;
            }
        } else {
            cond_alt = 0;
            ds_values_count = 0;
        }

        break;
    }
}

// void printMyObject(mavlink_param_value_t& customVar)
// {
//     Serial.println("===============");
//     Serial.print("ID: ");
//     Serial.println(customVar.param_id);
//     Serial.print("Value: ");
//     Serial.println(customVar.param_value, 5);
//     Serial.print("Type: ");
//     Serial.println(customVar.param_type);
//     Serial.print("Index: ");
//     Serial.println(customVar.param_index);
//     Serial.print("Count: ");
//     Serial.println(customVar.param_count);
//     Serial.println("===============");
// }
