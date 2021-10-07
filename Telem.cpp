#include "Telem.h"

// Constructor
Telem::Telem(HardwareSerial &hs)
{

    _MAVSerial = &hs;

    system_id = SYSID;
    component_id = COMPID;

    target_sysid = TARGET_SYSID;
    target_compid = TARGET_COMPID;

    link = false;
    last_heatbeat = 0;
    time_heartbeat = 0;

    APdata.armed = false;

}

/**
 * @brief Telemtry Begin
 * @return Nada
 */
void Telem::begin()
{

    _MAVSerial->begin(SERIAL_BAUD_TELEM);

}

void Telem::run(void (*msgRecivedCallback)(uint8_t _msgid))
{

    // LATIMOS cada segundo
    if (millis() >= time_heartbeat + HEARTBEAT_INTERVAL)
    {
        // Enviamos heartbeat
        heartbeat();

        // Comprobamos link
        check_link();

        // Incrementamos
        time_heartbeat += HEARTBEAT_INTERVAL;
    }

    // ESCUCHAMOS
    mavlink_message_t msg;
    mavlink_status_t status;

    while (_MAVSerial->available() > 0)
    {
        uint8_t c = _MAVSerial->read();
        
        // Parseamos posibles msg
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
        {
      
            if (msg.sysid == target_sysid) // Si recivimos de la pix
            {

                switch (msg.msgid) {

                    case MAVLINK_MSG_ID_HEARTBEAT: // #0: Heartbeat

                        mavlink_heartbeat_t heartbeat;
                        mavlink_msg_heartbeat_decode(&msg, &heartbeat);

                        // Capturamos datos para APdata
                        APdata.custom_mode = heartbeat.custom_mode;
                        APdata.base_mode = heartbeat.base_mode;

                        // Detectamos armado/desarmado desde la pix (no gusta, poco preciso)
                        if (APdata.base_mode > 200)
                            APdata.armed = true;
                        else
                            APdata.armed = false;

                        // Time to get
                        last_heatbeat = millis();
                    
                        break;

                    case MAVLINK_MSG_ID_DISTANCE_SENSOR: // #132: DISTANCE SENSOR

                        mavlink_distance_sensor_t _distance_sensor;
                        mavlink_msg_distance_sensor_decode(&msg, &_distance_sensor);

                        APdata.distance_sensor = _distance_sensor.current_distance;

                        break;

                }
                //  Ejecutamos función callback
                msgRecivedCallback(msg.msgid);
            }
        }
    }
}

// Latido
void Telem::heartbeat()
{

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_heartbeat_pack(system_id, component_id, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, MAV_MODE_PREFLIGHT, 0, MAV_STATE_UNINIT);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    _MAVSerial->write(buf, len); // Se manda heartbeat

}

void Telem::armDisarm()
{
    // Invertimos estado armado
    APdata.armed = !APdata.armed;

    // Enviamos comando
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(system_id, component_id, &msg, target_sysid, target_compid, MAV_CMD_COMPONENT_ARM_DISARM, 0, APdata.armed, 0, 0, 0, 0, 0, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    _MAVSerial->write(buf, len);

}

// Requests

void Telem::request_data_streams(uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop){

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_request_data_stream_pack(system_id, component_id, &msg, target_sysid, target_compid, req_stream_id, req_message_rate, start_stop);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    _MAVSerial->write(buf, len);

}

void Telem::request_distance_sensor(){
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    mavlink_msg_command_long_pack(system_id, component_id, &msg, target_sysid, target_compid, MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_DISTANCE_SENSOR, DISTANCE_SENSOR_INTERVAL, 0, 0, 0, 0, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    _MAVSerial->write(buf,len);
}

// Functions

void Telem::check_link()
{

    // Calculamos tiempo pasado desde el último heartbeat
    unsigned long now = millis() / 1000;
    unsigned long last_hb = last_heatbeat / 1000;
    int dif = now - last_hb;

    if (dif >= LOST_TIME)
    { // PERDEMOS LINK
        Log.warning("LINK LOST!" CR);
        link = false;
    }
    else if (dif <= 1 && !link)
    {   // RECUPERAMOS LINK
        Log.notice("LINK OK!" CR);
        link = true;

        // Data strams a 0
        request_data_streams(MAV_DATA_STREAM_ALL, 0, 1);

        // Solicitamos sensor de distancia
        request_distance_sensor();

    }
}