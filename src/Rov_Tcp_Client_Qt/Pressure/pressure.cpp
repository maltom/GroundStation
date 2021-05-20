#include "pressure.h"
#include "gupikmodules.h"

static const int    PRESSURE_MSG_PAYLOAD_SIZE                   = 2;

QByteArray pressure::pressure_CreateGetPressureMessage(void)
{
    QByteArray pressure_command_msg;
    pressure_command_msg.clear();
    int unused = 0;

    pressure_command_msg.append(PRESSURE_SENSOR_MODULE_ID);
    pressure_command_msg.append(PRESSURE_MSG_PAYLOAD_SIZE); // payload size
    pressure_command_msg.append(GET_ACTUAL_PRESSURE); // pressure command id
    pressure_command_msg.append(unused); // pressure measurement rate

    return pressure_command_msg;
}

QByteArray pressure::pressure_CreateSetPressurePeriodMessage(int press_rate)
{
    QByteArray pressure_command_msg;
    pressure_command_msg.clear();

    pressure_command_msg.append(PRESSURE_SENSOR_MODULE_ID);
    pressure_command_msg.append(PRESSURE_MSG_PAYLOAD_SIZE); // payload size
    pressure_command_msg.append(SET_PRESSURE_PERIODIC_MEASURE); // pressure command id
    pressure_command_msg.append(press_rate); // pressure measurement rate

    return pressure_command_msg;
}

pressure::pressure()
{

}

QByteArray pressure::Pressure_CreateControlMessage(task_pressure_sensor_command_t cmd_id, int press_rate)
{
    QByteArray message;

    switch (cmd_id)
    {
        case GET_ACTUAL_PRESSURE:
            message = pressure_CreateGetPressureMessage();
        break;
        case SET_PRESSURE_PERIODIC_MEASURE:
            message = pressure_CreateSetPressurePeriodMessage(press_rate);
        break;
        default:
        break;
    }

    return message;
}

float pressure::Pressure_ParseReceivedMessage(QByteArray received_data)
{
    int first_byte = (static_cast<unsigned int>(received_data[2]) & 0xFF);
    int second_byte = (static_cast<unsigned int>(received_data[3]) & 0xFF);
    int third_byte = (static_cast<unsigned int>(received_data[4]) & 0xFF);
    int forth_byte = (static_cast<unsigned int>(received_data[5]) & 0xFF);

    float value = (forth_byte << 24) | (third_byte << 16) | (second_byte << 8) | first_byte;
    return value/100;
}
