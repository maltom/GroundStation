#ifndef PRESSURE_H
#define PRESSURE_H

#include <QByteArray>

/* pressure sensor control */

typedef enum
{
    GET_ACTUAL_PRESSURE,
    SET_PRESSURE_PERIODIC_MEASURE,

} task_pressure_sensor_command_t;

class pressure
{
private:
    QByteArray pressure_CreateGetPressureMessage( void );
    QByteArray pressure_CreateSetPressurePeriodMessage( int press_rate );

public:
    pressure();

    QByteArray Pressure_CreateControlMessage( task_pressure_sensor_command_t cmd_id, int press_rate );

    /*!
     * \brief From presssure module we can only receive pressure!!!
     * \param message
     * \return pressure
     */
    float Pressure_ParseReceivedMessage( QByteArray received_data );
};

#endif // PRESSURE_H
