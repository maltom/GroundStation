#ifndef AHRSREADDATA_H
#define AHRSREADDATA_H

#include <QByteArray>

/* AHRS data id */
typedef enum
{
    AHRS_DATA_IMU,    // proc accel, gyro, mag only (plus additional measure time values, it have to be ignored)
    AHRS_DATA_EULER,  // euler data
    AHRS_DATA_HEALTH, // health data

} task_ahrs_data_type_id;

class AhrsReadData
{
private:
    QString ahrsReadData_ParseAhrsImuDataMessage( QByteArray ahrs_data_message );
    QString ahrsReadData_ParseAhrsEulerDataMessage( QByteArray ahrs_data_message );
    QString ahrsReadData_ParseAhrsHealthDataMessage( QByteArray ahrs_data_message );
    float intToFloat( quint8* in );

public:
    AhrsReadData();

    QString AhrsReadData_ParseAhrsDataMessage( QByteArray ahrs_data_message );
};

#endif // AHRSREADDATA_H
