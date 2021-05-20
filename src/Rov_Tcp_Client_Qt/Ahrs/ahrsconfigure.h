#ifndef AHRSCONFIGURE_H
#define AHRSCONFIGURE_H

#include <QByteArray>

typedef enum
{
    /* commands */
    AHRS_COMMAND_FLASH_COMMIT,
    AHRS_COMMAND_RESET_TO_FACTORY,
    AHRS_COMMAND_ZERO_GYROS,
    AHRS_COMMAND_SET_MAG_REFERENCE,
    AHRS_COMMAND_CALIBRATE_ACCELEROMETRS,
    AHRS_COMMAND_RESET_EKF,

    /* configure registers */
    AHRS_READ_HEALTH,

    /* configure register (write register operation) */
    AHRS_SET_IMU_RATE,
    AHRS_SET_EULER_RATE,
    AHRS_SET_ACC_MATRIX,
    AHRS_SET_MAG_MATRIX,
    AHRS_SET_GYRO_TRIM,
    AHRS_SET_MAG_BIAS,
    AHRS_SET_ACC_BIAS,

} ahrs_config_command_type;

class AhrsConfigure
{
private:
    QByteArray ahrsConfigure_CreateAhrsCommand(ahrs_config_command_type cmd_type_id);
    QByteArray ahrsConfigure_InvokeReadRegister(ahrs_config_command_type cmd_type_id);
    QByteArray ahrsConfigure_SetSensorRate(ahrs_config_command_type cmd_type_id, quint8 rate);
    QByteArray ahrsConfigure_SetMatrix(ahrs_config_command_type cmd_type_id, quint8 *data);
    QByteArray ahrsConfigure_SetBiasTrim(ahrs_config_command_type cmd_type_id, quint8 *data);

public:
    AhrsConfigure();

    QByteArray AhrsConfigure_SendConfigurationMassage(ahrs_config_command_type cmd_type_id);
    QByteArray AhrsConfigure_SendConfigurationMassageWithData(ahrs_config_command_type cmd_type_id, quint8 *data);
};

#endif // AHRSCONFIGURE_H
