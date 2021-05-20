#include "ahrsconfigure.h"
#include "gupikmodules.h"


QByteArray AhrsConfigure::ahrsConfigure_CreateAhrsCommand(ahrs_config_command_type cmd_type_id)
{
    QByteArray ahrs_config_command_msg;
    ahrs_config_command_msg.clear();
    ahrs_config_command_msg.append(AHRS_CONFIGURE_ID);
    ahrs_config_command_msg.append(1); // payload size
    ahrs_config_command_msg.append(cmd_type_id); // ahrs command id

    return ahrs_config_command_msg;
}

QByteArray AhrsConfigure::ahrsConfigure_InvokeReadRegister(ahrs_config_command_type cmd_type_id)
{
    QByteArray ahrs_config_command_msg;
    ahrs_config_command_msg.clear();
    ahrs_config_command_msg.append(AHRS_CONFIGURE_ID);
    ahrs_config_command_msg.append(1); // payload size
    ahrs_config_command_msg.append(cmd_type_id);

    return ahrs_config_command_msg;
}

QByteArray AhrsConfigure::ahrsConfigure_SetSensorRate(ahrs_config_command_type cmd_type_id, quint8 rate)
{
    QByteArray ahrs_config_command_msg;
    ahrs_config_command_msg.clear();
    ahrs_config_command_msg.append(AHRS_CONFIGURE_ID);
    ahrs_config_command_msg.append(2); // payload size
    ahrs_config_command_msg.append(cmd_type_id);
    ahrs_config_command_msg.append(rate);

    return ahrs_config_command_msg;
}

QByteArray AhrsConfigure::ahrsConfigure_SetMatrix(ahrs_config_command_type cmd_type_id, quint8 *data)
{
    QByteArray ahrs_config_command_msg;
    ahrs_config_command_msg.clear();
    ahrs_config_command_msg.append(AHRS_CONFIGURE_ID);
    ahrs_config_command_msg.append(37); // payload size 37 (data type + 36 bytes for matrix)
    ahrs_config_command_msg.append(cmd_type_id);
    ahrs_config_command_msg.append((char*)data, 36);

    return ahrs_config_command_msg;
}

QByteArray AhrsConfigure::ahrsConfigure_SetBiasTrim(ahrs_config_command_type cmd_type_id, quint8 *data)
{
    QByteArray ahrs_config_command_msg;
    ahrs_config_command_msg.clear();
    ahrs_config_command_msg.append(AHRS_CONFIGURE_ID);
    ahrs_config_command_msg.append(13); // payload size 13 (data type + 12 bytes for data)
    ahrs_config_command_msg.append(cmd_type_id);
    ahrs_config_command_msg.append((char*)data, 12);

    return ahrs_config_command_msg;
}


AhrsConfigure::AhrsConfigure()
{

}

QByteArray AhrsConfigure::AhrsConfigure_SendConfigurationMassage(ahrs_config_command_type cmd_type_id)
{

    QByteArray ahrs_config_message;
    ahrs_config_message.clear();

    switch (cmd_type_id)
    {
        /* send command */
        case AHRS_COMMAND_FLASH_COMMIT:
        case AHRS_COMMAND_RESET_TO_FACTORY:
        case AHRS_COMMAND_ZERO_GYROS:
        case AHRS_COMMAND_SET_MAG_REFERENCE:
        case AHRS_COMMAND_CALIBRATE_ACCELEROMETRS:
        case AHRS_COMMAND_RESET_EKF:
            ahrs_config_message = ahrsConfigure_CreateAhrsCommand(cmd_type_id);
        break;

        /* invoke read register */
        case AHRS_READ_HEALTH:
            ahrs_config_message = ahrsConfigure_InvokeReadRegister(cmd_type_id);
        break;

        default:
        break;
    }
    return ahrs_config_message;
}

QByteArray AhrsConfigure::AhrsConfigure_SendConfigurationMassageWithData(ahrs_config_command_type cmd_type_id, quint8 *data)
{

    QByteArray ahrs_config_message;
    ahrs_config_message.clear();

    switch (cmd_type_id)
    {
        /* configure register (write register operation) */
        case AHRS_SET_IMU_RATE:
        case AHRS_SET_EULER_RATE:
            ahrs_config_message = ahrsConfigure_SetSensorRate(cmd_type_id, *data);
        break;
        case AHRS_SET_ACC_MATRIX:
        case AHRS_SET_MAG_MATRIX:
            ahrs_config_message = ahrsConfigure_SetMatrix(cmd_type_id, data);
        break;
        case AHRS_SET_ACC_BIAS:
        case AHRS_SET_MAG_BIAS:
        case AHRS_SET_GYRO_TRIM:
            ahrs_config_message = ahrsConfigure_SetBiasTrim(cmd_type_id, data);
        break;
        default:
        break;
    }
    return ahrs_config_message;
}
