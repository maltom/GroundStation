#ifndef GUPIKMODULES_H
#define GUPIKMODULES_H

/**************************************************************************************/
/*****************************Gupik Rov Modules Id*************************************/
/**************************************************************************************/

/* \brief first digit in hex representation is core number */

/* M4 modules */
static const int MOTOR_CONTROL_MODULE_ID   = 0x40;
static const int PRESSURE_SENSOR_MODULE_ID = 0x41;
static const int GRIPPER_MODULE_ID         = 0x42;
static const int SERVO_MODULE_ID           = 0x43;
static const int LYKACZ_MODULE_ID          = 0x44;

/* M7 modules */
static const int AHRS_CONFIGURE_ID = 0x71;
static const int AHRS_DATA_ID      = 0x73;

/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/

class GupikModules
{
public:
    GupikModules();
};

#endif // GUPIKMODULES_H
