#include "ahrsreaddata.h"
#include <QString>

float AhrsReadData::intToFloat(quint8 *in)
{
    quint32 tmp = 0;
    tmp = (quint32)(*(in + 0)) << 24;
    tmp |= (quint32)(*(in + 1)) << 16;
    tmp |= (quint32)(*(in + 2)) << 8;
    tmp |= (quint32)(*(in + 3));
    float out = 0;
    memcpy(&out, &tmp, 4);
    return out;
}


QString AhrsReadData::ahrsReadData_ParseAhrsImuDataMessage(QByteArray ahrs_data_message)
{
    QString ret = "[AHRS] IMU data\r\nGyroscope: X=";
    quint8 len = ahrs_data_message[1] - 1;
    float t = 0;
    if(len >= 44)
    {
        t = intToFloat((quint8*)(ahrs_data_message.data() + 3));
        ret.append(QString::number(t));
        ret.append(" Y=");
        t = intToFloat((quint8*)(ahrs_data_message.data() + 7));
        ret.append(QString::number(t));
        ret.append(" Z=");
        t = intToFloat((quint8*)(ahrs_data_message.data() + 11));
        ret.append(QString::number(t));
        ret.append("\r\nAccelerometer: X=");
        t = intToFloat((quint8*)(ahrs_data_message.data() + 19));
        ret.append(QString::number(t));
        ret.append(" Y=");
        t = intToFloat((quint8*)(ahrs_data_message.data() + 23));
        ret.append(QString::number(t));
        ret.append(" Z=");
        t = intToFloat((quint8*)(ahrs_data_message.data() + 27));
        ret.append(QString::number(t));
        ret.append("\r\nMagnetometer: X=");
        t = intToFloat((quint8*)(ahrs_data_message.data() + 35));
        ret.append(QString::number(t));
        ret.append(" Y=");
        t = intToFloat((quint8*)(ahrs_data_message.data() + 39));
        ret.append(QString::number(t));
        ret.append(" Z=");
        t = intToFloat((quint8*)(ahrs_data_message.data() + 43));
        ret.append(QString::number(t));
        ret.append("\r\n");
    } else
    {
        ret = "[AHRS] (IMU) Incomplete data!\r\n";
    }

    return ret;
}

QString AhrsReadData::ahrsReadData_ParseAhrsEulerDataMessage(QByteArray ahrs_data_message)
{
    QString ret = "[AHRS] Euler data\r\nPitch angle: ";
    quint8 len = ahrs_data_message[1] - 1;
    qint16 t = 0;
    if(len >= 16)
    {
        //TODO: are separate 16-bit words in each register sent as big-endian or little-endian? Assumed big-endian.
        t = ahrs_data_message[5] << 8;
        t |= ahrs_data_message[6];
        ret.append(QString::number((float)t / 91.02222f));
        ret.append(" Pitch rate: ");
        t = ahrs_data_message[13] << 8;
        t |= ahrs_data_message[14];
        ret.append(QString::number((float)t / 16.0f));
        ret.append("\r\nRoll angle: ");
        t = ahrs_data_message[3] << 8;
        t |= ahrs_data_message[4];
        ret.append(QString::number((float)t / 91.02222f));
        ret.append(" Roll rate: ");
        t = ahrs_data_message[11] << 8;
        t |= ahrs_data_message[12];
        ret.append(QString::number((float)t / 16.0f));
        ret.append("\r\nYaw angle: ");
        t = ahrs_data_message[7] << 8;
        t |= ahrs_data_message[8];
        ret.append(QString::number((float)t / 91.02222f));
        ret.append(" Yaw rate: ");
        t = ahrs_data_message[15] << 8;
        t |= ahrs_data_message[16];
        ret.append(QString::number((float)t / 16.0f));
        ret.append("\r\n");
    } else
    {
        ret = "[AHRS] (Euler) Incomplete data!\r\n";
    }

    return ret;
}

QString AhrsReadData::ahrsReadData_ParseAhrsHealthDataMessage(QByteArray ahrs_data_message)
{
    QString ret = "[AHRS] Health data\r\nMagnetometer intialized: ";
    quint8 len = ahrs_data_message[1] - 1;
    if(len >= 4)
    {
        if(!(ahrs_data_message[6] & 2)) ret.append("True"); else ret.append("False");
        ret.append("\r\nGyroscope initialized: ");
        if(!(ahrs_data_message[6] & 4)) ret.append("True"); else ret.append("False");
        ret.append("\r\nAccelerometer initialized: ");
        if(!(ahrs_data_message[6] & 8)) ret.append("True"); else ret.append("False");
        ret.append("\r\nAccelerometer norm exceeded: ");
        if(ahrs_data_message[6] & 16) ret.append("True"); else ret.append("False");
        ret.append("\r\nMagnetometer norm exceeded: ");
        if(ahrs_data_message[6] & 32) ret.append("True"); else ret.append("False");
        ret.append("\r\nUART overflow: ");
        if(ahrs_data_message[7] & 1) ret.append("True"); else ret.append("False");
        ret.append("\r\n");
    } else
    {
        ret = "[AHRS] (Health) Incomplete data!\r\n";
    }
    return ret;
}


AhrsReadData::AhrsReadData()
{

}

QString AhrsReadData::AhrsReadData_ParseAhrsDataMessage(QByteArray ahrs_data_message)
{
    QString ret;
    switch(ahrs_data_message[2])
    {
        case AHRS_DATA_IMU:
            ret = ahrsReadData_ParseAhrsImuDataMessage(ahrs_data_message);
        break;
        case AHRS_DATA_EULER:
            ret = ahrsReadData_ParseAhrsEulerDataMessage(ahrs_data_message);
        break;
        case AHRS_DATA_HEALTH:
            ret = ahrsReadData_ParseAhrsHealthDataMessage(ahrs_data_message);
        break;

        default:
        break;
    }
    return ret;
}
