#ifndef TCPCONNECTIONHANDLER_H
#define TCPCONNECTIONHANDLER_H

#include <QObject>
#include "src/Rov_Tcp_Client_Qt/Ahrs/ahrsconfigure.h"
#include "src/Rov_Tcp_Client_Qt/Ahrs/ahrsreaddata.h"
#include "src/Rov_Tcp_Client_Qt/lykacz/lykacz.h"
#include "src/Rov_Tcp_Client_Qt/Pressure/pressure.h"
#include "src/Rov_Tcp_Client_Qt/Tcp_Connection/tcpclientsocket.h"

class tcpConnectionHandler : public QObject
{
    Q_OBJECT
public:
    explicit tcpConnectionHandler( QObject* parent = nullptr );

private:
    TcpClientSocket rov_tcp_client;

    AhrsConfigure ahrs_config;
    AhrsReadData ahrs_readData;
    pressure press;
    Lykacz lykacz;

    // tcp connection status slots
public slots:

    void connectionSuccesfull();
    void connectionFailed();
    //    void TcpNewTcpReceiveLogs();

    //    void on_pushSendCustomCommand_clicked();
    //    void on_pushButtonSendMotorCommand_clicked();

    void openConnection();
    void closeConnection();

    void sendGetPressureRequest();

    void sendOpenGripper();
    void sendCloseGripper();

    void sendOpenGulper();
    void sendCloseGulper();
    void sendStopGulper();
    void sendBackwardGulper();
    void sendForwardGulper();

    //    void on_horizontalSliderServo1_valueChanged( int value );
    //    void on_horizontalSliderServo2_valueChanged( int value );

    void sendImuRate( int newRate );
    void sendEulerRate( int newRate );
    void sendPressureRate( int newRate );

    //    void on_pushButtonZeroGyros_clicked();
    //    void on_pushButtonSetMagRef_clicked();

    //    void on_pushButtonResetToFactory_clicked();

    //    void on_pushButtonAccelCalib_clicked();

    //    void on_pushButtonFlashCommit_clicked();

    //    void on_pushButtonResetEKF_clicked();

    //    void on_pushButtonReadHealth_clicked();

    //    void on_pbApplyAccMtx_clicked();
    //    void on_pbApplyMagMtx_clicked();

    //    void on_pbApplyAccBias_clicked();
    //    void on_pbApplyMagBias_clicked();
    //    void on_pbApplyGyroTrim_clicked();

    //    void on_pushButtonStopAllMotors_clicked();
signals:
    void sendConnectionStatus( unsigned status );

private:
    void on_horizontalSliderServo_SendServoValue( int servo_number, int servo_value );
    void sendGripperCommand( bool open );

    void floatToBytes( float in, quint8* out );
};

#endif // TCPCONNECTIONHANDLER_H
