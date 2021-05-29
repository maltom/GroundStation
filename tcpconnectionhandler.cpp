#include "tcpconnectionhandler.h"
#include "src/Rov_Tcp_Client_Qt/gupikmodules.h"
#include <QDateTime>
#include <QDebug>
#include <QByteArray>
#include <QVarLengthArray>
#include <QTextCodec>

tcpConnectionHandler::tcpConnectionHandler( QObject* parent )
{

    connect( rov_tcp_client.TcpClientConnect_GetSocket(), SIGNAL( connected() ), this, SLOT( TcpNewConnectionLogs() ) );
    connect( rov_tcp_client.TcpClientConnect_GetSocket(),
             SIGNAL( disconnected() ),
             this,
             SLOT( TcpNewDisconnectionLogs() ) );
    connect( rov_tcp_client.TcpClientConnect_GetSocket(), SIGNAL( readyRead() ), this, SLOT( TcpNewTcpReceiveLogs() ) );
}

/* motor control message id */
typedef enum
{
    MOTOR_CONTROL_TORQUE_MESSAGE_ID   = 0,
    MOTOR_CONTROL_STOP_ALL_MESSAGE_ID = 1,
} motor_control_message_id_t;

static QString mainwindow_QVarLengthArrayToQString( QVarLengthArray< qint8 > message_array )
{
    QString message_string = "";
    for( int i = 0; i < message_array.length(); i++ )
    {
        message_string.append( static_cast< char >( message_array[ i ] ) );
    }
    return message_string;
}

void tcpConnectionHandler::TcpNewConnectionLogs()
{
    this->addToLogs( "Connected!" );
}

void tcpConnectionHandler::TcpNewDisconnectionLogs()
{
    this->addToLogs( "Disconnected!" );
}

void tcpConnectionHandler::TcpNewTcpReceiveLogs()
{
    QString log;
    QByteArray received_data = rov_tcp_client.TcpClientConnect_GetAllReceivedData();
    //    QString received_data_log = "Data received: " + received_data.toStdString();
    //    this->addToLogs(received_data_log);

    int module_id = static_cast< int >( received_data[ 0 ] );

    switch( module_id )
    {
    case PRESSURE_SENSOR_MODULE_ID:
    {
        float pressure = press.Pressure_ParseReceivedMessage( received_data );
        log            = "Pressure: " + ( QString::number( pressure ) );
        this->addToLogs( log );
    }
    break;

    case AHRS_DATA_ID:
    {
        // this->addToLogs("Received ahrs data!!!");
        // dalsze parsowanie received danych, na podstawie ahrs data id
        this->addToLogs( ahrs_readData.AhrsReadData_ParseAhrsDataMessage( received_data ) );
    }
    break;

    default:
    {
        this->addToLogs( "No matching module id!!!" );
    }
    break;
    }
}

void tcpConnectionHandler::on_pushButtonClose_clicked()
{
    // before close free used memory, close all connections...
    this->addToLogs( "Closing..." );
    rov_tcp_client.TcpClient_Disconnect();
    close();
}

void tcpConnectionHandler::addToLogs( QString message )
{
    QString currentDateTime = QDateTime::currentDateTime().toString( "yyyy.MM.dd hh:mm:ss" );
    ui->textEditLogs->append( currentDateTime + "\t" + message );
}

void tcpConnectionHandler::on_pushSendCustomCommand_clicked()
{
    this->addToLogs( "Send custom command..." );
    QString qs_custom_command = ui->lineEditCustomCommand->text();
    this->addToLogs( "Entered custom command: " + qs_custom_command );
    // variable to hold following message bytes
    QVarLengthArray< int > bytes;
    // variable to hold one byte value of message
    QString s_byte;
    // transfer string to bytes, to send to rov
    for( int i = 0; i < qs_custom_command.length(); i++ )
    {
        if( qs_custom_command[ i ] == ':' )
        {
            bytes.append( s_byte.toInt() );
            s_byte.clear();
        }
        else
        {
            s_byte.append( qs_custom_command[ i ] );
        }
    }
    // append last byte of command
    bytes.append( s_byte.toInt() );
    // construct command message
    qs_custom_command.clear();
    for( int i = 0; i < bytes.length(); i++ )
    {
        qs_custom_command.append( static_cast< char >( bytes[ i ] ) );
    }
    // send message to rov
    rov_tcp_client.TcpClient_Transmit( qs_custom_command.toUtf8() );
}

void tcpConnectionHandler::on_pushButtonSendMotorCommand_clicked()
{
    this->addToLogs( "Send motor command..." );
    QString log_motor
        = "Motor number: " + ui->lineEditMotorNumber->text() + " Motor torque: " + ui->lineEditTorque->text();
    this->addToLogs( log_motor );

    int motor_number = ui->lineEditMotorNumber->text().toInt();
    int motor_torque = ui->lineEditTorque->text().toInt();

    QVarLengthArray< quint8 > bytes;
    bytes.clear();

    if( motor_torque >= 0 && motor_torque <= 2000 )
    {
        // construct motor control mesage
        bytes.append( MOTOR_CONTROL_MODULE_ID );         // motor module M4 handler id
        bytes.append( 4 );                               // payload szie
        bytes.append( MOTOR_CONTROL_TORQUE_MESSAGE_ID ); // motor control message id
        bytes.append( motor_number );
        quint16 value   = static_cast< quint16 >( motor_torque );
        quint8 ms_value = static_cast< quint8 >( value >> 8 );
        quint8 ls_value = static_cast< quint8 >( value & 0xFF );
        bytes.append( ms_value );
        bytes.append( ls_value );

        // convet to QByteArray
        QByteArray motor_control_message;
        motor_control_message.clear();
        for( int i = 0; i < bytes.length(); i++ )
        {
            motor_control_message.append( static_cast< char >( bytes[ i ] ) );
        }

        // send message to rov
        rov_tcp_client.TcpClient_Transmit( motor_control_message );
    }
}

void tcpConnectionHandler::openConnection()
{
    this->addToLogs( "Tcp Client connecting..." );
    this->addToLogs( rov_tcp_client.TcpClientConnect_GetAddressAndPort() );
    rov_tcp_client.TcpClient_Connect();
}

void tcpConnectionHandler::closeConnection()
{
    this->addToLogs( "Tcp Client disconnecting..." );
    rov_tcp_client.TcpClient_Disconnect();
}

void tcpConnectionHandler::on_pushButtonGetPressure_clicked()
{
    QByteArray msg = press.Pressure_CreateControlMessage( GET_ACTUAL_PRESSURE, 0 );
    this->addToLogs( "[Pressure] Get pressure!" );
    rov_tcp_client.TcpClient_Transmit( msg );
}

void tcpConnectionHandler::on_pushButtonSetPressRate_clicked()
{
    int press_rate = ui->lineEditPressRate->text().toUInt(); // ui->lineEdit
    if( press_rate >= 0 && press_rate <= 10 )
    {
        this->addToLogs( "[Pressure] Set new pressure rate!" );
        QByteArray msg = press.Pressure_CreateControlMessage( SET_PRESSURE_PERIODIC_MEASURE, press_rate );
        rov_tcp_client.TcpClient_Transmit( msg );
    }
}

void tcpConnectionHandler::sendOpenGripper()
{
    this->addToLogs( "Open Gripper!" );
    on_pushButtonCloseGripper_SendGrippercommand( true ); // 0 - closed gripper
}

void tcpConnectionHandler::sendCloseGripper()
{
    this->addToLogs( "Close Gripper" );
    on_pushButtonCloseGripper_SendGrippercommand( false ); // 1 - open gripper
}

void tcpConnectionHandler::on_pushButtonCloseGripper_SendGrippercommand( bool open )
{
    QVarLengthArray< qint8 > message_array;
    message_array.append( GRIPPER_MODULE_ID );
    message_array.append( 1 ); // payload size
    message_array.append( open );

    QString message_string = mainwindow_QVarLengthArrayToQString( message_array );

    rov_tcp_client.TcpClient_Transmit( message_string.toUtf8() );
}

void tcpConnectionHandler::on_horizontalSliderServo1_valueChanged( int value )
{
    on_horizontalSliderServo_SendServoValue( 1, value );
}

void tcpConnectionHandler::on_horizontalSliderServo2_valueChanged( int value )
{
    on_horizontalSliderServo_SendServoValue( 2, value );
}

void tcpConnectionHandler::on_horizontalSliderServo_SendServoValue( int servo_number, int servo_value )
{
    if( servo_value % 10 == 0 && servo_value > 0 && servo_value < 100 )
    {
        QString log = "Value servo" + QString::number( servo_number ) + " : " + QString::number( servo_value );
        this->addToLogs( log );

        QVarLengthArray< qint8 > message_array;
        message_array.append( SERVO_MODULE_ID );
        message_array.append( 2 );            // payload size
        message_array.append( servo_number ); // servo number
        message_array.append( servo_value );  // servo value - 0 neutral, 10 min, 90 max

        QString message_string = mainwindow_QVarLengthArrayToQString( message_array );

        rov_tcp_client.TcpClient_Transmit( message_string.toUtf8() );
    }
}

void tcpConnectionHandler::on_pushButtonZeroGyros_clicked()
{
    this->addToLogs( "[AHRS] Zero Gyros" );
    QByteArray ahrs_config_message = ahrs_config.AhrsConfigure_SendConfigurationMassage( AHRS_COMMAND_ZERO_GYROS );
    rov_tcp_client.TcpClient_Transmit( ahrs_config_message );
}

void tcpConnectionHandler::on_pushButtonSetImuRate_clicked()
{
    quint32 rate = ui->leImuRate->text().toInt();
    if( rate > 10 )
    {
        this->addToLogs( "[AHRS] IMU rate must not be higher than 10 Hz!" );
        return;
    }
    this->addToLogs( "[AHRS] Set IMU rate to " + QString::number( rate ) + " Hz." );
    QByteArray ahrs_config_message
        = ahrs_config.AhrsConfigure_SendConfigurationMassageWithData( AHRS_SET_IMU_RATE, ( quint8* )&rate );
    rov_tcp_client.TcpClient_Transmit( ahrs_config_message );
}

void tcpConnectionHandler::on_pushButtonSetEulerRate_clicked()
{
    quint32 rate = ui->leEulerRate->text().toInt();
    if( rate > 10 )
    {
        this->addToLogs( "[AHRS] Euler rate must not be higher than 10 Hz!" );
        return;
    }
    this->addToLogs( "[AHRS] Set Euler rate to " + QString::number( rate ) + " Hz." );
    QByteArray ahrs_config_message
        = ahrs_config.AhrsConfigure_SendConfigurationMassageWithData( AHRS_SET_EULER_RATE, ( quint8* )&rate );
    rov_tcp_client.TcpClient_Transmit( ahrs_config_message );
}

void tcpConnectionHandler::on_pushButtonSetMagRef_clicked()
{
    this->addToLogs( "[AHRS] Set magnetometer reference" );
    QByteArray ahrs_config_message
        = ahrs_config.AhrsConfigure_SendConfigurationMassage( AHRS_COMMAND_SET_MAG_REFERENCE );
    rov_tcp_client.TcpClient_Transmit( ahrs_config_message );
}

void tcpConnectionHandler::on_pushButtonResetToFactory_clicked()
{
    this->addToLogs( "[AHRS] Reset to factory" );
    QByteArray ahrs_config_message
        = ahrs_config.AhrsConfigure_SendConfigurationMassage( AHRS_COMMAND_RESET_TO_FACTORY );
    rov_tcp_client.TcpClient_Transmit( ahrs_config_message );
}

void tcpConnectionHandler::on_pushButtonAccelCalib_clicked()
{
    this->addToLogs( "[AHRS] Calibrate accelerometers" );
    QByteArray ahrs_config_message
        = ahrs_config.AhrsConfigure_SendConfigurationMassage( AHRS_COMMAND_CALIBRATE_ACCELEROMETRS );
    rov_tcp_client.TcpClient_Transmit( ahrs_config_message );
}

void tcpConnectionHandler::on_pushButtonFlashCommit_clicked()
{
    this->addToLogs( "[AHRS] Commit to flash" );
    QByteArray ahrs_config_message = ahrs_config.AhrsConfigure_SendConfigurationMassage( AHRS_COMMAND_FLASH_COMMIT );
    rov_tcp_client.TcpClient_Transmit( ahrs_config_message );
}

void tcpConnectionHandler::on_pushButtonResetEKF_clicked()
{
    this->addToLogs( "[AHRS] Reset EKF" );
    QByteArray ahrs_config_message = ahrs_config.AhrsConfigure_SendConfigurationMassage( AHRS_COMMAND_RESET_EKF );
    rov_tcp_client.TcpClient_Transmit( ahrs_config_message );
}

void tcpConnectionHandler::on_pushButtonReadHealth_clicked()
{
    this->addToLogs( "[AHRS] Read health" );
    QByteArray ahrs_config_message = ahrs_config.AhrsConfigure_SendConfigurationMassage( AHRS_READ_HEALTH );
    rov_tcp_client.TcpClient_Transmit( ahrs_config_message );
}

void tcpConnectionHandler::on_pbApplyAccMtx_clicked()
{
    this->addToLogs( "[AHRS] Set accelerometer calibration matrix" );
    quint8 data[ 36 ] = { 0 };
    floatToBytes( ui->leAccMtx11->text().toFloat(), &( data[ 0 ] ) );
    floatToBytes( ui->leAccMtx12->text().toFloat(), &( data[ 4 ] ) );
    floatToBytes( ui->leAccMtx13->text().toFloat(), &( data[ 8 ] ) );
    floatToBytes( ui->leAccMtx21->text().toFloat(), &( data[ 12 ] ) );
    floatToBytes( ui->leAccMtx22->text().toFloat(), &( data[ 16 ] ) );
    floatToBytes( ui->leAccMtx23->text().toFloat(), &( data[ 20 ] ) );
    floatToBytes( ui->leAccMtx31->text().toFloat(), &( data[ 24 ] ) );
    floatToBytes( ui->leAccMtx32->text().toFloat(), &( data[ 28 ] ) );
    floatToBytes( ui->leAccMtx33->text().toFloat(), &( data[ 32 ] ) );
    QByteArray ahrs_config_message
        = ahrs_config.AhrsConfigure_SendConfigurationMassageWithData( AHRS_SET_ACC_MATRIX, data );
    rov_tcp_client.TcpClient_Transmit( ahrs_config_message );
}

// copies float as bytes to byte array (MSB first)
void tcpConnectionHandler::floatToBytes( float in, quint8* out )
{
    quint32 tmp = 0;
    memcpy( &tmp, &in, 4 );
    out[ 0 ] = ( tmp >> 24 ) & 0xFF;
    out[ 1 ] = ( tmp >> 16 ) & 0xFF;
    out[ 2 ] = ( tmp >> 8 ) & 0xFF;
    out[ 3 ] = ( tmp )&0xFF;
}

void tcpConnectionHandler::on_pbApplyMagMtx_clicked()
{
    this->addToLogs( "[AHRS] Set magnetometer calibration matrix" );
    quint8 data[ 36 ] = { 0 };
    floatToBytes( ui->leMagMtx11->text().toFloat(), &( data[ 0 ] ) );
    floatToBytes( ui->leMagMtx12->text().toFloat(), &( data[ 4 ] ) );
    floatToBytes( ui->leMagMtx13->text().toFloat(), &( data[ 8 ] ) );
    floatToBytes( ui->leMagMtx21->text().toFloat(), &( data[ 12 ] ) );
    floatToBytes( ui->leMagMtx22->text().toFloat(), &( data[ 16 ] ) );
    floatToBytes( ui->leMagMtx23->text().toFloat(), &( data[ 20 ] ) );
    floatToBytes( ui->leMagMtx31->text().toFloat(), &( data[ 24 ] ) );
    floatToBytes( ui->leMagMtx32->text().toFloat(), &( data[ 28 ] ) );
    floatToBytes( ui->leMagMtx33->text().toFloat(), &( data[ 32 ] ) );
    QByteArray ahrs_config_message
        = ahrs_config.AhrsConfigure_SendConfigurationMassageWithData( AHRS_SET_MAG_MATRIX, data );
    rov_tcp_client.TcpClient_Transmit( ahrs_config_message );
}

void tcpConnectionHandler::on_pbApplyAccBias_clicked()
{
    this->addToLogs( "[AHRS] Set accelerometer bias" );
    quint8 data[ 12 ] = { 0 };
    floatToBytes( ui->leAccBiasX->text().toFloat(), &( data[ 0 ] ) );
    floatToBytes( ui->leAccBiasY->text().toFloat(), &( data[ 4 ] ) );
    floatToBytes( ui->leAccBiasZ->text().toFloat(), &( data[ 8 ] ) );
    QByteArray ahrs_config_message
        = ahrs_config.AhrsConfigure_SendConfigurationMassageWithData( AHRS_SET_ACC_BIAS, data );
    rov_tcp_client.TcpClient_Transmit( ahrs_config_message );
}

void tcpConnectionHandler::on_pbApplyMagBias_clicked()
{
    this->addToLogs( "[AHRS] Set magnetometer bias" );
    quint8 data[ 12 ] = { 0 };
    floatToBytes( ui->leMagBiasX->text().toFloat(), &( data[ 0 ] ) );
    floatToBytes( ui->leMagBiasY->text().toFloat(), &( data[ 4 ] ) );
    floatToBytes( ui->leMagBiasZ->text().toFloat(), &( data[ 8 ] ) );
    QByteArray ahrs_config_message
        = ahrs_config.AhrsConfigure_SendConfigurationMassageWithData( AHRS_SET_MAG_BIAS, data );
    rov_tcp_client.TcpClient_Transmit( ahrs_config_message );
}

void tcpConnectionHandler::on_pbApplyGyroTrim_clicked()
{
    this->addToLogs( "[AHRS] Set gyroscope trim" );
    quint8 data[ 12 ] = { 0 };
    floatToBytes( ui->leGyroTrimX->text().toFloat(), &( data[ 0 ] ) );
    floatToBytes( ui->leGyroTrimY->text().toFloat(), &( data[ 4 ] ) );
    floatToBytes( ui->leGyroTrimZ->text().toFloat(), &( data[ 8 ] ) );
    QByteArray ahrs_config_message
        = ahrs_config.AhrsConfigure_SendConfigurationMassageWithData( AHRS_SET_GYRO_TRIM, data );
    rov_tcp_client.TcpClient_Transmit( ahrs_config_message );
}

void tcpConnectionHandler::on_pushButtonStopAllMotors_clicked()
{
    // send command to stop all motors
    this->addToLogs( "Stopping all motors..." );

    QVarLengthArray< quint8 > bytes;
    bytes.clear();

    // construct motor control mesage
    bytes.append( MOTOR_CONTROL_MODULE_ID );           // motor module M4 handler id
    bytes.append( 4 );                                 // payload szie
    bytes.append( MOTOR_CONTROL_STOP_ALL_MESSAGE_ID ); // motor control message id
    bytes.append( 0 );                                 // unused, but payload size must be equal to 4
    bytes.append( 0 );
    bytes.append( 0 );

    // convet to QByteArray
    QByteArray motor_control_message;
    motor_control_message.clear();
    for( int i = 0; i < bytes.length(); i++ )
    {
        motor_control_message.append( static_cast< char >( bytes[ i ] ) );
    }

    // send message to rov
    rov_tcp_client.TcpClient_Transmit( motor_control_message );
}

void tcpConnectionHandler::on_pushButtonStop_clicked()
{
    this->addToLogs( "[AHRS] Stop pipe robot" );
    rov_tcp_client.TcpClient_Transmit( lykacz.sendCommand( LYKACZ_STOP ) );
}

void tcpConnectionHandler::on_pushButtonDriveBackward_clicked()
{
    this->addToLogs( "[AHRS] Move pipe robot forward" );
    rov_tcp_client.TcpClient_Transmit( lykacz.sendCommand( LYKACZ_FORWARD ) );
}

void tcpConnectionHandler::on_pushButtonDriveForward_clicked()
{
    this->addToLogs( "[AHRS] Move pipe robot backward" );
    rov_tcp_client.TcpClient_Transmit( lykacz.sendCommand( LYKACZ_BACKWARD ) );
}

void tcpConnectionHandler::sendOpenGulper()
{
    this->addToLogs( "[AHRS] Open pipe robot's jaws" );
    rov_tcp_client.TcpClient_Transmit( lykacz.sendCommand( LYKACZ_OPEN ) );
}

void tcpConnectionHandler::sendCloseGulper()
{
    this->addToLogs( "[AHRS] Close pipe robot's jaws" );
    rov_tcp_client.TcpClient_Transmit( lykacz.sendCommand( LYKACZ_CLOSE ) );
}
