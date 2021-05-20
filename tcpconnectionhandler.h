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

    TcpClientSocket rov_tcp_client;

    AhrsConfigure ahrs_config;
    AhrsReadData ahrs_readData;
    pressure press;
    Lykacz lykacz;
};

#endif // TCPCONNECTIONHANDLER_H
