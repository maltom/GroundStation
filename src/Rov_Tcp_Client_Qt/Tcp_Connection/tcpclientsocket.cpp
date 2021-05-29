#include "tcpclientsocket.h"
#include <QDebug>
#include <QHostAddress>

#define BASIC_STR( x ) #x
#define STR( x ) BASIC_STR( x )

TcpClientSocket::TcpClientSocket( QObject* parent ) : QObject( parent ) {}

QByteArray TcpClientSocket::TcpClientConnect_GetAllReceivedData()
{
    QByteArray received_data = socket.readAll();
    return received_data;
}

QString TcpClientSocket::TcpClientConnect_GetAddressAndPort()
{
    QString addr_port = ( "TCP address: " + QString::fromStdString( ADDRESS ) + " port: " + QString::number( PORT ) );
    return addr_port;
}

QTcpSocket* TcpClientSocket::TcpClientConnect_GetSocket()
{
    return &socket;
}

bool TcpClientSocket::TcpClient_Connect()
{

    socket.connectToHost( QHostAddress( ADDRESS ), PORT );

    if( !socket.waitForConnected( 5000 ) )
    {
        qDebug() << socket.errorString();
        return false;
    }
    return true;
}

void TcpClientSocket::TcpClient_Disconnect()
{

    socket.close();
}

void TcpClientSocket::TcpClient_Transmit( QByteArray bytes )
{

    socket.write( bytes );
}
