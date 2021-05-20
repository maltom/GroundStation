#ifndef TCPCLIENTSOCKET_H
#define TCPCLIENTSOCKET_H

#include <QObject>
#include <QTcpSocket>
#include <QAbstractSocket>
#include <QDebug>
#include <string>
#include <QString>

#define ADDRESS     "169.254.0.20"
#define PORT        4242

class TcpClientSocket : public QObject
{
    Q_OBJECT
public:
    explicit TcpClientSocket(QObject *parent = nullptr);

    QByteArray TcpClientConnect_GetAllReceivedData(void);
    QString TcpClientConnect_GetAddressAndPort(void);
    QTcpSocket * TcpClientConnect_GetSocket(void);

    bool TcpClient_Connect(void);
    void TcpClient_Disconnect(void);
    void TcpClient_Transmit(QByteArray bytes);

private:
    QTcpSocket socket;
};

#endif // TCPCLIENTSOCKET_H
