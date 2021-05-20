#ifndef LYKACZ_H
#define LYKACZ_H

#include <QByteArray>

typedef enum
{
    LYKACZ_STOP = 1,
    LYKACZ_FORWARD = 2,
    LYKACZ_BACKWARD = 3,
    LYKACZ_OPEN = 4,
    LYKACZ_CLOSE = 5,
} lykacz_command_t;

class Lykacz
{
public:
    Lykacz();
    QByteArray sendCommand(lykacz_command_t cmd);
};

#endif // LYKACZ_H
