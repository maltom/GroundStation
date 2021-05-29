#include "lykacz.h"
#include "../gupikmodules.h"

Lykacz::Lykacz() {}

QByteArray Lykacz::sendCommand( lykacz_command_t cmd )
{
    QByteArray data;
    data.clear();
    data.append( LYKACZ_MODULE_ID );
    data.append( 1 );
    data.append( cmd );
    return data;
}
