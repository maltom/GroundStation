#include "spacemousecontroller.h"
#include <libevdev/libevdev.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
spaceMouseController::spaceMouseController( QObject* parent ) : QObject( parent )
{

    findDevice();
    // receiveCoordinates();
}
spaceMouseController::~spaceMouseController()
{
    dev = nullptr;
}

void spaceMouseController::findDevice( void )
{
    for( int i = 0; i < 50; ++i )
    {
        sprintf( eventName, "/dev/input/event%d", i );
        fd     = open( eventName, O_RDONLY | O_NONBLOCK );
        int rc = libevdev_new_from_fd( fd, &dev );
        if( rc >= 0 )
        {
            getName = libevdev_get_name( dev );
            if( getName == "3Dconnexion SpaceMouse Pro" )
            {
                status = 1;
                // emit sendSpaceStatus(status);
                return;
            }
        }
    }
    // status = 0;
    // emit sendSpaceStatus(status);
    return;
}
void spaceMouseController::receiveCoordinates( void )
{
    int rc = libevdev_next_event( dev, LIBEVDEV_READ_FLAG_NORMAL, &ev );
    if( rc == 0 )
    {
        if( ev.type == EV_REL )
        {
            switch( ev.code )
            {
            case REL_Y:
                x = -ev.value;
                break;
            case REL_X:
                y = ev.value;
                break;
            case REL_Z:
                z = ev.value;
                break;
            case REL_RY:
                roll = -ev.value;
                break;
            case REL_RX:
                pitch = ev.value;
                break;
            case REL_RZ:
                yaw = ev.value;
                break;
            }

            status = 1;
            emit sendSpaceStatus( status );
            emit sendCoordinates( x, y, z, roll, pitch, yaw );
        }
        else if( ev.type == 1 )
        {
            if( ev.value )
            {
                switch( ev.code )
                {
                case 257:
                    emit sendCameraChange();
                    break;
                case 256:
                    emit sendCoralProcessingChange();
                    break;
                    /*case REL_X:
                y = ev.value;
                break;
            case REL_Z:
                z = ev.value;
                break;
            case REL_RY:
                roll = -ev.value;
                break;
            case REL_RX:
                pitch = ev.value;
                break;
            case REL_RZ:
                yaw = ev.value;
                break;*/
                }
            }
        }
    }
}
