

#include "lqrhandler.h"
#include <iostream>
#include <cmath>

using namespace Eigen;

LQRHandler::LQRHandler( int LQRThreadTimerMiliseconds, positionData* rawPosition, QObject* parent )
    : QObject( parent ), rovPosition( rawPosition )
{
    deltaT = static_cast< double >( LQRThreadTimerMiliseconds ) / 1000.0;
}

void LQRHandler::receiveDesiredForces( double x, double y, double z, double roll, double pitch, double yaw )
{
    this->error << x, y, z, roll, pitch, yaw;
}

void LQRHandler::calculateError()
{
    // this->error = this->error;

    for( int i = 0; i < 6; ++i )
    {
        if( error( i ) > 80.0 )
            error( i ) = 80.0;
        else if( error( i ) < -80.0 )
            error( i ) = -80.0;
    }
}

void LQRHandler::normalizeError()
{
    double currentForcesRadius = std::sqrt( std::pow( error.x(), 2 ) + std::pow( error.y(), 2 ) );
    if( currentForcesRadius > maxForcesRadius )
    {
        auto factor = maxForcesRadius / currentForcesRadius;
        error.x()   = error.x() * factor;
        error.y()   = error.y() * factor;
    }
}

void LQRHandler::update()
{
    calculateError();
    // normalizeError();
    this->rov.thrust_allocation( error ); //[Fx, Fy, Fz, Mr, Mp, My]

    ++ticksElapsed;

    VectorXd thrustToSend       = VectorXd::Zero( 5 );
    thrustToSend                = rov.getThrustSignal();
    VectorXd biasToAdjustForSTM = VectorXd::Ones( 5 );
    thrustToSend                = ( thrustToSend + biasToAdjustForSTM ) * 1000.0;

    VectorXd azimuthToSend = VectorXd::Zero( 2 );
    azimuthToSend << rov.getAzimuth().x(), rov.getAzimuth().y();

    VectorXd biasToAdjustAngleForSTM = VectorXd::Ones( 2 ) * 30.0;
    azimuthToSend                    = azimuthToSend * 60 / 4.18 + biasToAdjustAngleForSTM;

    emit sendCalculatedThrust( thrustToSend );
    emit sendCalculatedAzimuth( azimuthToSend );
}
