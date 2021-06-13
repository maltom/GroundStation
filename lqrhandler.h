#ifndef LQRHANDLER_H
#define LQRHANDLER_H

#include <QObject>

#include "positiondata.h"
#include "ROV.h"
#include "typedefs.h"
#include <Eigen/Core>
#include <ct/optcon/optcon.h>
#include <cmath>
#include <EigenQP.h>
#include <Eigen/Dense>
using namespace Eigen;

class LQRHandler : public QObject
{
    Q_OBJECT
public:
    explicit LQRHandler( int LQRThreadTimerMiliseconds = 10,
                         positionData* rawPosition     = nullptr,
                         QObject* parent               = nullptr );

private:
    // regulator
    // time elapsed from one regulator call to another, for calculating velocity
    double deltaT;
    double centimeterPrescaler = 1.0;
    long long ticksElapsed     = 0;
    positionData* rovPosition;

    bool receivedNewK = false;

    ROV rov;

    // error
    VectorXd error = VectorXd::Zero( 6 );

    void updateRawPositions();

    void calculateError();

public slots:
    void update();
    void receiveDesiredForces( double x, double y, double z, double roll, double pitch, double yaw );
signals:

    void sendCalculatedThrust( VectorXd );
    void sendCalculatedAzimuth( VectorXd );
};

#endif // LQRHANDLER_H
