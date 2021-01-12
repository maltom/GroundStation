#include "lqrhander.h"
/*#include <Eigen/Core>
#include <Eigen/Dense>*/

using namespace Eigen;

LQRHander::LQRHander(int LQRThreadTimerMiliseconds, positionData* rawPosition, QObject *parent) : QObject(parent), rovPosition(rawPosition)
{
    deltaT=1.0/(static_cast<double>(LQRThreadTimerMiliseconds)/1000.0);
    Q.setRandom();
    R.setRandom();
}


void LQRHander::loadEigenPositions()
{
    pastState = actualState;

    VectorXd newActualState(12);
    const auto posVec = rovPosition->getPositionAndVelocity("current","position");
    for(int i=0;i<6;++i)
    {
        newActualState[i] = posVec[i]/centimeterPrescaler;
    }
    for(int i=6;i<12;++i)
    {
        newActualState[i] = (newActualState[i-6]-pastState[i-6])/deltaT;
    }

    this->actualState = newActualState;

    VectorXd newDesiredPosition(6);
    const auto posVec2 = rovPosition->getPositionAndVelocity("future","position");
    for(int i=0;i<6;++i)
    {
        newDesiredPosition[i] = posVec2[i]/centimeterPrescaler;
    }
    this->desiredPosition = newDesiredPosition;


}

void LQRHander::calculateKMatrix()
{
    A = rov.A_state_matrix(actualState);
    B = rov.B_state_matrix();

    ct::optcon::LQR<state_dim,control_dim> lqrSolver;

    lqrSolver.compute(Q,R,A,B,K);
}

void LQRHander::calculateRegulatorFeedbackPose()
{
    this->regulatorFeedbackPosition = -K*actualState;

}

void LQRHander::calculateError()
{
    this->error = this->desiredPosition - this->regulatorFeedbackPosition;
}

void LQRHander::update()
{
    loadEigenPositions();
    calculateKMatrix();
    calculateError();
    this->rov.thrust_allocation(error);
}
