#ifndef LQRHANDER_H
#define LQRHANDER_H

#include <QObject>

#include "positiondata.h"
#include "ROV.h"

#include <Eigen/Core>
#include <ct/optcon/optcon.h>
#include <cmath>
#include <EigenQP.h>
#include <Eigen/Dense>
using namespace Eigen;

class LQRHander : public QObject
{
    Q_OBJECT
public:
    explicit LQRHander(int LQRThreadTimerMiliseconds = 10, positionData* rawPosition = nullptr, QObject *parent = nullptr);

private:
    // regulator
    // time elapsed from one regulator call to another, for calculating velocity
    double deltaT;
    double centimeterPrescaler = 100.0;
    positionData* rovPosition;

    ROV rov;
    static constexpr size_t state_dim = 12;
    static constexpr size_t control_dim = 6;

    Matrix<double,state_dim,state_dim> A;
    Matrix<double,state_dim,control_dim> B;

    Matrix<double,state_dim,state_dim> Q;
    Matrix<double,control_dim,control_dim> R;

    Matrix<double,6,12> K;
    // x(k-1) in state space
    VectorXd pastState = VectorXd(12);
    // x(k) in state space
    VectorXd actualState = VectorXd(12);
    // hmpff
    VectorXd desiredPosition = VectorXd(6);

    // error
    VectorXd error = VectorXd(6);

    VectorXd regulatorFeedbackPosition = VectorXd(6);

    void loadEigenPositions();
    void updateRawPositions();

    void calculateKMatrix();
    void calculateRegulatorFeedbackPose();
    void calculateError();



public slots:

    //void setPastPosition(const positionData &position);
    void update();
signals:
    void sendDemandForPosition();
    void sendCalculatedThrust(VectorXd);
    void sendCalculatedPosition();
};

#endif // LQRHANDER_H
