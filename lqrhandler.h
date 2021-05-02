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
    explicit LQRHandler(int LQRThreadTimerMiliseconds = 10, positionData* rawPosition = nullptr, QObject *parent = nullptr);

private:
    // regulator
    // time elapsed from one regulator call to another, for calculating velocity
    double deltaT;
    double centimeterPrescaler = 1.0;
    long long ticksElapsed = 0;
    positionData* rovPosition;

    bool receivedNewK = false;

    ROV rov;
    static constexpr size_t state_dim = 12;
    static constexpr size_t control_dim = 6;

    Matrix<double,state_dim,state_dim> A=Matrix<double,state_dim,state_dim>::Zero(state_dim,state_dim);
    Matrix<double,state_dim,control_dim> B=Matrix<double,state_dim,control_dim>::Zero(state_dim,control_dim);

    //Matrix<double,state_dim,state_dim> Q=Matrix<double,state_dim,state_dim>::Zero(state_dim,state_dim);
    //Matrix<double,control_dim,control_dim> R=Matrix<double,control_dim,control_dim>::Zero(control_dim,control_dim);

    DiagonalMatrix<double,state_dim> Q;
    DiagonalMatrix<double,control_dim> R;

    Matrix<double,control_dim,state_dim> K = Matrix<double,control_dim,state_dim>::Zero(control_dim,state_dim);
    // x(k-1) in state space
    VectorXd pastState = VectorXd::Zero(12);
    // x(k) in state space
    VectorXd actualState = VectorXd::Zero(12);
    // hmpff
    VectorXd desiredPosition = VectorXd::Zero(6);

    // error
    VectorXd error = VectorXd::Zero(6);

    VectorXd regulatorFeedbackPosition = VectorXd::Zero(6);

    VectorXd simulationResultState = VectorXd::Zero(12);

    void loadEigenPositions();
    void updateRawPositions();

    void calculateKMatrix();
    void calculateRegulatorFeedbackPose();
    void calculateError();

public slots:
    void receiveK(Matrix612 K);
    void update();
signals:
    void positionReady(VectorXd position, VectorXd thrusterAzimuth);
    void timePositionReady(double timeElapsed, VectorXd position, VectorXd thrusterAzimuth);
    void sendCalculatedThrust(VectorXd);
    void sendAB(Matrix1212 A, Matrix126 B);
};

#endif // LQRHANDLER_H
