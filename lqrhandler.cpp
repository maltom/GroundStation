#include "lqrhandler.h"
#include <iostream>

using namespace Eigen;

LQRHandler::LQRHandler(int LQRThreadTimerMiliseconds, positionData* rawPosition, QObject *parent) : QObject(parent), rovPosition(rawPosition)
{
    deltaT=1.0/(static_cast<double>(LQRThreadTimerMiliseconds)/1000.0);
    VectorXd Qdiag = VectorXd::Zero(state_dim);
    VectorXd Rdiag = VectorXd::Zero(control_dim);

    Rdiag<< 1.0, 1.0, 0.5, 1.0, 1.0, 1.0;
    Qdiag<<500.0, 500.0, 1500.0, 5000.0, 5000.0, 5000.0, 1.0, 1.0, 1.0, 18000.0, 18000.0, 18000.0;
    //Q = Qdiag.asDiagonal();
    //R = Rdiag.asDiagonal();
    Q.diagonal() << Qdiag;
    R.diagonal() << Rdiag;
}

void LQRHandler::loadEigenPositions()
{

    //VectorXd newActualState(12);
    //const auto posVec = rovPosition->getPositionAndVelocity("current","position");
    //for(int i=0;i<12;++i)
    //{
    //   newActualState[i] = posVec[i]/centimeterPrescaler;
    //}


    //this->actualState = newActualState;
    std::cout <<"Aktual"<< actualState<<std::endl;
    VectorXd newDesiredPosition(6);
    const auto posVec2 = rovPosition->getPositionAndVelocity("future","position");
    for(int i=0;i<6;++i)
    {
        newDesiredPosition[i] = posVec2[i]/centimeterPrescaler;
    }
    this->desiredPosition = newDesiredPosition;

    std::cout <<"Chcemy:"<< desiredPosition<<std::endl;
    
    
}

void LQRHandler::calculateKMatrix()
{
    A = rov.A_state_matrix(actualState);
    B = rov.B_state_matrix();
    //ct::optcon::LQR<state_dim,control_dim> lqrSolver;
    //std::cout<<"zegnajcie"<<std::endl;
    //std::cout <<"K przed"<< K<<std::endl;
    //std::cout<<"Oto QRABiK"<<std::endl<<Q<<std::endl<<std::endl<<R<<std::endl<<std::endl<<A<<std::endl<<std::endl<<B<<std::endl<<std::endl<<K<<std::endl<<std::endl;;
    //std::cout<<lqrSolver.compute(Q,R,A,B,K,true,true)<<std::endl;
    //std::cout <<"K po"<< K<<std::endl;
    //std::cout<<"jestem jak feniks"<<std::endl;
    emit sendAB(A,B);
}

void LQRHandler::receiveK(Matrix612 K)
{
    this->K=K;
    this->receivedNewK = true;
}

void LQRHandler::calculateRegulatorFeedbackPose()
{
    this->regulatorFeedbackPosition = -K*actualState;

    std::cout <<"Kuupa:"<< K<<std::endl;
    std::cout <<"Sprzenzenie:"<< regulatorFeedbackPosition<<std::endl;

}

void LQRHandler::calculateError()
{

    this->error = this->desiredPosition + this->regulatorFeedbackPosition;

    std::cout <<"Uhyp:"<< error<<std::endl;
}




void LQRHandler::update()
{
    loadEigenPositions();
    calculateKMatrix();// Ackchyually let matlab calculate this shit
    if(this->receivedNewK)
    {
        calculateRegulatorFeedbackPose();
        calculateError();
        this->rov.thrust_allocation(error);
        this->simulationResultState = /*actualState +*/ this->rov.getFutureState(this->actualState,this->A,this->B,rov.getThrustSignal(),deltaT);

        std::cout <<"Mamy"<< simulationResultState<<std::endl;
        this->rovPosition->setPositionAndVelocity("current",{simulationResultState[0],
                                                             simulationResultState[1],
                                                             simulationResultState[2],
                                                             simulationResultState[3],
                                                             simulationResultState[4],
                                                             simulationResultState[5],
                                                             simulationResultState[6],
                                                             simulationResultState[7],
                                                             simulationResultState[8],
                                                             simulationResultState[9],
                                                             simulationResultState[10],
                                                             simulationResultState[11]});

        emit positionReady();
        actualState = simulationResultState;
        this->receivedNewK = false;
    }
}
