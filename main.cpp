#include "gsmainwindow.h"

#include <QApplication>
/*#include "ROV.h"
#include <Eigen/Core>
#include <ct/optcon/optcon.h>
#include <cmath>
#include <EigenQP.h>
#include <Eigen/Dense>
using namespace Eigen;*/


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    GSMainWindow w;

    /*ROV robot;
    const size_t state_dim = 12;
        const size_t control_dim = 6;

        ct::optcon::LQR<state_dim,control_dim> lqrSolver;   //Initializing lqr solver

        Matrix<double,6,12> K;                              //K matrix which corresponds to gain of regulator
        VectorXd v = VectorXd::Zero(12);               //Test state vector

        Matrix<double,state_dim,state_dim> A;               //A state space matrix
        Matrix<double,state_dim,control_dim> B;             //B state space matrix

        A = robot.A_state_matrix(v);
        B = robot.B_state_matrix();
        MatrixXd Q = MatrixXd::Random(12,12);
            MatrixXd R = MatrixXd::Random(6,6);

            //Computing K matrix
            lqrSolver.compute(Q,R,A,B,K);

std::cout<<K;*/
    w.show();

    return a.exec();
}
