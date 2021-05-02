//
// Created by filip on 25.02.2020.
//

#ifndef CONTROL_SYSTEM_ROV_H
#define CONTROL_SYSTEM_ROV_H

#include <Eigen/Dense>
#include "typedefs.h"
using namespace Eigen;

class ROV {
private:

    // Mass and inertia moments
    int m;
    double Ix, Iy, Iz;
    double Ixy, Iyx, Ixz, Izx, Iyz, Izy;
    Matrix3d Ib = Matrix3d::Zero(3,3);

    //Thrust configuration matrix;
    MatrixXd T = MatrixXd::Zero(6,5);
    VectorXd t1 = VectorXd::Zero(6,1);
    VectorXd t2 = VectorXd::Zero(6,1);
    VectorXd t3 = VectorXd::Zero(6,1);
    VectorXd t4 = VectorXd::Zero(6,1);
    VectorXd t5 = VectorXd::Zero(6,1);
    VectorXd u = VectorXd::Zero(5,1);

    MatrixXd KAll = MatrixXd::Zero(5,5);

    double alpha01 = 0.0;
    double alpha02 = 0.0;

    //Center of Gravity
    Vector3d rg = Vector3d::Zero(3);

    //Weight and buoyancy
    double W;
    double B;

    //MRB and Ma
    Matrix<double,6,6> Mrb = Matrix<double,6,6>::Zero(6,6);
    Matrix<double, 6,6> Ma = Matrix<double,6,6>::Zero(6,6);

    //Coeffs. of drag
    double Xu,Yv,Zw,Kp,Mq,Nr;
    double Xuu, Yvv, Zww, Kpp, Mqq, Nrr;
    double Xua, Yva, Zwa, Kpa, Mqa, Nra;
    VectorXd vl = VectorXd::Zero(6);
    VectorXd vnl = VectorXd::Zero(6);
    
    //Diagonal matrices of coeffs
    MatrixXd Dl = MatrixXd::Zero(6,6);
    MatrixXd Dnl = MatrixXd::Zero(6,6);

    //Rate of angular acceleration of thruster. Used in thrust allocation
    double deltaU;


    static Matrix3d Smtrx(Vector3d r);  //Function creating a special kind of matrix
    void init_geometry();               //Initializing mass, inertia moments, rg
    void init_drag();                   //initializing drag matrices 
    void init_thrust();
    VectorXd getRestoringForces(VectorXd currentState); //Getting restoring forces vector

public:
    ROV();                                                   //Constructor initializing variables
   // VectorXd states = VectorXd::Zero(12);
    Matrix<double,6,6> coriolis_matrix(VectorXd cur_state);
    Matrix<double, 12, 12> A_state_matrix(VectorXd cur_state);
    Matrix<double, 12, 6> B_state_matrix();
    void thrust_allocation(VectorXd tau);
    VectorXd getThrustSignal() const;
    VectorXd getAzimuth() const;
    VectorXd getFutureState(VectorXd currentState, Matrix1212 A, Matrix126 B, double deltaT);
    MatrixXd getNbar(Matrix1212 A, Matrix126 B, Matrix612 K);
};


#endif //CONTROL_SYSTEM_ROV_H
