//
// Created by filip on 25.02.2020.
//
#include <iostream>
#include "Eigen/Dense"
#include "ROV.h"
#include <cmath>
#include <EigenQP.h>

//Constructor initializing variables
ROV::ROV(){
    init_geometry();
    init_drag();
    init_thrust();
}


//Function creating a special kind of matrix
//Its definition can be found in Fossen.
Matrix3d ROV::Smtrx(Eigen::Vector3d r) {
    Eigen::Matrix3d mtrx;
    mtrx << 0, -r(2), r(1),
    r(2), 0, -r(0),
    -r(1), r(0), 0;
    return mtrx;
}


//Initializing mass, inertia moments, rg 
void ROV::init_geometry(){
    //Inertia moments
    Ix = 0.3; Iy = 0.3; Iz = 0.3;
    Ixy=0; Iyx=0; Ixz=0; Izx=0; Iyz=0; Izy=0;

    //Moments of inertia matrix
    Ib << Ix, -Ixy, -Ixz,
            -Iyx, Iy, -Iyz,
            -Izx, -Izy, Iz;

    //Mass
    m = 19;
    //Location of CoG in relation to Center of Origin of axes. It is set to 0.02 in z as it is located at Center of Buoy.
    rg << 0,0,0.02;

    Mrb.block(0,0,3,3) = m*Matrix3d::Identity(3,3);
    Mrb.block(0,3,3,3) = -m*Smtrx(rg);
    Mrb.block(3,0,3,3) = m*Smtrx(rg);
    Mrb.block(3,3,3,3) = Ib;

}

//Initializing drag matrices
void ROV::init_drag(){
    //Coeffs. of linear drag
        Xu = -4.03;
        Yv = -6.22;
        Zw = 5.18;
        Kp = -0.07;
        Mq = -0.07;
        Nr = -0.07;

    //Coeffs. of quadratic drag
        Xuu = -18.18;
        Yvv = -21.66;
        Zww = 36.99;
        Kpp = -1.55;
        Mqq = -1.55;
        Nrr = -1.55;

    //Creating diagonal matrices
        vl << Xu,Yv,Zw,Kp,Mq,Nr;
        vnl << Xuu,Yvv,Zww,Kpp,Mqq,Nrr;
        Dl = vl.asDiagonal();
        Dnl = vnl.asDiagonal();
}

//Initializing thrust configuration matrix
void ROV::init_thrust(){
    alpha01 = 0;
    alpha02 = 0;

    t1 << cos(alpha01),sin(alpha01),0,0,0,(sin(alpha01) * (-0.04)) - (cos(alpha01) * (-0.08));
    t2 << cos(alpha02),sin(alpha02),0,0,0,(sin(alpha02) * (-0.04)) - (cos(alpha02) * (0.08));
    t3 << 0,0,1,-0.11,-0.14,0;
    t4 << 0,0,-1,-0.11,0.14,0;
    t5 << 0,0,1,0,0.23,0;


    T << t1,t2,t3,t4,t5;

}


//Function for creating Coriolis forces matrix
//Its definition can be found in Fossen
//I've only rewritten it into C++
Matrix<double, 6, 6> ROV::coriolis_matrix(VectorXd cur_state) {
    //Initializing parameters
    Matrix<double,6,6> Crb;    
    Matrix<double,6,1> speed;
    Matrix3d M11, M12, M21, M22;
    Vector3d nu1, nu2;

    //Splitting linear and angular velocities into 2 vectors
    speed = cur_state.block(6,0,6,1);
    nu1 = speed.block(0,0,3,1);
    nu2 = speed.block(3,0,3,1);

    //Creating sub-matrices derived from rigid-body mass matrix
    Mrb = 0.5*(Mrb * Mrb.transpose());  //Making matrix square
    M11 = Mrb.topLeftCorner(3,3);
    M12 = Mrb.topRightCorner(3,3);
    M21 = Mrb.bottomLeftCorner(3,3);
    M22 = Mrb.bottomRightCorner(3,3);

    //Creating Coriolis forces matrix
    Crb << Matrix3d::Zero(3,3), -Smtrx(M11*nu1 + M12*nu2),
            -Smtrx(M11*nu1 + M12*nu2), -Smtrx(M21*nu1 + M22*nu2);

    return Crb;
}


//Function for creating State Space A matrix.
//Its definition, sizes and elements are defined in documentation
Matrix<double, 12, 12> ROV::A_state_matrix(VectorXd cur_state) {
    Matrix<double,12,12> A = MatrixXd::Zero(12,12);
    Matrix<double,6,1> speed = MatrixXd::Zero(6,1);
    Matrix<double,6,6> damping_coeffs = MatrixXd::Zero(6,6);
    MatrixXd speed_diag = MatrixXd::Zero(6,6);

    //Obtaining velocity vector and putting it as diagonal into a speed_diag matrix
    speed = cur_state.block(6,0,6,1);
    speed_diag = speed.asDiagonal();

    //This definition can also be found in documentation
    //First I create damping_coeffs matrix which is the sum
    //Of all elements which create opposing forces
    //Then I divide it by -M matrix which comes from State Space equation
    damping_coeffs = Dnl * speed_diag + coriolis_matrix(cur_state) + Dl;
    damping_coeffs = -Mrb.inverse() * damping_coeffs;

    //State Space matrix
    A << MatrixXd::Zero(6,6), MatrixXd::Zero(6,6),
        MatrixXd::Zero(6,6), damping_coeffs;

    return A;



}

//Function for creating State Space matrix B
//Its definition can also be found in the documentation.
//It can be represented as:
//B = [0
//    1/M]
Matrix<double, 12, 6> ROV::B_state_matrix() {
    Matrix<double, 12,6> B = MatrixXd::Zero(12,6);
    B.block(6,0,6,6) = Mrb.inverse();
    return B;
}

void ROV::thrust_allocation(VectorXd tau) {
    //Initializing thrust conf. matrix for azimuthal thrusters
    MatrixXd T1(3,1);
    MatrixXd T2(3,1);
    MatrixXd T_azimuth(3,2);    //Thrust conf. matrix for 2 azimuthal thrusters, including only x,y,yaw forces
    VectorXd tau_desired(3,1);  //Vector of desired forces  and moments: x,y,yaw
    T1 << t1(0), t1(1), t1(5);  //t1 and t2 are global thrust conf. matrices including sin and cos
    T2 << t2(0), t2(1), t2(5);
    T_azimuth << T1,T2;

    tau_desired << tau(0), tau(1), tau(5);


    //Constraints
    double delta_a = 0.01;      //Speed of servo - the angle which it turns by in 1 timestep

    double u_min = -0.4;        //delta u which means how fast the force can grow in 1 timestep
    double u_max = 0.4;

    //Cost matrices for quad prog
    VectorXd Q(3);      //Penalizing the difference between desired tau and generated one
    VectorXd Om(2);     //Penalizing too fast turn rate - not really important
    VectorXd W(2);      //Penalizing the power consumption of motors. Not really important as it's taken care of in LQR
    Q << 100,100,100;
    Om << 1, 1;
    W << 300,300;

    //Diagonal matrix H which is main matrix in quadprog problem. x^T * H * X + f*X
    VectorXd diag_H(7);
    diag_H << 2*W, 2*Q, 2*Om;
    MatrixXd H = MatrixXd::Zero(7,7);
    H = diag_H.asDiagonal();

    //Vector of linearity in quadprog as seen before
    VectorXd f = VectorXd::Zero(7);


    //Calculating derivatives for linearization
    MatrixXd da1 = MatrixXd::Zero(3,1);
    MatrixXd da2 = MatrixXd::Zero(3,1);
    MatrixXd diff_T = MatrixXd::Zero(3,2);

    //First and second azimuthal thruster. Below are calculated derivatives of thrust. conf. matrices
    da1 << -sin(alpha01) * u(0), cos(alpha01) * u(0), ((-0.08*sin(alpha01)) - (0.04 * cos(alpha01))) * u(0);
    da2 << -sin(alpha02) * u(1), cos(alpha02) * u(1), ((0.08*sin(alpha02)) - (0.04 * cos(alpha02))) * u(1);

    diff_T << da1,da2;


    //Equality constraints for QP
    //In Matlab there is only Aeq and beq. Here I need to pass Aeq^T to the function so I calculate it's transpose
    MatrixXd Aeq = MatrixXd::Zero(7,3);
    MatrixXd temp_Aeq = MatrixXd::Zero(3,7);    //Matrix which looks identical to that one from Matlab

    temp_Aeq.block(0,0,3,2) = T_azimuth;
    Vector3d v_diag(1,1,1);
    temp_Aeq.block(0,2,3,3) = v_diag.asDiagonal();
    temp_Aeq.block(0,5,3,2) = diff_T;

    Aeq = temp_Aeq.transpose();

    //Also the same as Matlab
    MatrixXd Beq;
    Beq = -(tau_desired - (T_azimuth * u.block(0,0,2,1)));

    //Inequality constraints
    //I need to specify lower and upper bounds for the variables
    //The difference between Matlab and this library is that in matlab the function looks like
    //lb < x < ub
    //Here it looks like
    //Ci^T * X + ci0 >= 0
    //So I needed to create a matrix Ci which gives vector of both +-u, +-s, +-alpha
    //And ci0 vector which corresponds to proper values of bounds
    MatrixXd Lb = MatrixXd::Zero(7,7);
    MatrixXd Ub = MatrixXd::Zero(7,7);
    VectorXd vec_ones(7);
    //The same as in Aeq - I pass tranposed version of matrix so I need to create temp_Ci matrix
    MatrixXd Ci = MatrixXd::Zero(7,14);
    MatrixXd temp_Ci = MatrixXd::Zero(14,7);
    VectorXd ci0 = VectorXd::Zero(14,1);
    vec_ones << 1,1,0,0,0,1,1;  //u,u,s,s,s,a,a
    Lb = vec_ones.asDiagonal(); //Lower bound
    Ub = -Lb;                   //Upper bound
    temp_Ci << Lb,Ub;
    Ci = temp_Ci.transpose();

    ci0 << -u_min, -u_min, 0,0,0,delta_a,delta_a,u_max, u_max,0,0,0,delta_a,delta_a; //Vector of bound valuses

    VectorXd x = VectorXd::Random(7); //Initializing solution vector

    //Solving and printing quadprog
    std::cout << "Solve quadprog:" << QP::solve_quadprog(H,f,Aeq,Beq,Ci,ci0,x) << std::endl;
    std::cout << "x= " << std::endl << x << std::endl;

    u(0) += x(0);   //Adding values of calculated change in force
    u(1) += x(1);

    alpha01 += x(5);      //And calculated change in servo angle
    alpha02 += x(6);


    //Classical THRUST ALLOCATION
    //Here I solve thrust allocation problem in classical way for forces in z,roll,pitch, for other 3 thrusters
    MatrixXd Thrust_conf(6,3);  //Matrix for only 3 thrusters
    MatrixXd Thrust_conf_inv;         //Its pseudoinverse
    Thrust_conf << t3,t4,t5;
    Thrust_conf_inv = Thrust_conf.completeOrthogonalDecomposition().pseudoInverse();

    //Matrix of maximum values of thrust force
    Vector3d diag_K(40,40,40);
    Matrix3d K;
    K = diag_K.asDiagonal();

    //Desired tau for this thrust allocation
    VectorXd tau_c(6);
    tau_c << 0,0,tau(2),tau(3),tau(4),0;

    //Final calculated vector of control signal
    Vector3d u2;
    u2 = K.inverse() * Thrust_conf_inv * tau_c;

    //Making sure that we cannot demand 110% of power
    for(int i =0; i<=2; i++){
        if(u(i) > 1){
            u(i) = 1;
        } else if (u(i) < -1){
            u(i) = -1;
        }
    }

    //Final vector u which is vector of all control signals for all thrusters
    u(2) = u2(0);
    u(3) = u2(1);
    u(4) = u2(2);

    std::cout << "Alpha 01: " << alpha01 << " alpha 02: " << alpha02 << std::endl;
    std::cout << "u = " << u << std::endl;

}
