#include "forcedistribute.h"
#include<Eigen/Dense>
#include "mycommand.h"
#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "optimization.h"
#include "mainwindow.h"
#include <iostream>
#include "iMath.h"
#include <vector>

using namespace std;
using namespace Eigen;
using namespace alglib;

void ExpEndForceCal(VectorXf d_p, VectorXf dd_p,MatrixXf K,MatrixXf B,double *EndForce)//calculate end force.
{

    VectorXf endforce(6);

    endforce = K*d_p + B*dd_p;

    for(short i=0;i<6;i++)
    {
        EndForce[i] = endforce(i);
        //        cout << EndForce[i]<< endl;
    }
}

void  nlcfunc2_jac(const real_1d_array &x, real_1d_array &fi, real_2d_array &jac, void *ptr)
{
    //** calculate dir of cables force.
    vector<vector<double>> Rx={  {1,         0,          0,            0},   \
                                 {0,         cos(RealPose[3]),  -sin(RealPose[3]),   0},   \
                                 {0,         sin(RealPose[3]),  cos(RealPose[3]),    0},   \
                                 {0,         0,          0,            1}};

    vector<vector<double>> Ry={  {cos(RealPose[4]), 0,          sin(RealPose[4]),    0},   \
                                 {0,         1,          0,            0},   \
                                 {-sin(RealPose[4]),0,          cos(RealPose[4]),    0},   \
                                 {0,         0,          0,            1}};

    vector<vector<double>> Rz={  {cos(RealPose[5]), -sin(RealPose[5]), 0,            0},   \
                                 {sin(RealPose[5]), cos(RealPose[5]),  0,            0},   \
                                 {0,         0,          1,            0},   \
                                 {0,         0,          0,            1}};

    vector<vector<double>> Ptrf={{1,0,0,RealPose[0]},{0,1,0,RealPose[1]},{0,0,1,RealPose[2]},{0,0,0,1}};

    vector<vector<double>> T = matrix_multiply(Ptrf, matrix_multiply(Rz, matrix_multiply(Ry, Rx)));

    vector<vector<double> > V_ep(4,vector<double>(8));

    for(int i=0;i<4;i++)
    {
        for (int j = 0; j < 8;j++)
        {
            V_ep[i][j]=ep[i][j];
        }
    }
    vector<vector<double>> G_ep = matrix_multiply(T, V_ep);

    double bp1[3] = {bp[0][0],bp[1][0],bp[2][0]};
    double bp2[3] = {bp[0][1],bp[1][1],bp[2][1]};
    double bp3[3] = {bp[0][2],bp[1][2],bp[2][2]};
    double bp4[3] = {bp[0][3],bp[1][3],bp[2][3]};
    double bp5[3] = {bp[0][4],bp[1][4],bp[2][4]};
    double bp6[3] = {bp[0][5],bp[1][5],bp[2][5]};
    double bp7[3] = {bp[0][6],bp[1][6],bp[2][6]};
    double bp8[3] = {bp[0][7],bp[1][7],bp[2][7]};

    double G_ep1[3] = {G_ep[0][0],G_ep[1][0],G_ep[2][0]};
    double G_ep2[3] = {G_ep[0][1],G_ep[1][1],G_ep[2][1]};
    double G_ep3[3] = {G_ep[0][2],G_ep[1][2],G_ep[2][2]};
    double G_ep4[3] = {G_ep[0][3],G_ep[1][3],G_ep[2][3]};
    double G_ep5[3] = {G_ep[0][4],G_ep[1][4],G_ep[2][4]};
    double G_ep6[3] = {G_ep[0][5],G_ep[1][5],G_ep[2][5]};
    double G_ep7[3] = {G_ep[0][6],G_ep[1][6],G_ep[2][6]};
    double G_ep8[3] = {G_ep[0][7],G_ep[1][7],G_ep[2][7]};

    double dir_1[3] = {bp1[0] - G_ep1[0],bp1[1] - G_ep1[1],bp1[2] - G_ep1[2]};
    Map<VectorXd> l_dir_1(dir_1,3);
    double dir_2[3] = {bp2[0] - G_ep2[0],bp2[1] - G_ep2[1],bp2[2] - G_ep2[2]};
    Map<VectorXd> l_dir_2(dir_2,3);
    double dir_3[3] = {bp3[0] - G_ep3[0],bp3[1] - G_ep3[1],bp3[2] - G_ep3[2]};
    Map<VectorXd> l_dir_3(dir_3,3);
    double dir_4[3] = {bp4[0] - G_ep4[0],bp4[1] - G_ep4[1],bp4[2] - G_ep4[2]};
    Map<VectorXd> l_dir_4(dir_4,3);
    double dir_5[3] = {bp5[0] - G_ep5[0],bp5[1] - G_ep5[1],bp5[2] - G_ep5[2]};
    Map<VectorXd> l_dir_5(dir_5,3);
    double dir_6[3] = {bp6[0] - G_ep6[0],bp6[1] - G_ep6[1],bp6[2] - G_ep6[2]};
    Map<VectorXd> l_dir_6(dir_6,3);
    double dir_7[3] = {bp7[0] - G_ep7[0],bp7[1] - G_ep7[1],bp7[2] - G_ep7[2]};
    Map<VectorXd> l_dir_7(dir_7,3);
    double dir_8[3] = {bp8[0] - G_ep8[0],bp8[1] - G_ep8[1],bp8[2] - G_ep8[2]};
    Map<VectorXd> l_dir_8(dir_8,3);

    for(int i = 0;i<3;i++)
    {
        dir_1[i] = dir_1[i]/l_dir_1.norm();
        dir_2[i] = dir_2[i]/l_dir_2.norm();
        dir_3[i] = dir_3[i]/l_dir_3.norm();
        dir_4[i] = dir_4[i]/l_dir_4.norm();
        dir_5[i] = dir_5[i]/l_dir_5.norm();
        dir_6[i] = dir_6[i]/l_dir_6.norm();
        dir_7[i] = dir_7[i]/l_dir_7.norm();
        dir_8[i] = dir_8[i]/l_dir_8.norm();
    }

    double r_1[3] = {G_ep1[0] - RealPose[0],G_ep1[1] - RealPose[1],G_ep1[2] - RealPose[2]};
    double r_2[3] = {G_ep2[0] - RealPose[0],G_ep2[1] - RealPose[1],G_ep2[2] - RealPose[2]};
    double r_3[3] = {G_ep3[0] - RealPose[0],G_ep3[1] - RealPose[1],G_ep3[2] - RealPose[2]};
    double r_4[3] = {G_ep4[0] - RealPose[0],G_ep4[1] - RealPose[1],G_ep4[2] - RealPose[2]};
    double r_5[3] = {G_ep5[0] - RealPose[0],G_ep5[1] - RealPose[1],G_ep5[2] - RealPose[2]};
    double r_6[3] = {G_ep6[0] - RealPose[0],G_ep6[1] - RealPose[1],G_ep6[2] - RealPose[2]};
    double r_7[3] = {G_ep7[0] - RealPose[0],G_ep7[1] - RealPose[1],G_ep7[2] - RealPose[2]};
    double r_8[3] = {G_ep8[0] - RealPose[0],G_ep8[1] - RealPose[1],G_ep8[2] - RealPose[2]};


    fi[0] = x[0]*x[0]+x[1]*x[1]+x[2]*x[2]+x[3]*x[3]+x[4]*x[4]+x[5]*x[5]+x[6]*x[6]+x[7]*x[7];
    fi[1] = x[0]*dir_1[0] + x[1]*dir_2[0] + x[2]*dir_3[0] + x[3]*dir_4[0] + x[4]*dir_5[0] + x[5]*dir_6[0] + x[6]*dir_7[0] + x[7]*dir_8[0] - ExpectEndForce[0];
    fi[2] = x[0]*dir_1[1] + x[1]*dir_2[1] + x[2]*dir_3[1] + x[3]*dir_4[1] + x[4]*dir_5[1] + x[5]*dir_6[1] + x[6]*dir_7[1] + x[7]*dir_8[1] - ExpectEndForce[1];
    fi[3] = x[0]*dir_1[2] + x[1]*dir_2[2] + x[2]*dir_3[2] + x[3]*dir_4[2] + x[4]*dir_5[2] + x[5]*dir_6[2] + x[6]*dir_7[2] + x[7]*dir_8[2] - ExpectEndForce[2];

    double Tor1[3],Tor2[3],Tor3[3],Tor4[3],Tor5[3],Tor6[3],Tor7[3],Tor8[3];
    double Fc_1[3],Fc_2[3],Fc_3[3],Fc_4[3],Fc_5[3],Fc_6[3],Fc_7[3],Fc_8[3];

    Fc_1[0] = x[0]*dir_1[0];
    Fc_1[1] = x[0]*dir_1[1];
    Fc_1[2] = x[0]*dir_1[2];
    Fc_2[0] = x[1]*dir_2[0];
    Fc_2[1] = x[1]*dir_2[1];
    Fc_2[2] = x[1]*dir_2[2];
    Fc_3[0] = x[2]*dir_3[0];
    Fc_3[1] = x[2]*dir_3[1];
    Fc_3[2] = x[2]*dir_3[2];
    Fc_4[0] = x[3]*dir_4[0];
    Fc_4[1] = x[3]*dir_4[1];
    Fc_4[2] = x[3]*dir_4[2];
    Fc_5[0] = x[4]*dir_5[0];
    Fc_5[1] = x[4]*dir_5[1];
    Fc_5[2] = x[4]*dir_5[2];
    Fc_6[0] = x[5]*dir_6[0];
    Fc_6[1] = x[5]*dir_6[1];
    Fc_6[2] = x[5]*dir_6[2];
    Fc_7[0] = x[6]*dir_7[0];
    Fc_7[1] = x[6]*dir_7[1];
    Fc_7[2] = x[6]*dir_7[2];
    Fc_8[0] = x[7]*dir_8[0];
    Fc_8[1] = x[7]*dir_8[1];
    Fc_8[2] = x[7]*dir_8[2];

    m_cross3d(r_1,Fc_1,Tor1);
    m_cross3d(r_2,Fc_2,Tor2);
    m_cross3d(r_3,Fc_3,Tor3);
    m_cross3d(r_4,Fc_4,Tor4);
    m_cross3d(r_5,Fc_5,Tor5);
    m_cross3d(r_6,Fc_6,Tor6);
    m_cross3d(r_7,Fc_7,Tor7);
    m_cross3d(r_8,Fc_8,Tor8);

    fi[4] = Tor1[0] + Tor2[0] + Tor3[0] + Tor4[0] + Tor5[0] + Tor6[0] + Tor7[0] + Tor8[0] - ExpectEndForce[3];
    fi[5] = Tor1[1] + Tor2[1] + Tor3[1] + Tor4[1] + Tor5[1] + Tor6[1] + Tor7[1] + Tor8[1] - ExpectEndForce[4];
    fi[6] = Tor1[2] + Tor2[2] + Tor3[2] + Tor4[2] + Tor5[2] + Tor6[2] + Tor7[2] + Tor8[2] - ExpectEndForce[5];

    fi[7] = MinCalbleForce - x[0];
    fi[8] = MinCalbleForce - x[1];
    fi[9] = MinCalbleForce - x[2];
    fi[10] = MinCalbleForce - x[3];
    fi[11] = MinCalbleForce - x[4];
    fi[12] = MinCalbleForce - x[5];
    fi[13] = MinCalbleForce - x[6];
    fi[14] = MinCalbleForce - x[7];

    jac[0][0] = 2*x[0];
    jac[0][1] = 2*x[1];
    jac[0][2] = 2*x[2];
    jac[0][3] = 2*x[3];
    jac[0][4] = 2*x[4];
    jac[0][5] = 2*x[5];
    jac[0][6] = 2*x[6];
    jac[0][7] = 2*x[7];
    jac[1][0] = dir_1[0];
    jac[1][1] = dir_2[0];
    jac[1][2] = dir_3[0];
    jac[1][3] = dir_4[0];
    jac[1][4] = dir_5[0];
    jac[1][5] = dir_6[0];
    jac[1][6] = dir_7[0];
    jac[1][7] = dir_8[0];
    jac[2][0] = dir_1[1];
    jac[2][1] = dir_2[1];
    jac[2][2] = dir_3[1];
    jac[2][3] = dir_4[1];
    jac[2][4] = dir_5[1];
    jac[2][5] = dir_6[1];
    jac[2][6] = dir_7[1];
    jac[2][7] = dir_8[1];
    jac[3][0] = dir_1[2];
    jac[3][1] = dir_2[2];
    jac[3][2] = dir_3[2];
    jac[3][3] = dir_4[2];
    jac[3][4] = dir_5[2];
    jac[3][5] = dir_6[2];
    jac[3][6] = dir_7[2];
    jac[3][7] = dir_8[2];
    jac[4][0] = dir_1[2]*r_1[1] - dir_1[1]*r_1[2];
    jac[4][1] = dir_2[2]*r_2[1] - dir_2[1]*r_2[2];
    jac[4][2] = dir_3[2]*r_3[1] - dir_3[1]*r_3[2];
    jac[4][3] = dir_4[2]*r_4[1] - dir_4[1]*r_4[2];
    jac[4][4] = dir_5[2]*r_5[1] - dir_5[1]*r_5[2];
    jac[4][5] = dir_6[2]*r_6[1] - dir_6[1]*r_6[2];
    jac[4][6] = dir_7[2]*r_7[1] - dir_7[1]*r_7[2];
    jac[4][7] = dir_8[2]*r_8[1] - dir_8[1]*r_8[2];
    jac[5][0] = dir_1[0]*r_1[2] - dir_1[2]*r_1[0];
    jac[5][1] = dir_2[0]*r_2[2] - dir_2[2]*r_2[0];
    jac[5][2] = dir_3[0]*r_3[2] - dir_3[2]*r_3[0];
    jac[5][3] = dir_4[0]*r_4[2] - dir_4[2]*r_4[0];
    jac[5][4] = dir_5[0]*r_5[2] - dir_5[2]*r_5[0];
    jac[5][5] = dir_6[0]*r_6[2] - dir_6[2]*r_6[0];
    jac[5][6] = dir_7[0]*r_7[2] - dir_7[2]*r_7[0];
    jac[5][7] = dir_8[0]*r_8[2] - dir_8[2]*r_8[0];
    jac[6][0] = dir_1[1]*r_1[0] - dir_1[0]*r_1[1];
    jac[6][1] = dir_2[1]*r_2[0] - dir_2[0]*r_2[1];
    jac[6][2] = dir_3[1]*r_3[0] - dir_3[0]*r_3[1];
    jac[6][3] = dir_4[1]*r_4[0] - dir_4[0]*r_4[1];
    jac[6][4] = dir_5[1]*r_5[0] - dir_5[0]*r_5[1];
    jac[6][5] = dir_6[1]*r_6[0] - dir_6[0]*r_6[1];
    jac[6][6] = dir_7[1]*r_7[0] - dir_7[0]*r_7[1];
    jac[6][7] = dir_8[1]*r_8[0] - dir_8[0]*r_8[1];
    jac[7][0] = -1.0;
    jac[7][1] = 0.0;
    jac[7][2] = 0.0;
    jac[7][3] = 0.0;
    jac[7][4] = 0.0;
    jac[7][5] = 0.0;
    jac[7][6] = 0.0;
    jac[7][7] = 0.0;
    jac[8][0] = 0.0;
    jac[8][1] = -1.0;
    jac[8][2] = 0.0;
    jac[8][3] = 0.0;
    jac[8][4] = 0.0;
    jac[8][5] = 0.0;
    jac[8][6] = 0.0;
    jac[8][7] = 0.0;
    jac[9][0] = 0.0;
    jac[9][1] = 0.0;
    jac[9][2] = -1.0;
    jac[9][3] = 0.0;
    jac[9][4] = 0.0;
    jac[9][5] = 0.0;
    jac[9][6] = 0.0;
    jac[9][7] = 0.0;
    jac[10][0] = 0.0;
    jac[10][1] = 0.0;
    jac[10][2] = 0.0;
    jac[10][3] = -1.0;
    jac[10][4] = 0.0;
    jac[10][5] = 0.0;
    jac[10][6] = 0.0;
    jac[10][7] = 0.0;
    jac[11][0] = 0.0;
    jac[11][1] = 0.0;
    jac[11][2] = 0.0;
    jac[11][3] = 0.0;
    jac[11][4] = -1.0;
    jac[11][5] = 0.0;
    jac[11][6] = 0.0;
    jac[11][7] = 0.0;
    jac[12][0] = 0.0;
    jac[12][1] = 0.0;
    jac[12][2] = 0.0;
    jac[12][3] = 0.0;
    jac[12][4] = 0.0;
    jac[12][5] = -1.0;
    jac[12][6] = 0.0;
    jac[12][7] = 0.0;
    jac[13][0] = 0.0;
    jac[13][1] = 0.0;
    jac[13][2] = 0.0;
    jac[13][3] = 0.0;
    jac[13][4] = 0.0;
    jac[13][5] = 0.0;
    jac[13][6] = -1.0;
    jac[13][7] = 0.0;
    jac[14][0] = 0.0;
    jac[14][1] = 0.0;
    jac[14][2] = 0.0;
    jac[14][3] = 0.0;
    jac[14][4] = 0.0;
    jac[14][5] = 0.0;
    jac[14][6] = 0.0;
    jac[14][7] = -1.0;
}


void EndForceDistrToJointCal(float *expecttesion)//calculate joint tesion.
{
    //    //
    //    // This example demonstrates minimization of
    //    //
    //    //     f(x0,x1) = x0+x1
    //    //
    //    // subject to nonlinear inequality constraint
    //    //
    //    //    x0^2 + x1^2 - 1 <= 0
    //    //
    //    // and nonlinear equality constraint
    //    //
    //    //    x2-exp(x0) = 0
    //    //
    //    real_1d_array x0 = "[0,0,0]";
    //    real_1d_array s = "[1,1,1]";
    //    double epsg = 0;
    //    double epsf = 0;
    //    double epsx = 0.000001;
    //    ae_int_t maxits = 0;
    //    ae_int_t outerits = 5;
    //    ae_int_t updatefreq = 10;
    //    double rho = 1000;
    //    minnlcstate state;
    //    minnlcreport rep;
    //    real_1d_array x1;

    //set initial value.
    QString x0_s = "[";
    x0_s.append(QString::number(ExpectTesion[0],'f',4));
    x0_s.append(",");
    x0_s.append(QString::number(ExpectTesion[1],'f',4));
    x0_s.append(",");
    x0_s.append(QString::number(ExpectTesion[2],'f',4));
    x0_s.append(",");
    x0_s.append(QString::number(ExpectTesion[3],'f',4));
    x0_s.append(",");
    x0_s.append(QString::number(ExpectTesion[4],'f',4));
    x0_s.append(",");
    x0_s.append(QString::number(ExpectTesion[5],'f',4));
    x0_s.append(",");
    x0_s.append(QString::number(ExpectTesion[6],'f',4));
    x0_s.append(",");
    x0_s.append(QString::number(ExpectTesion[7],'f',4));
    x0_s.append("]");

    QByteArray tmp = x0_s.toLatin1();
    char *a = tmp.data();

   // real_1d_array x0 = "[0,0,0,0,0,0,0,0]";
    real_1d_array x0 = a;
    real_1d_array s = "[1,1,1,1,1,1,1,1]";
    double epsg = 0;
    double epsf = 0;
    double epsx = 1E-5;
    ae_int_t maxits = 1000;
    ae_int_t outerits = 5;
    ae_int_t updatefreq = 10;
    double rho = 1000;
    minnlcstate state;
    minnlcreport rep;
    real_1d_array x1;

    //
    // Create optimizer object, choose AUL algorithm and tune its settings:
    // * rho=1000       penalty coefficient
    // * outerits=5     number of outer iterations to tune Lagrange coefficients
    // * epsx=0.000001  stopping condition for inner iterations
    // * s=[1,1]        all variables have unit scale
    // * exact low-rank preconditioner is used, updated after each 10 iterations
    // * upper limit on step length is specified (to avoid probing locations where exp() is large)
    //
    minnlccreate(8, x0, state);
    minnlcsetalgoaul(state, rho, outerits);
    minnlcsetcond(state, epsg, epsf, epsx, maxits);
    minnlcsetscale(state, s);
    minnlcsetprecexactlowrank(state, updatefreq);
    minnlcsetstpmax(state, 10.0);

    //
    // Set constraints:
    //
    // Nonlinear constraints are tricky - you can not "pack" general
    // nonlinear function into double precision array. That's why
    // minnlcsetnlc() does not accept constraints itself - only constraint
    // counts are passed: first parameter is number of equality constraints,
    // second one is number of inequality constraints.
    //
    // As for constraining functions - these functions are passed as part
    // of problem Jacobian (see below).
    //
    // NOTE: MinNLC optimizer supports arbitrary combination of boundary, general
    //       linear and general nonlinear constraints. This example does not
    //       show how to work with boundary or general linear constraints, but you
    //       can easily find it in documentation on minnlcsetbc() and
    //       minnlcsetlc() functions.

    minnlcsetnlc(state, 6, 8);


    // Optimize and test results.
    //
    // Optimizer object accepts vector function and its Jacobian, with first
    // component (Jacobian row) being target function, and next components
    // (Jacobian rows) being nonlinear equality and inequality constraints.
    //
    // So, our vector function has form
    //
    //     {f0,f1,f2} = { x0+x1 , x2-exp(x0) , x0^2+x1^2-1 }
    //
    // with Jacobian
    //
    //         [  +1      +1       0 ]
    //     J = [-exp(x0)  0        1 ]
    //         [ 2*x0    2*x1      0 ]
    //
    // with f0 being target function, f1 being equality constraint "f1=0",
    // f2 being inequality constraint "f2<=0". Number of equality/inequality
    // constraints is specified by minnlcsetnlc(), with equality ones always
    // being first, inequality ones being last.
    //
    alglib::minnlcoptimize(state, nlcfunc2_jac);
    minnlcresults(state, x1, rep);


    float calTesion[8];
    for(short i=0;i<8;i++)
    {
        calTesion[i] = x1.operator ()(i);
    }
    Map<VectorXf> LastTesion(ExpectTesion,8);
    Map<VectorXf> CurrentTesion(calTesion,8);

    VectorXf dTesion = LastTesion - CurrentTesion;

        for(short i = 0;i<8;i++)
        {
            if(i==0)
            {
                cout<<"calTesion: ";
            }
            cout<<calTesion[i]<<"\t";
            if(i == 7)
            {
                cout<<endl;
            }
        }

    float error = dTesion.norm();
    //    cout << "ExpectTesion 0: "<< ExpectTesion[0]<<endl;
    if(error < 100 | ExpectTesion[0] == 0.0)//
    {
        for(short i=0;i<8;i++)
        {
            if(5.0 < calTesion[i] && calTesion[i]<100.0)
            {
                expecttesion[i] = calTesion[i];
            }
            else
            {
                expecttesion[i] = 5.0;
            }
        }
        //        cout<<"cal tesion ok!!"<<endl;
    }
    else
    {
        for(short i=0;i<8;i++)
        {
            if(5.0 < ExpectTesion[i] && ExpectTesion[i]<100.0)
            {
                expecttesion[i] = ExpectTesion[i];
            }
            else
            {
                expecttesion[i] = 5.0;
            }
        }
    }
}

void JointTesionToEndCal(float *realtesion, double *realendforce)
{
    vector<vector<double>> Rx={  {1,         0,          0,            0},   \
                                 {0,         cos(RealPose[3]),  -sin(RealPose[3]),   0},   \
                                 {0,         sin(RealPose[3]),  cos(RealPose[3]),    0},   \
                                 {0,         0,          0,            1}};

    vector<vector<double>> Ry={  {cos(RealPose[4]), 0,          sin(RealPose[4]),    0},   \
                                 {0,         1,          0,            0},   \
                                 {-sin(RealPose[4]),0,          cos(RealPose[4]),    0},   \
                                 {0,         0,          0,            1}};

    vector<vector<double>> Rz={  {cos(RealPose[5]), -sin(RealPose[5]), 0,            0},   \
                                 {sin(RealPose[5]), cos(RealPose[5]),  0,            0},   \
                                 {0,         0,          1,            0},   \
                                 {0,         0,          0,            1}};

    vector<vector<double>> Ptrf={{1,0,0,RealPose[0]},{0,1,0,RealPose[1]},{0,0,1,RealPose[2]},{0,0,0,1}};

    vector<vector<double>> T = matrix_multiply(Ptrf, matrix_multiply(Rz, matrix_multiply(Ry, Rx)));

    vector<vector<double> > V_ep(4,vector<double>(8));

    for(int i=0;i<4;i++)
    {
        for (int j = 0; j < 8;j++)
        {
            V_ep[i][j]=ep[i][j];
        }
    }
    vector<vector<double>> G_ep = matrix_multiply(T, V_ep);

    double bp1[3] = {bp[0][0],bp[1][0],bp[2][0]};
    double bp2[3] = {bp[0][1],bp[1][1],bp[2][1]};
    double bp3[3] = {bp[0][2],bp[1][2],bp[2][2]};
    double bp4[3] = {bp[0][3],bp[1][3],bp[2][3]};
    double bp5[3] = {bp[0][4],bp[1][4],bp[2][4]};
    double bp6[3] = {bp[0][5],bp[1][5],bp[2][5]};
    double bp7[3] = {bp[0][6],bp[1][6],bp[2][6]};
    double bp8[3] = {bp[0][7],bp[1][7],bp[2][7]};

    double G_ep1[3] = {G_ep[0][0],G_ep[1][0],G_ep[2][0]};
    double G_ep2[3] = {G_ep[0][1],G_ep[1][1],G_ep[2][1]};
    double G_ep3[3] = {G_ep[0][2],G_ep[1][2],G_ep[2][2]};
    double G_ep4[3] = {G_ep[0][3],G_ep[1][3],G_ep[2][3]};
    double G_ep5[3] = {G_ep[0][4],G_ep[1][4],G_ep[2][4]};
    double G_ep6[3] = {G_ep[0][5],G_ep[1][5],G_ep[2][5]};
    double G_ep7[3] = {G_ep[0][6],G_ep[1][6],G_ep[2][6]};
    double G_ep8[3] = {G_ep[0][7],G_ep[1][7],G_ep[2][7]};

    double dir_1[3] = {bp1[0] - G_ep1[0],bp1[1] - G_ep1[1],bp1[2] - G_ep1[2]};
    Map<VectorXd> l_dir_1(dir_1,3);
    double dir_2[3] = {bp2[0] - G_ep2[0],bp2[1] - G_ep2[1],bp2[2] - G_ep2[2]};
    Map<VectorXd> l_dir_2(dir_2,3);
    double dir_3[3] = {bp3[0] - G_ep3[0],bp3[1] - G_ep3[1],bp3[2] - G_ep3[2]};
    Map<VectorXd> l_dir_3(dir_3,3);
    double dir_4[3] = {bp4[0] - G_ep4[0],bp4[1] - G_ep4[1],bp4[2] - G_ep4[2]};
    Map<VectorXd> l_dir_4(dir_4,3);
    double dir_5[3] = {bp5[0] - G_ep5[0],bp5[1] - G_ep5[1],bp5[2] - G_ep5[2]};
    Map<VectorXd> l_dir_5(dir_5,3);
    double dir_6[3] = {bp6[0] - G_ep6[0],bp6[1] - G_ep6[1],bp6[2] - G_ep6[2]};
    Map<VectorXd> l_dir_6(dir_6,3);
    double dir_7[3] = {bp7[0] - G_ep7[0],bp7[1] - G_ep7[1],bp7[2] - G_ep7[2]};
    Map<VectorXd> l_dir_7(dir_7,3);
    double dir_8[3] = {bp8[0] - G_ep8[0],bp8[1] - G_ep8[1],bp8[2] - G_ep8[2]};
    Map<VectorXd> l_dir_8(dir_8,3);

    for(int i = 0;i<3;i++)
    {
        dir_1[i] = dir_1[i]/l_dir_1.norm();
        dir_2[i] = dir_2[i]/l_dir_2.norm();
        dir_3[i] = dir_3[i]/l_dir_3.norm();
        dir_4[i] = dir_4[i]/l_dir_4.norm();
        dir_5[i] = dir_5[i]/l_dir_5.norm();
        dir_6[i] = dir_6[i]/l_dir_6.norm();
        dir_7[i] = dir_7[i]/l_dir_7.norm();
        dir_8[i] = dir_8[i]/l_dir_8.norm();
    }
    realendforce[0]  = realtesion[0]*dir_1[0]+realtesion[1]*dir_2[0]+realtesion[2]*dir_3[0]+realtesion[3]*dir_4[0]+\
            realtesion[4]*dir_5[0]+realtesion[5]*dir_6[0]+realtesion[6]*dir_7[0]+realtesion[7]*dir_8[0];
    realendforce[1]  = realtesion[0]*dir_1[1]+realtesion[1]*dir_2[1]+realtesion[2]*dir_3[1]+realtesion[3]*dir_4[1]+\
            realtesion[4]*dir_5[1]+realtesion[5]*dir_6[1]+realtesion[6]*dir_7[1]+realtesion[7]*dir_8[1];
    realendforce[2]  = realtesion[0]*dir_1[2]+realtesion[1]*dir_2[2]+realtesion[2]*dir_3[2]+realtesion[3]*dir_4[2]+\
            realtesion[4]*dir_5[2]+realtesion[5]*dir_6[2]+realtesion[6]*dir_7[2]+realtesion[7]*dir_8[2];

    double r_1[3] = {G_ep1[0] - RealPose[0],G_ep1[1] - RealPose[1],G_ep1[2] - RealPose[2]};
    double r_2[3] = {G_ep2[0] - RealPose[0],G_ep2[1] - RealPose[1],G_ep2[2] - RealPose[2]};
    double r_3[3] = {G_ep3[0] - RealPose[0],G_ep3[1] - RealPose[1],G_ep3[2] - RealPose[2]};
    double r_4[3] = {G_ep4[0] - RealPose[0],G_ep4[1] - RealPose[1],G_ep4[2] - RealPose[2]};
    double r_5[3] = {G_ep5[0] - RealPose[0],G_ep5[1] - RealPose[1],G_ep5[2] - RealPose[2]};
    double r_6[3] = {G_ep6[0] - RealPose[0],G_ep6[1] - RealPose[1],G_ep6[2] - RealPose[2]};
    double r_7[3] = {G_ep7[0] - RealPose[0],G_ep7[1] - RealPose[1],G_ep7[2] - RealPose[2]};
    double r_8[3] = {G_ep8[0] - RealPose[0],G_ep8[1] - RealPose[1],G_ep8[2] - RealPose[2]};

    double Tor1[3],Tor2[3],Tor3[3],Tor4[3],Tor5[3],Tor6[3],Tor7[3],Tor8[3];
    double Fc_1[3],Fc_2[3],Fc_3[3],Fc_4[3],Fc_5[3],Fc_6[3],Fc_7[3],Fc_8[3];

    Fc_1[0] = realtesion[0]*dir_1[0];
    Fc_1[1] = realtesion[0]*dir_1[1];
    Fc_1[2] = realtesion[0]*dir_1[2];
    Fc_2[0] = realtesion[1]*dir_2[0];
    Fc_2[1] = realtesion[1]*dir_2[1];
    Fc_2[2] = realtesion[1]*dir_2[2];
    Fc_3[0] = realtesion[2]*dir_3[0];
    Fc_3[1] = realtesion[2]*dir_3[1];
    Fc_3[2] = realtesion[2]*dir_3[2];
    Fc_4[0] = realtesion[3]*dir_4[0];
    Fc_4[1] = realtesion[3]*dir_4[1];
    Fc_4[2] = realtesion[3]*dir_4[2];
    Fc_5[0] = realtesion[4]*dir_5[0];
    Fc_5[1] = realtesion[4]*dir_5[1];
    Fc_5[2] = realtesion[4]*dir_5[2];
    Fc_6[0] = realtesion[5]*dir_6[0];
    Fc_6[1] = realtesion[5]*dir_6[1];
    Fc_6[2] = realtesion[5]*dir_6[2];
    Fc_7[0] = realtesion[6]*dir_7[0];
    Fc_7[1] = realtesion[6]*dir_7[1];
    Fc_7[2] = realtesion[6]*dir_7[2];
    Fc_8[0] = realtesion[7]*dir_8[0];
    Fc_8[1] = realtesion[7]*dir_8[1];
    Fc_8[2] = realtesion[7]*dir_8[2];

    m_cross3d(r_1,Fc_1,Tor1);
    m_cross3d(r_2,Fc_2,Tor2);
    m_cross3d(r_3,Fc_3,Tor3);
    m_cross3d(r_4,Fc_4,Tor4);
    m_cross3d(r_5,Fc_5,Tor5);
    m_cross3d(r_6,Fc_6,Tor6);
    m_cross3d(r_7,Fc_7,Tor7);
    m_cross3d(r_8,Fc_8,Tor8);

    realendforce[3] = Tor1[0] + Tor2[0] + Tor3[0] + Tor4[0] + Tor5[0] + Tor6[0] + Tor7[0] + Tor8[0];
    realendforce[4] = Tor1[1] + Tor2[1] + Tor3[1] + Tor4[1] + Tor5[1] + Tor6[1] + Tor7[1] + Tor8[1];
    realendforce[5] = Tor1[2] + Tor2[2] + Tor3[2] + Tor4[2] + Tor5[2] + Tor6[2] + Tor7[2] + Tor8[2];
}
