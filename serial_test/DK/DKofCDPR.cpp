#include "mainwindow.h"
#include <iostream>
#include "iMath.h"
#include "optimization.h"
#include <cmath>
#include "ParaofCDPR.h"   //Include para of CDPR.
#include "DKofCDPR.h"     //Include direct kinematic function of CDPR.
#include <Eigen/Core>
#include "stdio.h"
#include "stdlib.h"
#include "stdafx.h"
#include "cablelencal.h"

#define pi 3.14159265354

using namespace std;
using namespace alglib;

double length[8]={0.0};

void  function1_fvec(const real_1d_array &x, real_1d_array &fi, void *ptr)
{
    vector<vector<double>> Rx={  {1,         0,          0,            0},   \
                                 {0,         cos(x[3]),  -sin(x[3]),   0},   \
                                 {0,         sin(x[3]),  cos(x[3]),    0},   \
                                 {0,         0,          0,            1}};

    vector<vector<double>> Ry={  {cos(x[4]), 0,          sin(x[4]),    0},   \
                                 {0,         1,          0,            0},   \
                                 {-sin(x[4]),0,          cos(x[4]),    0},   \
                                 {0,         0,          0,            1}};

    vector<vector<double>> Rz={  {cos(x[5]), -sin(x[5]), 0,            0},   \
                                 {sin(x[5]), cos(x[5]),  0,            0},   \
                                 {0,         0,          1,            0},   \
                                 {0,         0,          0,            1}};

    vector<vector<double>> Ptrf={{1,0,0,x[0]},{0,1,0,x[1]},{0,0,1,x[2]},{0,0,0,1}};

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

    double bp1[] = {bp[0][0],bp[1][0],bp[2][0]};
    double bp2[] = {bp[0][1],bp[1][1],bp[2][1]};
    double bp3[] = {bp[0][2],bp[1][2],bp[2][2]};
    double bp4[] = {bp[0][3],bp[1][3],bp[2][3]};
    double bp5[] = {bp[0][4],bp[1][4],bp[2][4]};
    double bp6[] = {bp[0][5],bp[1][5],bp[2][5]};
    double bp7[] = {bp[0][6],bp[1][6],bp[2][6]};
    double bp8[] = {bp[0][7],bp[1][7],bp[2][7]};

    double G_ep1[] = {G_ep[0][0],G_ep[1][0],G_ep[2][0]};
    double G_ep2[] = {G_ep[0][1],G_ep[1][1],G_ep[2][1]};
    double G_ep3[] = {G_ep[0][2],G_ep[1][2],G_ep[2][2]};
    double G_ep4[] = {G_ep[0][3],G_ep[1][3],G_ep[2][3]};
    double G_ep5[] = {G_ep[0][4],G_ep[1][4],G_ep[2][4]};
    double G_ep6[] = {G_ep[0][5],G_ep[1][5],G_ep[2][5]};
    double G_ep7[] = {G_ep[0][6],G_ep[1][6],G_ep[2][6]};
    double G_ep8[] = {G_ep[0][7],G_ep[1][7],G_ep[2][7]};


    fi[0]=CableLenCal(G_ep1, bp1, pulleyRad[0]) - length[0];
    fi[1]=CableLenCal(G_ep2, bp2, pulleyRad[1]) - length[1];
    fi[2]=CableLenCal(G_ep3, bp3, pulleyRad[2]) - length[2];
    fi[3]=CableLenCal(G_ep4, bp4, pulleyRad[3]) - length[3];
    fi[4]=CableLenCal(G_ep5, bp5, pulleyRad[4]) - length[4];
    fi[5]=CableLenCal(G_ep6, bp6, pulleyRad[5]) - length[5];
    fi[6]=CableLenCal(G_ep7, bp7, pulleyRad[6]) - length[6];
    fi[7]=CableLenCal(G_ep8, bp8, pulleyRad[7]) - length[7];

}


void DKofCDPR(double *MotorAngle, double *Pose)//angle unit: rad
{
    length[0] = pow(pow((dia[0]*MotorAngle[0])/2,2),0.5);
    length[1] = pow(pow((dia[1]*MotorAngle[1])/2,2),0.5);
    length[2] = pow(pow((dia[2]*MotorAngle[2])/2,2),0.5);
    length[3] = pow(pow((dia[3]*MotorAngle[3])/2,2),0.5);
    length[4] = pow(pow((dia[4]*MotorAngle[4])/2,2),0.5);
    length[5] = pow(pow((dia[5]*MotorAngle[5])/2,2),0.5);
    length[6] = pow(pow((dia[6]*MotorAngle[6])/2,2),0.5);
    length[7] = pow(pow((dia[7]*MotorAngle[7])/2,2),0.5);

    real_1d_array x = "[0,0,0,0,0,0]";
    real_1d_array bndl = "[-500,-500,0,-1.0,-1.0,-1.0]";
    real_1d_array bndu = "[500,500,800, 1.0, 1.0, 1.0]";
    double epsx = 1E-6;
    ae_int_t maxits = 1000;
    minlmstate state;
    minlmreport rep;

    minlmcreatev(8, x, 0.0000001, state);
    minlmsetbc(state, bndl, bndu);
    minlmsetcond(state, epsx, maxits);
    alglib::minlmoptimize(state, function1_fvec);
    minlmresults(state, x, rep);

    for(short i=0;i<6;i++)
    {
        Pose[i] = x.operator ()(i);
    }
}
