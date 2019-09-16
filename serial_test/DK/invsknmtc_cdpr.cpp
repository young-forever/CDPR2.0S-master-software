#include "invsknmtc_cdpr.h"
#include<Eigen/Core>
#include<iomanip>
#include "cablelencal.h"
#include<iostream>
using namespace Eigen;
using namespace std;
// input unit :m.
VectorXd InvsKnmtc_cdpr(VectorXd pose_ee, MatrixXd ibp, MatrixXd iep, VectorXd d, double *r)
{
    VectorXd a1_e = iep.col(0);
    VectorXd a2_e = iep.col(1);
    VectorXd a3_e = iep.col(2);
    VectorXd a4_e = iep.col(3);
    VectorXd a5_e = iep.col(4);
    VectorXd a6_e = iep.col(5);
    VectorXd a7_e = iep.col(6);
    VectorXd a8_e = iep.col(7);

    VectorXd b1_g = ibp.col(0);
    VectorXd b2_g = ibp.col(1);
    VectorXd b3_g = ibp.col(2);
    VectorXd b4_g = ibp.col(3);
    VectorXd b5_g = ibp.col(4);
    VectorXd b6_g = ibp.col(5);
    VectorXd b7_g = ibp.col(6);
    VectorXd b8_g = ibp.col(7);

    MatrixXd Trans_matrix(4,4),Tx(4,4),Ty(4,4),Tz(4,4),T(4,4);

    Trans_matrix <<     1,  0,  0,  pose_ee(0),
                        0,  1,  0,  pose_ee(1),
                        0,  0,  1,  pose_ee(2),
                        0,  0,  0,  1;

    Tx  <<  cos(pose_ee(5)),    -sin(pose_ee(5)),   0,  0,
            sin(pose_ee(5)),    cos(pose_ee(5)),    0,  0,
            0,                  0,                  1,  0,
            0,                  0,                  0,  1;

    Ty  <<  cos(pose_ee(4)),    0,  sin(pose_ee(4)),    0,
            0,                  1,  0,                  0,
            -sin(pose_ee(4)),   0,  cos(pose_ee(4)),    0,
            0,                  0,  0,                  1;

    Tz  <<  1,  0,                  0,                  0,
            0,  cos(pose_ee(3)),    -sin(pose_ee(3)),   0,
            0,  sin(pose_ee(3)),    cos(pose_ee(3)),    0,
            0,  0,                  0,                  1;

    T=Trans_matrix*Tx*Ty*Tz;

    VectorXd a1_g = T*a1_e;
    VectorXd a2_g = T*a2_e;
    VectorXd a3_g = T*a3_e;
    VectorXd a4_g = T*a4_e;
    VectorXd a5_g = T*a5_e;
    VectorXd a6_g = T*a6_e;
    VectorXd a7_g = T*a7_e;
    VectorXd a8_g = T*a8_e;


    double a1_g_array[3],a2_g_array[3],a3_g_array[3],a4_g_array[3],a5_g_array[3],a6_g_array[3],a7_g_array[3],a8_g_array[3];
    double b1_g_array[3],b2_g_array[3],b3_g_array[3],b4_g_array[3],b5_g_array[3],b6_g_array[3],b7_g_array[3],b8_g_array[3];


    for(int i=0;i<3;i++)
    {
        a1_g_array[i] = a1_g(i);
        a2_g_array[i] = a2_g(i);
        a3_g_array[i] = a3_g(i);
        a4_g_array[i] = a4_g(i);
        a5_g_array[i] = a5_g(i);
        a6_g_array[i] = a6_g(i);
        a7_g_array[i] = a7_g(i);
        a8_g_array[i] = a8_g(i);

        b1_g_array[i] = b1_g(i);
        b2_g_array[i] = b2_g(i);
        b3_g_array[i] = b3_g(i);
        b4_g_array[i] = b4_g(i);
        b5_g_array[i] = b5_g(i);
        b6_g_array[i] = b6_g(i);
        b7_g_array[i] = b7_g(i);
        b8_g_array[i] = b8_g(i);
    }

    double length_cable1 = CableLenCal(a1_g_array, b1_g_array, r[0]);
    double length_cable2 = CableLenCal(a2_g_array, b2_g_array, r[1]);
    double length_cable3 = CableLenCal(a3_g_array, b3_g_array, r[2]);
    double length_cable4 = CableLenCal(a4_g_array, b4_g_array, r[3]);
    double length_cable5 = CableLenCal(a5_g_array, b5_g_array, r[4]);
    double length_cable6 = CableLenCal(a6_g_array, b6_g_array, r[5]);
    double length_cable7 = CableLenCal(a7_g_array, b7_g_array, r[6]);
    double length_cable8 = CableLenCal(a8_g_array, b8_g_array, r[7]);

    VectorXd Angle_abl(8);
    Angle_abl(0) = 2.0*length_cable1/d(0);
    Angle_abl(1) = 2.0*length_cable2/d(1);
    Angle_abl(2) = 2.0*length_cable3/d(2);
    Angle_abl(3) = 2.0*length_cable4/d(3);
    Angle_abl(4) = 2.0*length_cable5/d(4);
    Angle_abl(5) = 2.0*length_cable6/d(5);
    Angle_abl(6) = 2.0*length_cable7/d(6);
    Angle_abl(7) = 2.0*length_cable8/d(7);

    return Angle_abl;
}
