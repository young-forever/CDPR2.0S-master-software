#include <cmath>
#include <iostream>
#include "cablelencal.h"
#include "iMath.h"

#define pi 3.14159265354

using namespace std;



/**
 * @brief CableLenCal:calculate the ideal length of cable
 * @param bp: position of bp;
 * @param ep: position of enp point;
 * @param radui: radui of pulley;
 * @return IdeCalLen: output calbe length.
 */

double CableLenCal(double *ep, double *bp, double radui)
{
    double r = radui;
    double a[3] = {0.0},b[3]={0.0};

    for(int i=0;i<3;i++)
    {
        a[i] = ep[i];
    }

    for(int i=0;i<3;i++)
    {
        b[i] = bp[i];
    }

    double ialpha = atan((a[2]-b[2])/abs(a[0]-b[0]));
    double Ox = b[0] + sign(a[0]-b[0])*r*cos(ialpha);
    double Oy = b[1];
    double Oz = b[2]+r*sin(ialpha);
    double O[3]={Ox,Oy,Oz};

    double l1 = pow(pow(O[0]-a[0],2)+pow(O[1]-a[1],2)+pow(O[2]-a[2],2),0.5);
    double l2 = pow(l1*l1-r*r,0.5);

    double theta1 = atan(l2/r);
    double theta2 = atan(fabs(a[1]-O[1])/pow(pow(a[0]-O[0],2)+pow(a[2]-O[2],2),0.5));
    double theta3 = pi - (theta1 - theta2);
    double c = theta3*r; //Arc length;

    double IdeCalLen = l2 + c;

    return IdeCalLen;
}
