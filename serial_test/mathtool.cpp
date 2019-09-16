#include"mathtool.h"

//Numberial diffrencial method.
double iNumDiff(double* y, double h)
{
    return 1.0 / (2.0*h)*(y[0] - 4 * y[1] + 3 * y[2]);
}
