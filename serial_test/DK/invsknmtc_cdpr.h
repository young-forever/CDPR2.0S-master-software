#ifndef INVSKNMTC_CDPR_H
#define INVSKNMTC_CDPR_H
#include<Eigen/Core>
#include<iomanip>
using namespace Eigen;

VectorXd InvsKnmtc_cdpr(VectorXd pose_ee, MatrixXd ibp, MatrixXd iep, VectorXd d, double *r);

#endif // INVSKNMTC_CDPR_H
