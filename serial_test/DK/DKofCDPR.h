#ifndef DKOFCDPR_H
#define DKOFCDPR_H

extern double ActualPose[6];        //declare end positon of CDPR.

void DKofCDPR(double *MotorAngle, double *Pose);
void optimizationtest(void);

#endif // DKOFCDPR_H
