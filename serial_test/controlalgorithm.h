#ifndef Control_algorithm_H
#define Control_algorithm_H
#include<Eigen/Dense>
using namespace Eigen;


VectorXd spline3ToM(int num, VectorXd angle, VectorXf t, float v0, float vt);
	
Vector2f Calculate_MotVand_Angle(int Mot_Num, int n, float Init_interval , short Data_i, \
                             VectorXf T_Array, MatrixXd M_Matrix, MatrixXd anglenode);
//void Data_V_S_Compute(unsigned char Motor_num,int Data_Num_i,int Max_Data_num);
#endif

