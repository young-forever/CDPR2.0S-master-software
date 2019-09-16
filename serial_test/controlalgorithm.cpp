#include"controlalgorithm.h"
#include<stdio.h>
#include<stdlib.h>
#include<Eigen/Dense>
#include<iostream>
using namespace Eigen;
using namespace std;

/******

******/
VectorXd  spline3ToM( int num,VectorXd angle,VectorXf t,float v0,float vt)//
{
    int i,n;

    VectorXd h(num),f(num),Mat_P(num),lamda(num),u(num);

    MatrixXd K(num,num),inver_K(num,num);

    n=num;//

    for(i=0;i<(n-1);i++)
    {
        h(i) = t(i+1)-t(i);
        f(i) = (angle(i+1) -angle(i))/h(i);
    }

    Mat_P(0)=6.0/h(0)*(f(0)-v0);

    for(i=0;i<(n-2);i++)//
    {
        lamda(i)=h(i)/(h(i+1)+h(i));
        u(i) = 1-lamda(i);
        Mat_P(i+1)=6*(f(i+1)-f(i))/(h(i+1)+h(i));
    }
    Mat_P(n-1) = 6*(vt-f(n-2))/h(n-2);

    K=MatrixXd::Zero(n,n);

    K(0,0) =2;K(0,1)=1;

    for(i=1;i<n;i++)//
    {
        K(i,i-1)=lamda(i-1);
        K(i,i)=2;
        if(i<(n-1))
            K(i,i+1)=u(i-1);
    }
    K(n-1,n-2)=1;

    inver_K=K.inverse();

    VectorXd Mat_M;

    Mat_M = inver_K*Mat_P;
    return Mat_M;
}



//void Data_V_S_Compute(unsigned char Motor_num,int Data_Num_i,int Max_Data_num)
//{

//    if(Data_Num_i==(Max_Data_num-1))
//    {

//        if( Data_num[Motor_num-1]<Max_Data_num)
//            Calculate_MotVand_Angle(Motor_num,Data_Row,Inteval_Motor*Motor_num ,Max_Data_num-2,1);
//        else
//            Calculate_MotVand_Angle(Motor_num,Data_Row,Inteval_Motor*Motor_num ,Max_Data_num-1,0);
//    }
//    else
//    {
//        Calculate_MotVand_Angle(Motor_num,Data_Row,Inteval_Motor*Motor_num ,Data_Num_i,0);
//    }
//}


Vector2f Calculate_MotVand_Angle(int Mot_Num, int n, float Init_interval , short Data_i, \
                             VectorXf T_Array, MatrixXd M_Matrix, MatrixXd anglenode)
{
    float time,V1,V2,V3,V4,S1,S2,S3,S4,step;//
    int j=0;

    if(Data_i>0)
        time=Init_interval*Data_i;
    else
        time=0;

    if(time> T_Array(n-1))
    {
        time=T_Array(n-1);
        j=n-2;
    }
    else
        for(j=0;j<(n-1);j++)
        {
            if((time>= T_Array(j)) && (time< T_Array(j+1)))
            {
                break;
            }

            if(j==(n-2))
            {
                if(time==T_Array(j+1))
                    break;
            }

        }

    step= T_Array(j+1)- T_Array(j);

    V1=-M_Matrix(Mot_Num-1,j)*(pow( T_Array(j+1)-time,2))/(2.0*step);
    V2=M_Matrix(Mot_Num-1,j+1)*(pow(time- T_Array(j),2))/(2.0*step);
    V3=(anglenode(Mot_Num-1,j+1)-anglenode(Mot_Num-1,j))/step;
    V4=-(M_Matrix(Mot_Num-1,j+1)-M_Matrix(Mot_Num-1,j))*step/6.0;
    float Motor_Vel=V1+V2+V3+V4;//
    //
    S1=M_Matrix(Mot_Num-1,j)*(pow( T_Array(j+1)-time,3))/(6.0*step);
    S2=M_Matrix(Mot_Num-1,j+1)*(pow(time- T_Array(j),3))/(6.0*step);
    S3=(anglenode(Mot_Num-1,j)-M_Matrix(Mot_Num-1,j)*(pow(step,2))/6.0)*( T_Array(j+1)-time)/step;
    S4=(anglenode(Mot_Num-1,j+1)-M_Matrix(Mot_Num-1,j+1)*(pow(step,2))/6.0)*(time- T_Array(j))/step;

    float Motor_Angle=S1+S2+S3+S4;

    Vector2f P_msg;
    P_msg<<Motor_Vel,Motor_Angle;
    return P_msg;
}
